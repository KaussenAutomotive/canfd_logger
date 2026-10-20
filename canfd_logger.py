"""CAN/CAN-FD-Logger fuer PEAK PCAN und ETAS BOA, Python 3.10+.

Installation:
    py -m pip install "python-can>=4.6,<5"
    PEAK PCAN-Basic und den passenden PEAK-USB-Treiber installieren.
    PCANBasic.dll muss zur Python-Architektur (32/64 Bit) passen.
    Fuer ETAS: Windows, ETAS BOA V2 und passende Geraetetreiber installieren.
    BOA-DLLs muessen zur Python-Architektur passen; python-can sucht u.a. in
    C:\\Program Files\\ETAS\\BOA_V2\\Bin\\x64\\Dll\\Framework
    (32 Bit: Program Files (x86), Bin\\Win32). Nur der gewaehlte Treiber
    wird geladen; ETAS benoetigt weder PCAN-Basic noch PEAK-Hardware.
Start:
    py canfd_logger.py

CAN FD erfordert FD-faehige Hardware und Firmware.
ETAS: Treiber "ETAS BOA" waehlen, "ETAS-Kanaele suchen" nutzen oder die
vollstaendige BOA-URI eingeben (z.B. ETAS://ETH/ES910:abcd/CAN:1).
Die Suche prueft nicht, ob ein Kanal CAN FD unterstuetzt.
Das python-can-ETAS-Backend unterstuetzt kein Listen-only: aktive Teilnahme
mit ACKs, aber das Skript sendet keine Datenframes. ETAS-Sample-Points sind
nominal/Daten jeweils 80 % (Backend-Vorgabe); BOA kann Timing anpassen.
ETAS liefert im verwendeten Backend keine ESI-/Error-Frame-/Queue-Events;
CSV-Werte fuer ESI/Error sind dort keine verlaessliche Fehlerdiagnose.
Controller-Fehlerstatus wird zusaetzlich regelmaessig abgefragt.
Busstatus: Fehlerzaehler-Warnungen (PEAK BUSLIGHT/BUSHEAVY, Error-Passive)
werden nur angezeigt. Pufferueberlaeufe (moeglicher Frameverlust) werden
gezaehlt und beim Log-Ende gemeldet. Nur BUS-OFF trennt die Verbindung.
PEAK: Listen-only ist voreingestellt: keine Sendeframes, keine ACKs. Auf einem Bus
mit nur einem Sender wird daher ein weiterer aktiver Teilnehmer benoetigt.
PEAK-CAN-FD-Timing: 80 MHz, Sample Point nominal 80 %, Datenphase 75 %.
Bei abweichendem Netzwerktiming muessen diese Werte angepasst werden.

Bedienung: "Verbinden" oeffnet das Interface (Live-Anzeige laeuft sofort),
"Start Log"/"Stopp Log" schalten nur die CSV-Aufzeichnung, ohne den Bus
neu zu initialisieren. Mehrere Logs pro Verbindung sind moeglich; nach jedem
Log wird ein neuer Dateiname vorgeschlagen. "Trennen" beendet einen
laufenden Log und gibt das Interface frei. BUS-OFF und Treiberfehler trennen die
Verbindung; CSV-Fehler stoppen nur den Log.

CSV: Semikolon, UTF-8 mit BOM; jede waehrend des Logs empfangene Nachricht
wird gespeichert.
adapter_timestamp_s ist der unveraenderte python-can/Adapter-Zeitstempel,
nicht zwingend Unix-Zeit (abhaengig vom Backend und optionalem uptime-Paket).
relative_s bezieht sich auf die erste Nachricht des Logs (Live-Anzeige: seit
Verbinden). dlc_bytes bezeichnet die
dekodierte Laenge, nicht den 4-Bit-DLC-Code; bei RTR die angeforderte Laenge.
Die Live-Anzeige ist begrenzt und darf Frames auslassen, nicht die CSV.
Erkannte Fehler werden gemeldet; eine verlustfreie Aufnahme bei beliebiger Buslast wird nicht garantiert.
CSV wird alle 0,5 Sekunden und bei Stopp/Trennen geleert/geschlossen.
Bestehende Dateien werden niemals ueberschrieben.
Standard-Logordner ist der Windows-Downloads-Ordner (auch wenn verschoben);
Fallback: ~/Downloads, sonst das aktuelle Arbeitsverzeichnis.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import can


DISPLAY_QUEUE_SIZE = 5000
DISPLAY_ROWS = 1500
BACKENDS = {"PEAK PCAN": "pcan", "ETAS BOA": "etas"}
CSV_COLUMNS = (
    "adapter_timestamp_s", "relative_s", "channel", "id_hex",
    "extended", "fd", "brs", "esi", "rtr", "error", "direction",
    "dlc_bytes", "data_hex",
)


def downloads_dir() -> Path:
    candidates: list[Path] = []
    try:
        import ctypes
        import uuid

        # FOLDERID_Downloads; honours a relocated Downloads folder.
        folder_id = uuid.UUID("{374DE290-123F-4565-9164-39C4925E467B}")
        guid = (ctypes.c_byte * 16).from_buffer_copy(folder_id.bytes_le)
        raw_path = ctypes.c_wchar_p()
        shell32 = ctypes.windll.shell32
        if shell32.SHGetKnownFolderPath(ctypes.byref(guid), 0, None, ctypes.byref(raw_path)) == 0:
            try:
                candidates.append(Path(raw_path.value or ""))
            finally:
                ctypes.windll.ole32.CoTaskMemFree(raw_path)
    except (AttributeError, OSError, ValueError):
        pass
    candidates.append(Path.home() / "Downloads")
    for candidate in candidates:
        if str(candidate) and candidate.is_dir():
            return candidate
    return Path.cwd()


@dataclass(frozen=True)
class Settings:
    channel: str
    fd: bool
    nominal_bitrate: int
    data_bitrate: int
    listen_only: bool
    interface: str = "pcan"


@dataclass(frozen=True)
class Progress:
    frames: int
    display_dropped: int
    connected_s: float
    logging: bool
    log: tuple[int, int, float] | None  # frames, payload bytes, seconds
    bus_state: str = "OK"
    overruns: int = 0


@dataclass(frozen=True)
class Notice:
    text: str = ""
    progress: Progress | None = None
    event: str = ""  # connected, log_started, log_stopped, disconnected
    error: bool = False


@dataclass(frozen=True)
class ScanResult:
    channels: tuple[str, ...] = ()
    error: str = ""


def open_bus(settings: Settings) -> can.BusABC:
    import can

    if settings.interface == "etas":
        if settings.listen_only:
            raise ValueError("ETAS BOA unterstuetzt in python-can kein Listen-only.")
        from can.interfaces.etas import EtasBus

        return EtasBus(
            channel=settings.channel, fd=settings.fd,
            bitrate=settings.nominal_bitrate, data_bitrate=settings.data_bitrate,
            receive_own_messages=False,
        )
    if settings.interface != "pcan":
        raise ValueError(f"Unbekannter Treiber: {settings.interface}")
    from can.interfaces.pcan import PcanBus

    state = can.BusState.PASSIVE if settings.listen_only else can.BusState.ACTIVE
    if settings.fd:
        timing = can.BitTimingFd.from_sample_point(
            f_clock=80_000_000,
            nom_bitrate=settings.nominal_bitrate,
            nom_sample_point=80.0,
            data_bitrate=settings.data_bitrate,
            data_sample_point=75.0,
        )
        return PcanBus(
            channel=settings.channel, state=state, fd=True, timing=timing,
            receive_own_messages=False,
        )
    return PcanBus(
        channel=settings.channel, state=state, fd=False,
        bitrate=settings.nominal_bitrate, receive_own_messages=False,
    )


PCAN_BUSOFF = 0x10
PCAN_DATA_LOSS = 0x02 | 0x40  # OVERRUN (controller), QOVERRUN (receive queue)
PCAN_WARNINGS = {
    0x04: "Fehlerzaehler-Warnung (BUSLIGHT)",
    0x08: "Fehlerzaehler hoch (BUSHEAVY)",
    0x40000: "Error-Passive",
}
# 0x01/0x80: TX queue full, 0x20: RX queue empty; irrelevant for receiving.
PCAN_KNOWN = PCAN_DATA_LOSS | 0x04 | 0x08 | 0x40000 | 0x01 | 0x20 | 0x80


@dataclass(frozen=True)
class BusHealth:
    text: str = "OK"
    data_loss: bool = False


def check_bus_status(bus: can.BusABC, interface: str) -> BusHealth:
    """Raises only for bus-off; warnings and overruns are reported, not fatal."""
    if interface == "pcan":
        from can.interfaces.pcan import PcanBus

        if not isinstance(bus, PcanBus):
            return BusHealth()
        status = int(bus.status())
        if status & PCAN_BUSOFF:
            raise RuntimeError(
                f"PCAN-Status 0x{status:05X}: BUS-OFF. Keine Kommunikation moeglich. "
                "Verkabelung, Terminierung und Bitraten pruefen."
            )
        texts = [text for bit, text in PCAN_WARNINGS.items() if status & bit]
        data_loss = bool(status & PCAN_DATA_LOSS)
        if data_loss:
            texts.append("Pufferueberlauf, Frames verloren")
        if status & ~PCAN_KNOWN:
            texts.append(f"Status 0x{status:05X}")
        return BusHealth(", ".join(texts) or "OK", data_loss)
    if interface == "etas":
        import ctypes
        from can.interfaces.etas import EtasBus
        from can.interfaces.etas.boa import (
            OCI_CANControllerStatus, OCI_CAN_STATE_BUSOFF, OCI_CAN_STATE_ERRLIMIT,
            OCI_CAN_STATE_PASSIVE, OCI_GetCANControllerStatus,
        )

        if not isinstance(bus, EtasBus):
            return BusHealth()
        controller_status = OCI_CANControllerStatus()
        OCI_GetCANControllerStatus(bus.ctrl, ctypes.byref(controller_status))
        state = controller_status.stateCode
        if state & OCI_CAN_STATE_BUSOFF:
            raise RuntimeError(
                f"ETAS-Status 0x{state:05X}: BUS-OFF. Keine Kommunikation moeglich. "
                "Verkabelung, Terminierung und Bitraten pruefen."
            )
        texts = []
        if state & OCI_CAN_STATE_ERRLIMIT:
            texts.append("Fehlerzaehler-Warnung")
        if state & OCI_CAN_STATE_PASSIVE:
            texts.append("Error-Passive")
        return BusHealth(", ".join(texts) or "OK")
    raise ValueError(f"Unbekannter Treiber: {interface}")


def scan_etas(results: queue.Queue[ScanResult]) -> None:
    try:
        # Generic discovery can hide import failures and return partial timeout results.
        from can.interfaces.etas import EtasBus

        configs = EtasBus._detect_available_configs()
        channels = tuple(sorted({str(config["channel"]) for config in configs}))
        results.put(ScanResult(channels=channels))
    except Exception as exc:
        results.put(ScanResult(error=f"{type(exc).__name__}: {exc}"))


def csv_row(message: can.Message, first_timestamp: float, channel: str) -> tuple[str, ...]:
    return (
        f"{message.timestamp:.6f}",
        f"{message.timestamp - first_timestamp:.6f}",
        channel,
        f"{message.arbitration_id:08X}" if message.is_extended_id
        else f"{message.arbitration_id:03X}",
        str(int(message.is_extended_id)),
        str(int(message.is_fd)),
        str(int(message.bitrate_switch)),
        str(int(message.error_state_indicator)),
        str(int(message.is_remote_frame)),
        str(int(message.is_error_frame)),
        "RX" if message.is_rx else "TX",
        str(message.dlc),
        message.data.hex(" ").upper(),
    )


class CsvLog:
    def __init__(self, path: Path, channel: str) -> None:
        self.path = path
        self.channel = channel
        self.file = path.open("x", newline="", encoding="utf-8-sig")
        try:
            self.writer = csv.writer(self.file, delimiter=";")
            self.writer.writerow(CSV_COLUMNS)
            self.file.flush()
        except Exception:
            self.file.close()
            raise
        self.frames = 0
        self.payload_bytes = 0
        self.first_timestamp: float | None = None
        self.started = time.monotonic()
        self.last_flush = self.started

    def write(self, message: can.Message) -> None:
        if self.first_timestamp is None:
            self.first_timestamp = message.timestamp
        self.writer.writerow(csv_row(message, self.first_timestamp, self.channel))
        self.frames += 1
        self.payload_bytes += len(message.data)

    def flush(self) -> None:
        self.file.flush()
        self.last_flush = time.monotonic()

    def close(self) -> None:
        self.file.close()


class CanConnection(threading.Thread):
    """Owns the bus; CSV logging is switched on/off via commands."""

    def __init__(
        self,
        settings: Settings,
        rows: queue.Queue[tuple[str, ...]],
        notices: queue.Queue[Notice],
    ) -> None:
        super().__init__(name="CAN-Connection", daemon=False)
        self.settings = settings
        self.rows = rows
        self.notices = notices
        self.commands: queue.Queue[Path | None] = queue.Queue()
        self.disconnect_event = threading.Event()
        self.started = time.monotonic()
        self.frames = 0
        self.display_dropped = 0
        self.log: CsvLog | None = None
        self.last_log: tuple[int, int, float] | None = None
        self.health = BusHealth()
        self.overruns = 0
        self.log_overruns = 0

    def start_log(self, path: Path) -> None:
        self.commands.put(path)

    def stop_log(self) -> None:
        self.commands.put(None)

    def disconnect(self) -> None:
        self.disconnect_event.set()

    def progress(self) -> Progress:
        now = time.monotonic()
        if self.log is not None:
            logged = (self.log.frames, self.log.payload_bytes, now - self.log.started)
        else:
            logged = self.last_log
        return Progress(
            self.frames, self.display_dropped, now - self.started,
            self.log is not None, logged, self.health.text, self.overruns,
        )

    def update_health(self, health: BusHealth) -> None:
        # Overrun flags may stay set; count rising edges, not every poll.
        if health.data_loss and not self.health.data_loss:
            self.overruns += 1
            if self.log is not None:
                self.log_overruns += 1
        if health.text != self.health.text:
            self.notices.put(Notice(text=f"Busstatus: {health.text}"))
        self.health = health

    def close_log(self, reason: str = "") -> tuple[str, bool]:
        """Closes the active log; returns (message, close_failed)."""
        log = self.log
        if log is None:
            return "", False
        self.log = None
        self.last_log = (log.frames, log.payload_bytes, time.monotonic() - log.started)
        messages = [reason] if reason else []
        failed = False
        try:
            log.close()
        except OSError as exc:
            messages.append(f"CSV konnte nicht geschlossen werden: {exc}")
            failed = True
        messages.append(f"Log beendet: {log.frames:,} Frames in {log.path}")
        if self.log_overruns:
            messages.append(
                f"ACHTUNG: {self.log_overruns}x Pufferueberlauf waehrend des Logs "
                "gemeldet - Frames koennen fehlen."
            )
        return "\n".join(messages), failed

    def handle_commands(self) -> None:
        while True:
            try:
                path = self.commands.get_nowait()
            except queue.Empty:
                return
            if path is None:
                if self.log is not None:
                    text, failed = self.close_log()
                    self.notices.put(Notice(
                        text=text, event="log_stopped", error=failed,
                        progress=self.progress(),
                    ))
            elif self.log is None:
                try:
                    self.log = CsvLog(path, self.settings.channel)
                    self.log_overruns = 0
                except Exception as exc:
                    self.notices.put(Notice(
                        text=f"Log konnte nicht gestartet werden: "
                        f"{type(exc).__name__}: {exc}",
                        event="log_stopped", error=True,
                    ))
                else:
                    self.notices.put(Notice(text=f"Log laeuft: {path}", event="log_started"))

    def fail_log(self, exc: Exception) -> None:
        text, _ = self.close_log(
            f"CSV-Schreibfehler, Log gestoppt (Verbindung bleibt bestehen): "
            f"{type(exc).__name__}: {exc}"
        )
        self.notices.put(Notice(
            text=text, event="log_stopped", error=True, progress=self.progress(),
        ))

    def run(self) -> None:
        first_timestamp: float | None = None
        errors: list[str] = []
        final_text = ""
        bus = None
        try:
            bus = open_bus(self.settings)
            self.started = time.monotonic()
            self.notices.put(Notice(
                text=f"Verbunden: {self.settings.interface.upper()} {self.settings.channel}",
                event="connected",
            ))
            last_report = last_status = time.monotonic()
            while not self.disconnect_event.is_set():
                self.handle_commands()
                message = bus.recv(timeout=0.05)
                if message is not None:
                    self.frames += 1
                    if first_timestamp is None:
                        first_timestamp = message.timestamp
                    if self.log is not None:
                        try:
                            self.log.write(message)
                        except (OSError, csv.Error) as exc:
                            self.fail_log(exc)
                    try:
                        self.rows.put_nowait(
                            csv_row(message, first_timestamp, self.settings.channel)
                        )
                    except queue.Full:
                        # Only the live view is lossy; CSV writes happen before this.
                        self.display_dropped += 1
                now = time.monotonic()
                if self.log is not None and now - self.log.last_flush >= 0.5:
                    try:
                        self.log.flush()
                    except OSError as exc:
                        self.fail_log(exc)
                if now - last_status >= 0.25:
                    self.update_health(check_bus_status(bus, self.settings.interface))
                    last_status = now
                if now - last_report >= 0.25:
                    self.notices.put(Notice(progress=self.progress()))
                    last_report = now
        except Exception as exc:
            # The worker boundary must report driver and filesystem failures to Tk.
            errors.append(f"{type(exc).__name__}: {exc}")
        finally:
            if self.log is not None:
                final_text, close_failed = self.close_log(
                    "Log wegen Verbindungsfehler beendet." if errors else ""
                )
                if close_failed:
                    errors.append(final_text)
                    final_text = ""
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception as exc:
                    errors.append(f"CAN-Treiber konnte nicht freigegeben werden: {exc}")
            lines = [text for text in (*errors, final_text) if text]
            self.notices.put(Notice(
                text="\n".join(dict.fromkeys(lines + ["Getrennt."])),
                progress=self.progress(),
                event="disconnected",
                error=bool(errors),
            ))


class LoggerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.connection: CanConnection | None = None
        self.connected = False
        self.logging = False
        self.log_pending = False
        self.disconnecting = False
        self.closing = False
        self.rows: queue.Queue[tuple[str, ...]] = queue.Queue(DISPLAY_QUEUE_SIZE)
        self.notices: queue.Queue[Notice] = queue.Queue()
        self.scan_results: queue.Queue[ScanResult] = queue.Queue()
        self.scan_worker: threading.Thread | None = None
        self.backend = tk.StringVar(value="PEAK PCAN")
        self.etas_channels: tuple[str, ...] = ()
        self.saved_channels = {"PEAK PCAN": "PCAN_USBBUS1", "ETAS BOA": ""}
        self.previous_backend = "PEAK PCAN"
        self.peak_listen_only = True
        self.config_widgets: list[tuple[ttk.Widget, str]] = []
        self.log_widgets: list[ttk.Widget] = []
        self.channel = tk.StringVar(value="PCAN_USBBUS1")
        self.mode = tk.StringVar(value="CAN FD")
        self.nominal = tk.StringVar(value="500000")
        self.data_rate = tk.StringVar(value="2000000")
        self.listen_only = tk.BooleanVar(value=True)
        self.show_live = tk.BooleanVar(value=True)
        self.path = tk.StringVar(value=str(self.default_path()))
        self.status = tk.StringVar(value="Getrennt. Interface waehlen und verbinden.")
        self.driver_hint = tk.StringVar()
        self.statistics = tk.StringVar(value="Nicht verbunden")
        self.root.title("PEAK / ETAS BOA CAN FD Logger")
        self.root.geometry("1180x720")
        self.root.minsize(850, 500)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.build_ui()
        self.update_backend()
        self.root.after(100, self.poll)

    @staticmethod
    def default_path() -> Path:
        return downloads_dir() / f"canfd_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"

    def build_ui(self) -> None:
        panel = ttk.Frame(self.root, padding=10)
        panel.pack(fill="both", expand=True)
        config = ttk.LabelFrame(panel, text="Interface", padding=8)
        config.pack(fill="x")
        driver_panel = ttk.Frame(config)
        driver_panel.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        ttk.Label(driver_panel, text="Treiber:").pack(side="left")
        backend_box = ttk.Combobox(
            driver_panel, textvariable=self.backend, values=tuple(BACKENDS),
            state="readonly", width=18,
        )
        backend_box.pack(side="left", padx=8)
        backend_box.bind("<<ComboboxSelected>>", lambda _event: self.update_backend())
        self.scan_button = ttk.Button(
            driver_panel, text="ETAS-Kanaele suchen", command=self.start_scan,
        )
        self.scan_button.pack(side="left")
        self.disconnect_button = ttk.Button(
            driver_panel, text="Trennen", command=self.disconnect,
        )
        self.disconnect_button.pack(side="right")
        self.connect_button = ttk.Button(
            driver_panel, text="Verbinden", command=self.connect,
        )
        self.connect_button.pack(side="right", padx=6)
        self.connection_label = ttk.Label(driver_panel, text="Getrennt")
        self.connection_label.pack(side="right", padx=12)
        self.config_widgets.append((backend_box, "readonly"))
        fields = (
            ("Kanal / BOA-URI", self.channel,
             tuple(f"PCAN_USBBUS{i}" for i in range(1, 17)), "normal"),
            ("Modus", self.mode, ("CAN FD", "Klassisches CAN"), "readonly"),
            ("Nominal [bit/s]", self.nominal,
             ("125000", "250000", "500000", "1000000"), "normal"),
            ("Datenphase [bit/s]", self.data_rate,
             ("500000", "1000000", "2000000", "4000000", "5000000", "8000000"),
             "normal"),
        )
        for column, (label, variable, values, state) in enumerate(fields):
            ttk.Label(config, text=label).grid(row=1, column=column, sticky="w", padx=4)
            box = ttk.Combobox(
                config, textvariable=variable, values=values, state=state, width=22,
            )
            box.grid(row=2, column=column, sticky="ew", padx=4, pady=4)
            if column == 0:
                self.channel_box = box
            config.columnconfigure(column, weight=1)
            self.config_widgets.append((box, state))
        self.passive = ttk.Checkbutton(
            config, text="Listen-only (keine ACKs)", variable=self.listen_only,
        )
        self.passive.grid(row=3, column=0, columnspan=2, sticky="w", padx=4)
        ttk.Label(
            config, textvariable=self.driver_hint, wraplength=650,
        ).grid(row=3, column=2, columnspan=2, sticky="w", padx=4)
        log_frame = ttk.LabelFrame(panel, text="CSV-Log", padding=8)
        log_frame.pack(fill="x", pady=10)
        ttk.Label(log_frame, text="Datei:").pack(side="left")
        path_entry = ttk.Entry(log_frame, textvariable=self.path)
        path_entry.pack(side="left", fill="x", expand=True, padx=8)
        browse = ttk.Button(log_frame, text="Auswaehlen ...", command=self.browse)
        browse.pack(side="left")
        self.log_widgets.extend((path_entry, browse))
        self.start_button = ttk.Button(log_frame, text="Start Log", command=self.start)
        self.start_button.pack(side="left", padx=(12, 0))
        self.stop_button = ttk.Button(log_frame, text="Stopp Log", command=self.stop)
        self.stop_button.pack(side="left", padx=6)
        controls = ttk.Frame(panel)
        controls.pack(fill="x", pady=(0, 8))
        ttk.Button(controls, text="Anzeige leeren", command=self.clear_view).pack(side="left")
        ttk.Checkbutton(
            controls, text="Live-Anzeige (letzte 1500 Frames)", variable=self.show_live,
        ).pack(side="left", padx=12)
        ttk.Label(panel, textvariable=self.statistics).pack(anchor="w", pady=(0, 6))
        table = ttk.Frame(panel)
        table.pack(fill="both", expand=True)
        columns = ("time", "channel", "id", "format", "flags", "length", "data")
        self.tree = ttk.Treeview(table, columns=columns, show="headings", selectmode="browse")
        for column, label, width in (
            ("time", "Seit Verbinden [s]", 120), ("channel", "Kanal", 130),
            ("id", "ID (hex)", 90), ("format", "Format", 75),
            ("flags", "Flags", 150), ("length", "Bytes / RTR-Laenge", 120),
            ("data", "Daten (hex)", 600),
        ):
            self.tree.heading(column, text=label)
            self.tree.column(column, width=width, minwidth=60, stretch=column == "data")
        vertical = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        horizontal = ttk.Scrollbar(table, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)
        ttk.Label(
            panel, textvariable=self.status, wraplength=1100,
        ).pack(anchor="w", pady=(8, 0))

    def update_backend(self) -> None:
        backend = self.backend.get()
        if backend != self.previous_backend:
            self.saved_channels[self.previous_backend] = self.channel.get()
            if self.previous_backend == "PEAK PCAN":
                self.peak_listen_only = self.listen_only.get()
            self.channel.set(self.saved_channels[backend])
            self.previous_backend = backend
            self.listen_only.set(self.peak_listen_only if backend == "PEAK PCAN" else False)
        etas = backend == "ETAS BOA"
        self.channel_box.configure(values=(
            self.etas_channels if etas else tuple(f"PCAN_USBBUS{i}" for i in range(1, 17))
        ))
        self.driver_hint.set(
            "ETAS BOA: aktive Teilnahme mit ACKs, kein Listen-only. "
            "Sample Points: 80 % / 80 %. ESI-/Error-/Queue-Events nicht verfuegbar."
            if etas else "PEAK CAN FD: 80 MHz | Sample Points: nominal 80 %, Daten 75 %"
        )
        self.update_controls()

    def update_controls(self) -> None:
        idle = self.connection is None and self.scan_worker is None
        etas = self.backend.get() == "ETAS BOA"
        busy = self.log_pending or self.disconnecting
        for widget, state in self.config_widgets:
            widget.configure(state=state if idle else "disabled")
        self.passive.configure(state="normal" if idle and not etas else "disabled")
        self.scan_button.configure(state="normal" if idle and etas else "disabled")
        self.connect_button.configure(state="normal" if idle else "disabled")
        self.disconnect_button.configure(
            state="normal" if self.connection is not None and not self.disconnecting
            else "disabled"
        )
        for widget in self.log_widgets:
            widget.configure(state="disabled" if self.logging or busy else "normal")
        self.start_button.configure(
            state="normal" if self.connected and not self.logging and not busy
            else "disabled"
        )
        self.stop_button.configure(
            state="normal" if self.logging and not busy else "disabled"
        )
        if self.disconnecting:
            text = "Trenne ..."
        elif self.connected:
            text = "Verbunden, Log laeuft" if self.logging else "Verbunden"
        elif self.connection is not None:
            text = "Verbinde ..."
        else:
            text = "Getrennt"
        self.connection_label.configure(text=text)

    def start_scan(self) -> None:
        if self.connection is not None or self.scan_worker is not None:
            return
        self.status.set("Suche ETAS-BOA-Kanaele ...")
        self.scan_worker = threading.Thread(
            target=scan_etas, args=(self.scan_results,), name="ETAS-Discovery", daemon=True,
        )
        self.update_controls()
        self.scan_worker.start()

    def browse(self) -> None:
        selected = filedialog.asksaveasfilename(
            parent=self.root, title="Neue CSV-Datei waehlen (kein Ueberschreiben)",
            defaultextension=".csv", filetypes=[("CSV-Dateien", "*.csv")],
            initialdir=str(downloads_dir()), initialfile=self.default_path().name,
        )
        if selected:
            self.path.set(selected)

    def get_settings(self) -> Settings:
        interface = BACKENDS[self.backend.get()]
        channel = self.channel.get().strip()
        if interface == "pcan":
            channel = channel.upper()
            if channel not in {f"PCAN_USBBUS{i}" for i in range(1, 17)}:
                raise ValueError("PEAK-Kanal muss PCAN_USBBUS1 bis PCAN_USBBUS16 sein.")
        else:
            if not channel.startswith("ETAS://") or "/CAN:" not in channel:
                raise ValueError("Bitte eine vollstaendige ETAS-BOA-URI waehlen/eingeben.")
            if self.listen_only.get():
                raise ValueError("ETAS BOA unterstuetzt in python-can kein Listen-only.")
        nominal = int(self.nominal.get())
        fd = self.mode.get() == "CAN FD"
        data_rate = int(self.data_rate.get()) if fd else nominal
        if not 1 <= nominal <= 1_000_000:
            raise ValueError("Nominal-Bitrate muss zwischen 1 und 1000000 bit/s liegen.")
        if fd and not nominal <= data_rate <= 8_000_000:
            raise ValueError("Datenbitrate muss >= nominal und <= 8000000 bit/s sein.")
        return Settings(channel, fd, nominal, data_rate, self.listen_only.get(), interface)

    def get_log_path(self) -> Path:
        if not self.path.get().strip():
            raise ValueError("Bitte eine CSV-Datei angeben.")
        path = Path(self.path.get().strip()).expanduser().resolve()
        if path.exists():
            raise ValueError("Datei existiert bereits. Bitte eine neue CSV-Datei waehlen.")
        if not path.parent.is_dir():
            raise ValueError("Der Zielordner existiert nicht.")
        return path

    def connect(self) -> None:
        if self.connection is not None or self.scan_worker is not None:
            return
        try:
            import can
            settings = self.get_settings()
        except ImportError:
            messagebox.showerror(
                "Abhaengigkeit fehlt",
                'Bitte installieren:\npy -m pip install "python-can>=4.6,<5"',
                parent=self.root,
            )
            return
        except ValueError as exc:
            messagebox.showerror("Ungueltige Einstellungen", str(exc), parent=self.root)
            return
        self.clear_view()
        self.discard_pending_rows()
        self.statistics.set("Verbinde ...")
        self.status.set(f"Verbinde mit {self.backend.get()} ...")
        self.connection = CanConnection(settings, self.rows, self.notices)
        self.update_controls()
        self.connection.start()

    def disconnect(self) -> None:
        if self.connection is not None and not self.disconnecting:
            self.disconnecting = True
            self.status.set("Trenne Interface und schliesse ggf. CSV ...")
            self.connection.disconnect()
            self.update_controls()

    def start(self) -> None:
        if self.connection is None or not self.connected or self.logging or self.log_pending:
            return
        try:
            path = self.get_log_path()
        except (ValueError, OSError) as exc:
            messagebox.showerror("Ungueltige CSV-Datei", str(exc), parent=self.root)
            return
        self.log_pending = True
        self.status.set(f"Starte Log: {path}")
        self.connection.start_log(path)
        self.update_controls()

    def stop(self) -> None:
        if self.connection is not None and self.logging and not self.log_pending:
            self.log_pending = True
            self.status.set("Stoppe Log und schliesse CSV ...")
            self.connection.stop_log()
            self.update_controls()

    def clear_view(self) -> None:
        children = self.tree.get_children()
        if children:
            self.tree.delete(*children)

    def discard_pending_rows(self) -> None:
        while True:
            try:
                self.rows.get_nowait()
            except queue.Empty:
                break

    def show_row(self, row: tuple[str, ...]) -> None:
        flags = [
            name for index, name in ((6, "BRS"), (7, "ESI"), (8, "RTR"), (9, "ERR"))
            if row[index] == "1"
        ]
        flags.insert(0, row[10])
        self.tree.insert("", "end", values=(
            row[1], row[2], row[3],
            ("FD" if row[5] == "1" else "CAN") + (" EXT" if row[4] == "1" else " STD"),
            " ".join(flags), row[11], row[12],
        ))

    def show_progress(self, progress: Progress) -> None:
        text = (
            f"Empfangen: {progress.frames:,} Frames | Verbunden: {progress.connected_s:.1f} s"
        )
        if progress.log is not None:
            frames, payload_bytes, seconds = progress.log
            state = "aktiv" if progress.logging else "zuletzt"
            text += (
                f" | Log {state}: {frames:,} Frames, {payload_bytes:,} Bytes, {seconds:.1f} s"
            )
        else:
            text += " | Log: aus"
        text += f" | Nur Live-Anzeige verworfen: {progress.display_dropped:,}"
        text += f" | Bus: {progress.bus_state}"
        if progress.overruns:
            text += f" | Pufferueberlaeufe: {progress.overruns}"
        self.statistics.set(text)

    def handle_scan_result(self, scan_result: ScanResult) -> None:
        self.scan_worker = None
        if scan_result.error:
            self.status.set("ETAS-Kanalsuche fehlgeschlagen.")
            self.update_backend()
            messagebox.showerror(
                "ETAS BOA", scan_result.error
                + "\n\nETAS BOA V2, Geraetetreiber und DLL-Architektur pruefen.",
                parent=self.root,
            )
            return
        self.etas_channels = scan_result.channels
        if self.channel.get() not in self.etas_channels:
            self.channel.set(self.etas_channels[0] if self.etas_channels else "")
        self.status.set(
            f"{len(self.etas_channels)} ETAS-Kanal/Kanaele gefunden."
            if self.etas_channels else
            "Keine ETAS-Kanaele gefunden. Geraet, Verbindung und BOA pruefen."
        )
        self.update_backend()

    def handle_notice(self, notice: Notice) -> bool:
        """Returns False when the window was destroyed."""
        if notice.text:
            self.status.set(notice.text)
        if notice.progress is not None:
            self.show_progress(notice.progress)
        if notice.event == "connected":
            self.connected = True
        elif notice.event == "log_started":
            self.logging = True
            self.log_pending = False
        elif notice.event == "log_stopped":
            self.logging = False
            self.log_pending = False
            self.path.set(str(self.default_path()))
        elif notice.event == "disconnected":
            if self.connection is not None:
                self.connection.join()
            was_logging = self.logging
            self.connection = None
            self.connected = self.logging = self.log_pending = self.disconnecting = False
            if was_logging:
                self.path.set(str(self.default_path()))
            self.update_controls()
            if notice.error:
                self.closing = False
                messagebox.showerror(
                    "Verbindung getrennt / Log unvollstaendig",
                    notice.text
                    + "\n\nBei Verbindungsproblemen: passenden PEAK-/ETAS-Treiber, "
                    "PCAN-Basic bzw. ETAS BOA V2, "
                    "DLL-Architektur, FD-Hardware und Bit-Timing pruefen.",
                    parent=self.root,
                )
            if self.closing:
                self.root.destroy()
                return False
            return True
        self.update_controls()
        if notice.error and notice.event == "log_stopped":
            messagebox.showerror("CSV-Log", notice.text, parent=self.root)
        return True

    def poll(self) -> None:
        try:
            scan_result = self.scan_results.get_nowait()
        except queue.Empty:
            scan_result = None
        if scan_result is not None:
            self.handle_scan_result(scan_result)
        while True:
            try:
                notice = self.notices.get_nowait()
            except queue.Empty:
                break
            if not self.handle_notice(notice):
                return

        auto_scroll = self.tree.yview()[1] >= 0.99
        for _ in range(250):
            try:
                row = self.rows.get_nowait()
            except queue.Empty:
                break
            if self.show_live.get():
                self.show_row(row)
        children = self.tree.get_children()
        if len(children) > DISPLAY_ROWS:
            self.tree.delete(*children[:-DISPLAY_ROWS])
        if auto_scroll and children:
            self.tree.see(children[-1])
        self.root.after(100, self.poll)

    def close(self) -> None:
        if self.connection is None:
            self.root.destroy()
            return
        question = (
            "Log laeuft. Log stoppen, Interface trennen und beenden?"
            if self.logging else "Interface trennen und beenden?"
        )
        if messagebox.askyesno("Logger beenden", question, parent=self.root):
            self.closing = True
            self.disconnect()


def main() -> None:
    root = tk.Tk()
    LoggerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
