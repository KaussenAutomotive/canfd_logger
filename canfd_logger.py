"""CAN/CAN-FD-Logger fuer PEAK PCAN-USB (FD), Python 3.10+.

Installation:
    py -m pip install "python-can>=4.6,<5"
    PEAK PCAN-Basic und den passenden PEAK-USB-Treiber installieren.
    PCANBasic.dll muss zur Python-Architektur (32/64 Bit) passen.
Start:
    py canfd_logger.py

CAN FD erfordert einen PCAN-USB FD oder einen anderen FD-faehigen PCAN-Kanal.
Listen-only ist voreingestellt: keine Sendeframes, keine ACKs. Auf einem Bus
mit nur einem Sender wird daher ein weiterer aktiver Teilnehmer benoetigt.
CAN-FD-Timing: 80 MHz, Sample Point nominal 80 %, Datenphase 75 %.
Bei abweichendem Netzwerktiming muessen diese Werte angepasst werden.

CSV: Semikolon, UTF-8 mit BOM; jede empfangene Nachricht wird gespeichert.
adapter_timestamp_s ist der unveraenderte python-can/Adapter-Zeitstempel,
nicht zwingend Unix-Zeit (abhaengig vom Backend und optionalem uptime-Paket).
relative_s bezieht sich auf die erste Nachricht. dlc_bytes bezeichnet die
dekodierte Laenge, nicht den 4-Bit-DLC-Code; bei RTR die angeforderte Laenge.
Die Live-Anzeige ist begrenzt und darf Frames auslassen, nicht die CSV.
Bei erkannten Bus-/Treiber-/Schreibfehlern wird mit Fehlermeldung gestoppt;
eine verlustfreie Aufnahme bei beliebiger Buslast wird nicht garantiert.
CSV wird alle 0,5 Sekunden und beim Stoppen geleert/geschlossen.
Bestehende Dateien werden niemals ueberschrieben.
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
CSV_COLUMNS = (
    "adapter_timestamp_s", "relative_s", "channel", "id_hex",
    "extended", "fd", "brs", "esi", "rtr", "error", "direction",
    "dlc_bytes", "data_hex",
)


@dataclass(frozen=True)
class Settings:
    channel: str
    fd: bool
    nominal_bitrate: int
    data_bitrate: int
    listen_only: bool
    path: Path


@dataclass(frozen=True)
class Progress:
    frames: int
    payload_bytes: int
    display_dropped: int
    elapsed: float


@dataclass(frozen=True)
class Notice:
    text: str = ""
    progress: Progress | None = None
    finished: bool = False
    error: bool = False


def open_bus(settings: Settings) -> can.BusABC:
    import can
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


class Recorder(threading.Thread):
    def __init__(
        self,
        settings: Settings,
        rows: queue.Queue[tuple[str, ...]],
        notices: queue.Queue[Notice],
    ) -> None:
        super().__init__(name="PCAN-Recorder", daemon=False)
        self.settings = settings
        self.rows = rows
        self.notices = notices
        self.stop_event = threading.Event()

    def run(self) -> None:
        started = time.monotonic()
        frames = payload_bytes = display_dropped = 0
        first_timestamp: float | None = None
        errors: list[str] = []
        bus = None
        output = None
        try:
            from can.interfaces.pcan import PcanBus

            bus = open_bus(self.settings)
            output = self.settings.path.open("x", newline="", encoding="utf-8-sig")
            writer = csv.writer(output, delimiter=";")
            writer.writerow(CSV_COLUMNS)
            output.flush()
            self.notices.put(Notice(text=f"Aufzeichnung: {self.settings.path}"))
            last_flush = last_report = last_status = time.monotonic()
            while not self.stop_event.is_set():
                message = bus.recv(timeout=0.05)
                if message is not None:
                    if first_timestamp is None:
                        first_timestamp = message.timestamp
                    row = csv_row(message, first_timestamp, self.settings.channel)
                    writer.writerow(row)
                    frames += 1
                    payload_bytes += len(message.data)
                    try:
                        self.rows.put_nowait(row)
                    except queue.Full:
                        # Only the live view is lossy; the CSV was already written.
                        display_dropped += 1
                now = time.monotonic()
                if now - last_flush >= 0.5:
                    output.flush()
                    last_flush = now
                if now - last_status >= 0.25:
                    if isinstance(bus, PcanBus):
                        status = bus.status()
                        if status:
                            raise RuntimeError(
                                f"PCAN-Status 0x{status:05X}: Bus-/Pufferfehler. "
                                "Aufzeichnung eventuell unvollstaendig. "
                                "Verkabelung, Terminierung, Bitraten und Buslast pruefen."
                            )
                    last_status = now
                if now - last_report >= 0.25:
                    self.notices.put(Notice(progress=Progress(
                        frames, payload_bytes, display_dropped, now - started,
                    )))
                    last_report = now
        except Exception as exc:
            # The worker boundary must report driver and filesystem failures to Tk.
            errors.append(f"{type(exc).__name__}: {exc}")
        finally:
            if output is not None:
                try:
                    output.close()
                except OSError as exc:
                    errors.append(f"CSV konnte nicht geschlossen werden: {exc}")
            if bus is not None:
                try:
                    bus.shutdown()
                except Exception as exc:
                    errors.append(f"PCAN konnte nicht freigegeben werden: {exc}")
            self.notices.put(Notice(
                text="\n".join(errors) if errors else "Gestoppt. CSV geschlossen.",
                progress=Progress(
                    frames, payload_bytes, display_dropped, time.monotonic() - started,
                ),
                finished=True,
                error=bool(errors),
            ))


class LoggerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.worker: Recorder | None = None
        self.closing = False
        self.finished_notice: Notice | None = None
        self.rows: queue.Queue[tuple[str, ...]] = queue.Queue(DISPLAY_QUEUE_SIZE)
        self.notices: queue.Queue[Notice] = queue.Queue()
        self.config_widgets: list[tuple[ttk.Widget, str]] = []
        self.channel = tk.StringVar(value="PCAN_USBBUS1")
        self.mode = tk.StringVar(value="CAN FD")
        self.nominal = tk.StringVar(value="500000")
        self.data_rate = tk.StringVar(value="2000000")
        self.listen_only = tk.BooleanVar(value=True)
        self.show_live = tk.BooleanVar(value=True)
        self.path = tk.StringVar(value=str(self.default_path()))
        self.status = tk.StringVar(value="Bereit. PEAK-Treiber und PCAN-Basic erforderlich.")
        self.statistics = tk.StringVar(value="Frames: 0 | Daten: 0 Bytes | Laufzeit: 0.0 s")
        self.root.title("PEAK CAN / CAN FD Logger")
        self.root.geometry("1180x690")
        self.root.minsize(850, 480)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.build_ui()
        self.root.after(100, self.poll)

    @staticmethod
    def default_path() -> Path:
        return Path.cwd() / f"canfd_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"

    def build_ui(self) -> None:
        panel = ttk.Frame(self.root, padding=10)
        panel.pack(fill="both", expand=True)
        config = ttk.LabelFrame(panel, text="Verbindung", padding=8)
        config.pack(fill="x")
        fields = (
            ("PEAK-Kanal", self.channel,
             tuple(f"PCAN_USBBUS{i}" for i in range(1, 17)), "normal"),
            ("Modus", self.mode, ("CAN FD", "Klassisches CAN"), "readonly"),
            ("Nominal [bit/s]", self.nominal,
             ("125000", "250000", "500000", "1000000"), "normal"),
            ("Datenphase [bit/s]", self.data_rate,
             ("500000", "1000000", "2000000", "4000000", "5000000", "8000000"),
             "normal"),
        )
        for column, (label, variable, values, state) in enumerate(fields):
            ttk.Label(config, text=label).grid(row=0, column=column, sticky="w", padx=4)
            box = ttk.Combobox(
                config, textvariable=variable, values=values, state=state, width=22,
            )
            box.grid(row=1, column=column, sticky="ew", padx=4, pady=4)
            config.columnconfigure(column, weight=1)
            self.config_widgets.append((box, state))
        passive = ttk.Checkbutton(
            config, text="Listen-only (keine ACKs)", variable=self.listen_only,
        )
        passive.grid(row=2, column=0, columnspan=2, sticky="w", padx=4)
        self.config_widgets.append((passive, "normal"))
        ttk.Label(
            config, text="CAN FD: 80 MHz | Sample Points: nominal 80 %, Daten 75 %",
        ).grid(row=2, column=2, columnspan=2, sticky="w", padx=4)
        file_panel = ttk.Frame(panel)
        file_panel.pack(fill="x", pady=10)
        ttk.Label(file_panel, text="CSV-Datei:").pack(side="left")
        path_entry = ttk.Entry(file_panel, textvariable=self.path)
        path_entry.pack(side="left", fill="x", expand=True, padx=8)
        browse = ttk.Button(file_panel, text="Auswaehlen ...", command=self.browse)
        browse.pack(side="left")
        self.config_widgets.extend(((path_entry, "normal"), (browse, "normal")))
        controls = ttk.Frame(panel)
        controls.pack(fill="x", pady=(0, 8))
        self.start_button = ttk.Button(controls, text="Start", command=self.start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(
            controls, text="Stopp", command=self.stop, state="disabled",
        )
        self.stop_button.pack(side="left", padx=6)
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
            ("time", "Relativ [s]", 110), ("channel", "Kanal", 130),
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

    def browse(self) -> None:
        selected = filedialog.asksaveasfilename(
            parent=self.root, title="Neue CSV-Datei waehlen (kein Ueberschreiben)",
            defaultextension=".csv", filetypes=[("CSV-Dateien", "*.csv")],
            initialfile=self.default_path().name,
        )
        if selected:
            self.path.set(selected)

    def get_settings(self) -> Settings:
        channel = self.channel.get().strip().upper()
        if channel not in {f"PCAN_USBBUS{i}" for i in range(1, 17)}:
            raise ValueError("PEAK-Kanal muss PCAN_USBBUS1 bis PCAN_USBBUS16 sein.")
        nominal = int(self.nominal.get())
        fd = self.mode.get() == "CAN FD"
        data_rate = int(self.data_rate.get()) if fd else nominal
        if not 1 <= nominal <= 1_000_000:
            raise ValueError("Nominal-Bitrate muss zwischen 1 und 1000000 bit/s liegen.")
        if fd and not nominal <= data_rate <= 8_000_000:
            raise ValueError("Datenbitrate muss >= nominal und <= 8000000 bit/s sein.")
        if not self.path.get().strip():
            raise ValueError("Bitte eine CSV-Datei angeben.")
        path = Path(self.path.get().strip()).expanduser().resolve()
        if path.exists():
            raise ValueError("Datei existiert bereits. Bitte eine neue CSV-Datei waehlen.")
        if not path.parent.is_dir():
            raise ValueError("Der Zielordner existiert nicht.")
        return Settings(channel, fd, nominal, data_rate, self.listen_only.get(), path)

    def start(self) -> None:
        if self.worker is not None:
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
        except (ValueError, OSError) as exc:
            messagebox.showerror("Ungueltige Einstellungen", str(exc), parent=self.root)
            return
        self.clear_view()
        self.discard_pending_rows()
        self.finished_notice = None
        self.statistics.set("Frames: 0 | Daten: 0 Bytes | Laufzeit: 0.0 s")
        self.status.set("Verbinde mit PEAK ...")
        for widget, _ in self.config_widgets:
            widget.configure(state="disabled")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.worker = Recorder(settings, self.rows, self.notices)
        self.worker.start()

    def stop(self) -> None:
        if self.worker is not None:
            self.worker.stop_event.set()
            self.stop_button.configure(state="disabled")
            self.status.set("Stoppe und schliesse CSV ...")

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

    def poll(self) -> None:
        while True:
            try:
                notice = self.notices.get_nowait()
            except queue.Empty:
                break
            if notice.text:
                self.status.set(notice.text)
            if notice.progress is not None:
                progress = notice.progress
                self.statistics.set(
                    f"Frames: {progress.frames:,} | Daten: {progress.payload_bytes:,} Bytes"
                    f" | Laufzeit: {progress.elapsed:.1f} s"
                    f" | Nur Live-Anzeige verworfen: {progress.display_dropped:,}"
                )
            if notice.finished:
                self.finished_notice = notice

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

        if (
            self.worker is not None
            and self.finished_notice is not None
            and not self.worker.is_alive()
        ):
            self.worker.join()
            self.worker = None
            for widget, state in self.config_widgets:
                widget.configure(state=state)
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            if self.finished_notice is not None and self.finished_notice.error:
                self.closing = False
                messagebox.showerror(
                    "Aufzeichnung fehlgeschlagen / unvollstaendig",
                    self.finished_notice.text
                    + "\n\nBei Verbindungsproblemen: PEAK-Treiber, PCAN-Basic, "
                    "DLL-Architektur, FD-Hardware und Bit-Timing pruefen.",
                    parent=self.root,
                )
            if self.closing:
                self.root.destroy()
                return
        self.root.after(100, self.poll)

    def close(self) -> None:
        if self.worker is not None:
            if messagebox.askyesno(
                "Logger beenden", "Aufzeichnung stoppen und CSV schliessen?",
                parent=self.root,
            ):
                self.closing = True
                self.stop()
        else:
            self.root.destroy()


def main() -> None:
    root = tk.Tk()
    LoggerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
