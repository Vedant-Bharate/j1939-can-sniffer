#!/usr/bin/env python3
"""
J1939 CAN Bus Sniffer App
Connects to CAN network, shows named PGN traces, filters, live vehicle speed,
and supports opening / replaying Peak Systems .trc log files.
Uses SAE J1939 Digital Annex (JUL2026) data for names and key signal decoding.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import can
from can.io import TRCReader
import threading
import time
import json
from collections import deque, defaultdict
from datetime import datetime
import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Load J1939 PGN map (from SAE J1939DA JUL2026)
# ---------------------------------------------------------------------------
PGN_MAP: dict[int, str] = {}
try:
    map_path = Path(__file__).with_name("pgn_map.json")
    with open(map_path) as f:
        PGN_MAP = {int(k): v for k, v in json.load(f).items()}
except Exception as e:
    print("Warning: could not load pgn_map.json:", e)

# Key signal definitions (from J1939DA)
# Wheel-Based Vehicle Speed: SPN 84, PGN 65265, bytes 2-3, 0.00390625 km/h/bit
# Engine Speed: SPN 190, PGN 61444, bytes 4-5, 0.125 rpm/bit

def extract_pgn(can_id: int) -> int:
    """Extract 18-bit PGN from 29-bit J1939 CAN ID."""
    pf = (can_id >> 16) & 0xFF
    ps = (can_id >> 8) & 0xFF
    if pf < 240:  # PDU1
        return pf << 8
    return (pf << 8) | ps

def can_id_to_parts(can_id: int) -> dict:
    return {
        "prio": (can_id >> 26) & 0x7,
        "r": (can_id >> 25) & 0x1,
        "dp": (can_id >> 24) & 0x1,
        "pf": (can_id >> 16) & 0xFF,
        "ps": (can_id >> 8) & 0xFF,
        "sa": can_id & 0xFF,
        "pgn": extract_pgn(can_id),
    }

def decode_vehicle_speed(data: bytes) -> float | None:
    """Decode SPN 84 Wheel-Based Vehicle Speed from PGN 65265 data."""
    if len(data) < 3:
        return None
    raw = data[1] | (data[2] << 8)
    if raw == 0xFFFF:
        return None
    return raw * 0.00390625  # km/h

def decode_engine_speed(data: bytes) -> float | None:
    """Decode SPN 190 Engine Speed from PGN 61444 data."""
    if len(data) < 5:
        return None
    raw = data[3] | (data[4] << 8)
    if raw == 0xFFFF:
        return None
    return raw * 0.125  # rpm

def decode_actual_torque(data: bytes) -> float | None:
    """SPN 513 Actual Engine - Percent Torque, byte 3, offset -125."""
    if len(data) < 3:
        return None
    raw = data[2]
    if raw == 0xFF:
        return None
    return raw - 125.0

def format_data_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

# ---------------------------------------------------------------------------
# Simulator for demo (when no real CAN hardware)
# ---------------------------------------------------------------------------
class J1939Simulator:
    def __init__(self, bus: can.BusABC):
        self.bus = bus
        self.running = False
        self.thread: threading.Thread | None = None
        self.speed_kmh = 0.0
        self.rpm = 800.0

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _run(self):
        t0 = time.time()
        while self.running:
            t = time.time() - t0
            cycle = t % 40
            if cycle < 20:
                self.speed_kmh = min(90.0, cycle * 4.5)
            else:
                self.speed_kmh = max(0.0, 90 - (cycle - 20) * 4.5)
            self.rpm = 800 + self.speed_kmh * 18

            # PGN 65265 – Cruise Control/Vehicle Speed 1  (0x18FEF100)
            speed_raw = int(self.speed_kmh / 0.00390625) & 0xFFFF
            data_ccvs = bytearray(8)
            data_ccvs[1] = speed_raw & 0xFF
            data_ccvs[2] = (speed_raw >> 8) & 0xFF
            data_ccvs[6] = int(self.speed_kmh) & 0xFF
            try:
                self.bus.send(can.Message(arbitration_id=0x18FEF100, data=data_ccvs, is_extended_id=True))
            except Exception:
                pass

            # PGN 61444 – Electronic Engine Controller 1 (0x0CF00400)
            rpm_raw = int(self.rpm / 0.125) & 0xFFFF
            data_eec1 = bytearray(8)
            data_eec1[0] = 0xF0
            data_eec1[1] = 0x7D
            data_eec1[2] = int(max(0, min(250, (self.rpm - 800) / 10 + 125))) & 0xFF
            data_eec1[3] = rpm_raw & 0xFF
            data_eec1[4] = (rpm_raw >> 8) & 0xFF
            try:
                self.bus.send(can.Message(arbitration_id=0x0CF00400, data=data_eec1, is_extended_id=True))
            except Exception:
                pass

            # A few other typical messages
            for arb in (0x0CF00300, 0x18FEE000, 0x18FEF600):
                try:
                    self.bus.send(can.Message(arbitration_id=arb, data=bytes([0xFF] * 8), is_extended_id=True))
                except Exception:
                    pass

            time.sleep(0.05)


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
class J1939SnifferApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("J1939 CAN Sniffer – Live Vehicle Speed + .trc Replay")
        self.root.geometry("1280x820")
        self.root.minsize(960, 640)

        self.bus: can.BusABC | None = None
        self.notifier: can.Notifier | None = None
        self.sim: J1939Simulator | None = None
        self.running = False
        self.message_count = 0
        self.last_speed: float | None = None
        self.last_rpm: float | None = None
        self.last_torque: float | None = None

        # Message storage
        self.recent: deque = deque(maxlen=2000)
        self.pgn_counts: dict[int, int] = defaultdict(int)

        # .trc replay state
        self.trc_messages: list[can.Message] = []
        self.trc_replay_thread: threading.Thread | None = None
        self.trc_replaying = False
        self.trc_speed_factor = 1.0  # 1.0 = real-time

        self._build_ui()
        self._update_live_display()

    def _build_ui(self):
        # ---------- Top control bar ----------
        ctrl = ttk.Frame(self.root, padding=8)
        ctrl.pack(fill=tk.X)

        ttk.Label(ctrl, text="Interface:").pack(side=tk.LEFT)
        self.iface_var = tk.StringVar(value="virtual")
        ttk.Combobox(
            ctrl, textvariable=self.iface_var, width=12,
            values=["virtual", "socketcan", "pcan", "vector", "ixxat", "canalystii"]
        ).pack(side=tk.LEFT, padx=4)

        ttk.Label(ctrl, text="Channel:").pack(side=tk.LEFT, padx=(10, 0))
        self.channel_var = tk.StringVar(value="vcan0")
        ttk.Entry(ctrl, textvariable=self.channel_var, width=12).pack(side=tk.LEFT, padx=4)

        ttk.Label(ctrl, text="Bitrate:").pack(side=tk.LEFT, padx=(10, 0))
        self.bitrate_var = tk.StringVar(value="500000")
        ttk.Entry(ctrl, textvariable=self.bitrate_var, width=9).pack(side=tk.LEFT, padx=4)

        self.btn_start = ttk.Button(ctrl, text="▶ Start Live", command=self.start_sniff)
        self.btn_start.pack(side=tk.LEFT, padx=6)
        self.btn_stop = ttk.Button(ctrl, text="■ Stop", command=self.stop_sniff, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT)

        # .trc section
        ttk.Separator(ctrl, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Button(ctrl, text="📂 Open .trc", command=self.open_trc).pack(side=tk.LEFT, padx=2)
        self.btn_replay = ttk.Button(ctrl, text="▶ Replay .trc", command=self.start_trc_replay, state=tk.DISABLED)
        self.btn_replay.pack(side=tk.LEFT, padx=2)
        self.btn_stop_replay = ttk.Button(ctrl, text="■ Stop Replay", command=self.stop_trc_replay, state=tk.DISABLED)
        self.btn_stop_replay.pack(side=tk.LEFT, padx=2)

        ttk.Label(ctrl, text="Speed ×").pack(side=tk.LEFT, padx=(8, 0))
        self.replay_speed_var = tk.StringVar(value="1.0")
        ttk.Combobox(
            ctrl, textvariable=self.replay_speed_var, width=5,
            values=["0.25", "0.5", "1.0", "2.0", "5.0", "10.0", "50.0"]
        ).pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="Ready – Start Live or Open a .trc file")
        ttk.Label(ctrl, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)

        # ---------- Live vehicle data panel ----------
        live = ttk.LabelFrame(self.root, text="Live Vehicle Data", padding=10)
        live.pack(fill=tk.X, padx=8, pady=4)

        self.speed_label = ttk.Label(live, text="— km/h", font=("Helvetica", 36, "bold"), foreground="#0066cc")
        self.speed_label.pack(side=tk.LEFT, padx=16)
        ttk.Label(live, text="Vehicle Speed\n(SPN 84 / PGN 65265)", justify=tk.CENTER).pack(side=tk.LEFT)

        self.rpm_label = ttk.Label(live, text="— rpm", font=("Helvetica", 36, "bold"), foreground="#cc6600")
        self.rpm_label.pack(side=tk.LEFT, padx=(36, 16))
        ttk.Label(live, text="Engine Speed\n(SPN 190 / PGN 61444)", justify=tk.CENTER).pack(side=tk.LEFT)

        self.torque_label = ttk.Label(live, text="— %", font=("Helvetica", 28, "bold"), foreground="#228822")
        self.torque_label.pack(side=tk.LEFT, padx=(36, 16))
        ttk.Label(live, text="Actual Torque\n(SPN 513)", justify=tk.CENTER).pack(side=tk.LEFT)

        self.msg_count_label = ttk.Label(live, text="Msgs: 0")
        self.msg_count_label.pack(side=tk.RIGHT, padx=10)

        # ---------- Filter bar ----------
        filt = ttk.Frame(self.root, padding=(8, 4))
        filt.pack(fill=tk.X)
        ttk.Label(filt, text="Filter PGN / name:").pack(side=tk.LEFT)
        self.filter_var = tk.StringVar()
        self.filter_var.trace_add("write", lambda *_: self._refresh_table())
        ttk.Entry(filt, textvariable=self.filter_var, width=36).pack(side=tk.LEFT, padx=4)
        ttk.Button(filt, text="Clear", command=lambda: self.filter_var.set("")).pack(side=tk.LEFT)
        ttk.Button(filt, text="Only Speed (65265)", command=lambda: self.filter_var.set("65265")).pack(side=tk.LEFT, padx=4)
        ttk.Button(filt, text="Only EEC1 (61444)", command=lambda: self.filter_var.set("61444")).pack(side=tk.LEFT)
        ttk.Button(filt, text="Clear Table", command=self.clear_table).pack(side=tk.RIGHT, padx=4)

        # ---------- Message table ----------
        table_frame = ttk.Frame(self.root)
        table_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        columns = ("time", "can_id", "pgn", "name", "sa", "data", "decoded")
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=22)
        self.tree.heading("time", text="Time / Offset")
        self.tree.heading("can_id", text="CAN ID")
        self.tree.heading("pgn", text="PGN")
        self.tree.heading("name", text="PG Name")
        self.tree.heading("sa", text="SA")
        self.tree.heading("data", text="Data (hex)")
        self.tree.heading("decoded", text="Decoded (key signals)")

        self.tree.column("time", width=100, anchor=tk.W)
        self.tree.column("can_id", width=100, anchor=tk.CENTER)
        self.tree.column("pgn", width=70, anchor=tk.CENTER)
        self.tree.column("name", width=280, anchor=tk.W)
        self.tree.column("sa", width=45, anchor=tk.CENTER)
        self.tree.column("data", width=180, anchor=tk.W)
        self.tree.column("decoded", width=220, anchor=tk.W)

        vsb = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.tree.yview)
        hsb = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        # ---------- Bottom info ----------
        info = ttk.Label(
            self.root,
            text="Data: SAE J1939DA JUL2026  |  SPN 84 = Wheel-Based Vehicle Speed  |  SPN 190 = Engine Speed  |  Supports Peak .trc (v1.x / v2.x)",
            font=("Helvetica", 9), foreground="#555"
        )
        info.pack(side=tk.BOTTOM, pady=4)

    # ------------------------------------------------------------------
    # Live sniffing
    # ------------------------------------------------------------------
    def start_sniff(self):
        if self.running or self.trc_replaying:
            messagebox.showinfo("Busy", "Stop the current session first.")
            return
        iface = self.iface_var.get().strip()
        channel = self.channel_var.get().strip()
        try:
            bitrate = int(self.bitrate_var.get())
        except ValueError:
            messagebox.showerror("Error", "Invalid bitrate")
            return

        try:
            if iface == "virtual":
                self.bus = can.interface.Bus(
                    interface="virtual", channel=channel or "vcan0", receive_own_messages=True
                )
                self.sim = J1939Simulator(self.bus)
                self.sim.start()
                self.status_var.set("Running on VIRTUAL bus (simulated driving)")
            else:
                kwargs = {"interface": iface, "channel": channel, "bitrate": bitrate}
                if iface == "socketcan":
                    kwargs.pop("bitrate", None)
                self.bus = can.interface.Bus(**kwargs)
                self.status_var.set(f"Connected → {iface}:{channel}")
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            return

        self.running = True
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self.message_count = 0
        self.recent.clear()

        listener = can.Listener()
        listener.on_message_received = self._on_message
        self.notifier = can.Notifier(self.bus, [listener])

    def stop_sniff(self):
        self.running = False
        if self.sim:
            self.sim.stop()
            self.sim = None
        if self.notifier:
            self.notifier.stop()
            self.notifier = None
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_var.set("Stopped")

    def _on_message(self, msg: can.Message):
        if not msg.is_extended_id:
            return
        self._process_message(msg, time_str=datetime.now().strftime("%H:%M:%S.%f")[:-3])

    def _process_message(self, msg: can.Message, time_str: str | None = None):
        parts = can_id_to_parts(msg.arbitration_id)
        pgn = parts["pgn"]
        name = PGN_MAP.get(pgn, "Unknown / Proprietary")
        self.pgn_counts[pgn] += 1
        self.message_count += 1

        decoded = ""
        if pgn == 65265:
            spd = decode_vehicle_speed(msg.data)
            if spd is not None:
                self.last_speed = spd
                decoded = f"Speed: {spd:.1f} km/h"
        elif pgn == 61444:
            rpm = decode_engine_speed(msg.data)
            tq = decode_actual_torque(msg.data)
            parts_dec = []
            if rpm is not None:
                self.last_rpm = rpm
                parts_dec.append(f"RPM: {rpm:.0f}")
            if tq is not None:
                self.last_torque = tq
                parts_dec.append(f"Torque: {tq:.0f}%")
            decoded = "  ".join(parts_dec)

        if time_str is None:
            # Use relative timestamp from .trc if available
            ts = getattr(msg, "timestamp", None)
            time_str = f"{ts:.3f}" if ts is not None else datetime.now().strftime("%H:%M:%S.%f")[:-3]

        entry = {
            "time": time_str,
            "can_id": f"{msg.arbitration_id:08X}",
            "pgn": str(pgn),
            "name": name,
            "sa": str(parts["sa"]),
            "data": format_data_hex(msg.data),
            "decoded": decoded,
            "raw_pgn": pgn,
        }
        self.recent.appendleft(entry)
        self.root.after(0, self._refresh_table)

    # ------------------------------------------------------------------
    # .trc file support (Peak Systems format via python-can TRCReader)
    # ------------------------------------------------------------------
    def open_trc(self):
        if self.running or self.trc_replaying:
            messagebox.showinfo("Busy", "Stop the current session first.")
            return
        path = filedialog.askopenfilename(
            title="Open Peak .trc file",
            filetypes=[("Peak Trace files", "*.trc"), ("All files", "*.*")]
        )
        if not path:
            return
        try:
            messages = []
            with TRCReader(path) as reader:
                for msg in reader:
                    if msg is not None and msg.is_extended_id:
                        messages.append(msg)
            if not messages:
                messagebox.showwarning("Empty", "No extended (J1939) frames found in the file.")
                return
            self.trc_messages = messages
            self.btn_replay.config(state=tk.NORMAL)
            self.status_var.set(f"Loaded {len(messages)} frames from {Path(path).name}")
            # Immediately populate the table for review (static view)
            self.clear_table()
            for msg in messages:
                # Use the original relative timestamp from the log
                ts = getattr(msg, "timestamp", 0.0)
                self._process_message(msg, time_str=f"{ts:.3f}")
            messagebox.showinfo(
                "Loaded",
                f"Loaded {len(messages)} J1939 frames.\n\n"
                "You can now browse/filter them, or press “▶ Replay .trc” to play them back in real time "
                "(or accelerated)."
            )
        except Exception as e:
            messagebox.showerror("Failed to open .trc", str(e))

    def start_trc_replay(self):
        if not self.trc_messages:
            messagebox.showinfo("No file", "Open a .trc file first.")
            return
        if self.running or self.trc_replaying:
            return
        try:
            self.trc_speed_factor = float(self.replay_speed_var.get())
        except ValueError:
            self.trc_speed_factor = 1.0

        self.trc_replaying = True
        self.btn_replay.config(state=tk.DISABLED)
        self.btn_stop_replay.config(state=tk.NORMAL)
        self.btn_start.config(state=tk.DISABLED)
        self.clear_table()
        self.message_count = 0
        self.status_var.set(f"Replaying .trc at ×{self.trc_speed_factor} …")

        self.trc_replay_thread = threading.Thread(target=self._trc_replay_worker, daemon=True)
        self.trc_replay_thread.start()

    def stop_trc_replay(self):
        self.trc_replaying = False
        if self.trc_replay_thread:
            self.trc_replay_thread.join(timeout=1.5)
            self.trc_replay_thread = None
        self.btn_replay.config(state=tk.NORMAL if self.trc_messages else tk.DISABLED)
        self.btn_stop_replay.config(state=tk.DISABLED)
        self.btn_start.config(state=tk.NORMAL)
        self.status_var.set("Replay stopped")

    def _trc_replay_worker(self):
        if not self.trc_messages:
            return
        # Sort by timestamp just in case
        msgs = sorted(self.trc_messages, key=lambda m: getattr(m, "timestamp", 0.0))
        t0 = getattr(msgs[0], "timestamp", 0.0)
        wall_start = time.time()

        for msg in msgs:
            if not self.trc_replaying:
                break
            ts = getattr(msg, "timestamp", 0.0)
            target = wall_start + (ts - t0) / self.trc_speed_factor
            delay = target - time.time()
            if delay > 0.001:
                time.sleep(delay)
            self._process_message(msg, time_str=f"{ts:.3f}")

        self.root.after(0, self.stop_trc_replay)

    def clear_table(self):
        self.recent.clear()
        self.tree.delete(*self.tree.get_children())
        self.message_count = 0
        self.msg_count_label.config(text="Msgs: 0")

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------
    def _refresh_table(self):
        filt = self.filter_var.get().strip().lower()
        self.tree.delete(*self.tree.get_children())
        shown = 0
        for e in self.recent:
            if filt:
                if (filt not in e["pgn"] and filt not in e["name"].lower()
                        and filt not in e["can_id"].lower()):
                    continue
            self.tree.insert("", tk.END, values=(
                e["time"], e["can_id"], e["pgn"], e["name"], e["sa"], e["data"], e["decoded"]
            ))
            shown += 1
            if shown > 400:
                break
        self.msg_count_label.config(text=f"Msgs: {self.message_count}")

    def _update_live_display(self):
        if self.last_speed is not None:
            self.speed_label.config(text=f"{self.last_speed:.1f} km/h")
        if self.last_rpm is not None:
            self.rpm_label.config(text=f"{self.last_rpm:.0f} rpm")
        if self.last_torque is not None:
            self.torque_label.config(text=f"{self.last_torque:.0f} %")
        self.root.after(80, self._update_live_display)

    def on_close(self):
        self.stop_sniff()
        self.stop_trc_replay()
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except Exception:
        pass
    app = J1939SnifferApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
