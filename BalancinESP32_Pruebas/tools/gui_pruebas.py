#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interfaz grafica serial para BalancinESP32_Pruebas.

Usa solo tkinter + pyserial. No requiere PyQt.
"""

from __future__ import annotations

import argparse
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from tkinter import BOTH, END, HORIZONTAL, LEFT, RIGHT, X, Y, Canvas, StringVar, Tk
from tkinter import filedialog, messagebox, ttk

import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200
CSV_KEYS_V1 = [
    "t_ms",
    "ir_l",
    "ir_r",
    "ir8_l",
    "ir8_r",
    "enc_l",
    "enc_r",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "acc_roll",
    "acc_pitch",
    "comp_roll",
    "comp_pitch",
    "pwm_l",
    "pwm_r",
]
CSV_KEYS_V2 = [
    "t_ms",
    "ir_l",
    "ir_r",
    "ir8_l",
    "ir8_r",
    "enc_l",
    "enc_r",
    "ax_raw",
    "ay_raw",
    "az_raw",
    "gx_raw",
    "gy_raw",
    "gz_raw",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "acc_roll",
    "acc_pitch",
    "comp_roll",
    "comp_pitch",
    "pwm_l",
    "pwm_r",
]
INTEGER_KEYS = {
    "t_ms",
    "ir_l",
    "ir_r",
    "ir8_l",
    "ir8_r",
    "enc_l",
    "enc_r",
    "ax_raw",
    "ay_raw",
    "az_raw",
    "gx_raw",
    "gy_raw",
    "gz_raw",
    "pwm_l",
    "pwm_r",
}


@dataclass
class SerialEvent:
    kind: str
    text: str


class MiniPlot(Canvas):
    def __init__(self, master, title: str, series: dict[str, str], ymin=None, ymax=None):
        super().__init__(master, height=145, background="#111827", highlightthickness=0)
        self.title = title
        self.series = series
        self.ymin = ymin
        self.ymax = ymax
        self.data = {name: deque(maxlen=240) for name in series}
        self.bind("<Configure>", lambda _event: self.redraw())

    def append(self, values: dict[str, float]) -> None:
        for name in self.series:
            if name in values:
                self.data[name].append(float(values[name]))
        self.redraw()

    def clear(self) -> None:
        for points in self.data.values():
            points.clear()
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        w = max(10, self.winfo_width())
        h = max(10, self.winfo_height())
        pad_l, pad_r, pad_t, pad_b = 42, 10, 22, 22
        x0, y0 = pad_l, pad_t
        x1, y1 = w - pad_r, h - pad_b

        self.create_rectangle(x0, y0, x1, y1, outline="#374151", fill="#0f172a")
        self.create_text(10, 10, text=self.title, anchor="nw", fill="#d1d5db", font=("Segoe UI", 9, "bold"))

        all_values = [value for points in self.data.values() for value in points]
        if not all_values:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="Sin datos", fill="#6b7280")
            return

        ymin = min(all_values) if self.ymin is None else self.ymin
        ymax = max(all_values) if self.ymax is None else self.ymax
        if abs(ymax - ymin) < 1e-6:
            ymin -= 1.0
            ymax += 1.0
        margin = (ymax - ymin) * 0.08
        ymin -= margin
        ymax += margin

        for frac in (0.25, 0.5, 0.75):
            y = y0 + frac * (y1 - y0)
            self.create_line(x0, y, x1, y, fill="#1f2937")

        self.create_text(5, y0, text=f"{ymax:.1f}", anchor="nw", fill="#9ca3af", font=("Consolas", 8))
        self.create_text(5, y1 - 12, text=f"{ymin:.1f}", anchor="nw", fill="#9ca3af", font=("Consolas", 8))

        for index, (name, color) in enumerate(self.series.items()):
            points = list(self.data[name])
            if len(points) < 2:
                continue
            step = (x1 - x0) / max(1, len(points) - 1)
            coords = []
            for i, value in enumerate(points):
                x = x0 + i * step
                y = y1 - ((value - ymin) / (ymax - ymin)) * (y1 - y0)
                coords.extend([x, y])
            self.create_line(*coords, fill=color, width=2)
            self.create_text(x0 + 8 + index * 95, y1 + 4, text=name, anchor="nw", fill=color, font=("Consolas", 8))


class BalancinGui(Tk):
    def __init__(self, initial_port: str | None):
        super().__init__()
        self.title("Balancin - Monitor de pruebas")
        self.geometry("1180x760")
        self.minsize(980, 650)

        self.initial_port = initial_port
        self.ser: serial.Serial | None = None
        self.reader: threading.Thread | None = None
        self.stop_reader = threading.Event()
        self.events: queue.Queue[SerialEvent] = queue.Queue()
        self.latest: dict[str, float] = {}
        self.log_file = None
        self.connected_at = 0.0

        self.value_vars: dict[str, StringVar] = {}
        self.port_var = StringVar()
        self.baud_var = StringVar(value=str(DEFAULT_BAUD))
        self.status_var = StringVar(value="Desconectado")
        self.command_var = StringVar()
        self.volts_var = StringVar(value="3.0")
        self.ms_var = StringVar(value="700")
        self.rate_var = StringVar(value="250")
        self.cal_ms_var = StringVar(value="3000")
        self.enc_ms_var = StringVar(value="1000")
        self.mpu_samples_var = StringVar(value="500")
        self.inv_motor_l = StringVar(value="1")
        self.inv_motor_r = StringVar(value="0")
        self.inv_enc_l = StringVar(value="0")
        self.inv_enc_r = StringVar(value="1")

        self._setup_style()
        self._build_ui()
        self.refresh_ports()
        if self.initial_port:
            self.port_var.set(self.initial_port)

        self.after(50, self._poll_events)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _setup_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        self.configure(bg="#0b1120")
        style.configure(".", font=("Segoe UI", 9))
        style.configure("TFrame", background="#0b1120")
        style.configure("Panel.TFrame", background="#111827", relief="flat")
        style.configure("TLabel", background="#0b1120", foreground="#d1d5db")
        style.configure("Panel.TLabel", background="#111827", foreground="#d1d5db")
        style.configure("Muted.TLabel", background="#111827", foreground="#9ca3af")
        style.configure("Value.TLabel", background="#111827", foreground="#67e8f9", font=("Consolas", 13, "bold"))
        style.configure("Angle.TLabel", background="#111827", foreground="#f472b6", font=("Consolas", 24, "bold"))
        style.configure("Title.TLabel", background="#0b1120", foreground="#e5e7eb", font=("Segoe UI", 13, "bold"))
        style.configure("TButton", padding=(8, 5))
        style.configure("Stop.TButton", foreground="#ffffff", background="#b91c1c")
        style.map("Stop.TButton", background=[("active", "#ef4444")])

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=BOTH, expand=True)

        self._build_topbar(root)

        body = ttk.PanedWindow(root, orient=HORIZONTAL)
        body.pack(fill=BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(body, style="Panel.TFrame", padding=10)
        middle = ttk.Frame(body, style="Panel.TFrame", padding=10)
        right = ttk.Frame(body, style="Panel.TFrame", padding=10)
        body.add(left, weight=1)
        body.add(middle, weight=2)
        body.add(right, weight=2)

        self._build_values(left)
        self._build_controls(middle)
        self._build_log_and_plots(right)

    def _build_topbar(self, root: ttk.Frame) -> None:
        top = ttk.Frame(root)
        top.pack(fill=X)

        ttk.Label(top, text="Balancin - pruebas", style="Title.TLabel").pack(side=LEFT, padx=(0, 18))
        ttk.Label(top, text="Puerto").pack(side=LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=12, state="readonly")
        self.port_combo.pack(side=LEFT, padx=(6, 6))
        ttk.Button(top, text="Actualizar", command=self.refresh_ports).pack(side=LEFT, padx=(0, 12))

        ttk.Label(top, text="Baud").pack(side=LEFT)
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side=LEFT, padx=(6, 12))
        self.connect_btn = ttk.Button(top, text="Conectar", command=self.toggle_connection)
        self.connect_btn.pack(side=LEFT)

        ttk.Button(top, text="Guardar log", command=self.choose_log).pack(side=LEFT, padx=(12, 0))
        ttk.Button(top, text="Limpiar", command=self.clear_log).pack(side=LEFT, padx=(6, 0))
        ttk.Label(top, textvariable=self.status_var).pack(side=RIGHT)

    def _build_values(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Valores en vivo", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w")

        angle_panel = ttk.Frame(parent, style="Panel.TFrame")
        angle_panel.pack(fill=X, pady=(8, 8))
        ttk.Label(angle_panel, text="Inclinacion MPU", style="Muted.TLabel").pack(anchor="w")
        self.angle_deg_var = StringVar(value="- deg")
        ttk.Label(angle_panel, textvariable=self.angle_deg_var, style="Angle.TLabel").pack(anchor="e")
        self.angle_accel_var = StringVar(value="Accel: - deg")
        ttk.Label(angle_panel, textvariable=self.angle_accel_var, style="Muted.TLabel").pack(anchor="e")

        scroll_container = ttk.Frame(parent, style="Panel.TFrame")
        scroll_container.pack(fill=BOTH, expand=True, pady=(8, 0))
        canvas = Canvas(scroll_container, background="#111827", highlightthickness=0)
        scrollbar = ttk.Scrollbar(scroll_container, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side=LEFT, fill=BOTH, expand=True)
        scrollbar.pack(side=RIGHT, fill=Y)

        table = ttk.Frame(canvas, style="Panel.TFrame")
        table_window = canvas.create_window((0, 0), window=table, anchor="nw")
        table.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(table_window, width=event.width))

        rows = [
            ("ir_l", "IR L raw 12 bit"),
            ("ir_r", "IR R raw 12 bit"),
            ("ir8_l", "IR L 8 bit"),
            ("ir8_r", "IR R 8 bit"),
            ("enc_l", "Encoder L delta"),
            ("enc_r", "Encoder R delta"),
            ("ax_raw", "MPU ax raw 16 bit"),
            ("ay_raw", "MPU ay raw 16 bit"),
            ("az_raw", "MPU az raw 16 bit"),
            ("gx_raw", "MPU gx raw 16 bit"),
            ("gy_raw", "MPU gy raw 16 bit"),
            ("gz_raw", "MPU gz raw 16 bit"),
            ("ax_g", "Accel X g"),
            ("ay_g", "Accel Y g"),
            ("az_g", "Accel Z g"),
            ("gx_dps", "Gyro X dps"),
            ("gy_dps", "Gyro Y dps"),
            ("gz_dps", "Gyro Z dps"),
            ("acc_roll", "Accel roll deg"),
            ("acc_pitch", "Accel pitch deg"),
            ("comp_roll", "Filtro roll deg"),
            ("comp_pitch", "Filtro pitch deg"),
            ("pwm_l", "PWM L"),
            ("pwm_r", "PWM R"),
        ]
        for row, (key, label) in enumerate(rows):
            ttk.Label(table, text=label, style="Muted.TLabel").grid(row=row, column=0, sticky="w", pady=2)
            var = StringVar(value="-")
            self.value_vars[key] = var
            ttk.Label(table, textvariable=var, style="Value.TLabel").grid(row=row, column=1, sticky="e", pady=2, padx=(16, 0))
        table.columnconfigure(0, weight=1)
        table.columnconfigure(1, weight=0)

    def _build_controls(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Comandos", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w")

        stream = ttk.LabelFrame(parent, text="Telemetria", padding=8)
        stream.pack(fill=X, pady=(8, 8))
        ttk.Button(stream, text="Stream ON", command=lambda: self.send("stream on")).pack(side=LEFT)
        ttk.Button(stream, text="Stream OFF", command=lambda: self.send("stream off")).pack(side=LEFT, padx=6)
        ttk.Label(stream, text="Rate ms").pack(side=LEFT, padx=(12, 4))
        ttk.Entry(stream, textvariable=self.rate_var, width=7).pack(side=LEFT)
        ttk.Button(stream, text="Aplicar", command=lambda: self.send(f"rate {self.rate_var.get()}")).pack(side=LEFT, padx=6)

        motor = ttk.LabelFrame(parent, text="Motores", padding=8)
        motor.pack(fill=X, pady=8)
        row = ttk.Frame(motor)
        row.pack(fill=X)
        ttk.Label(row, text="Voltaje").pack(side=LEFT)
        ttk.Entry(row, textvariable=self.volts_var, width=7).pack(side=LEFT, padx=(4, 12))
        ttk.Label(row, text="ms").pack(side=LEFT)
        ttk.Entry(row, textvariable=self.ms_var, width=7).pack(side=LEFT, padx=(4, 12))
        ttk.Button(row, text="STOP", style="Stop.TButton", command=lambda: self.send("stop")).pack(side=RIGHT)

        row = ttk.Frame(motor)
        row.pack(fill=X, pady=(8, 0))
        ttk.Button(row, text="Pulso L", command=lambda: self.pulse("l")).pack(side=LEFT)
        ttk.Button(row, text="Pulso R", command=lambda: self.pulse("r")).pack(side=LEFT, padx=6)
        ttk.Button(row, text="Pulso ambos", command=lambda: self.pulse("b")).pack(side=LEFT)

        row = ttk.Frame(motor)
        row.pack(fill=X, pady=(8, 0))
        ttk.Button(row, text="Test L + encoder", command=lambda: self.test_motor("l")).pack(side=LEFT)
        ttk.Button(row, text="Test R + encoder", command=lambda: self.test_motor("r")).pack(side=LEFT, padx=6)

        inv = ttk.LabelFrame(parent, text="Inversiones en RAM", padding=8)
        inv.pack(fill=X, pady=8)
        self._inv_check(inv, "Motor L", self.inv_motor_l, "inv motor l")
        self._inv_check(inv, "Motor R", self.inv_motor_r, "inv motor r")
        self._inv_check(inv, "Enc L", self.inv_enc_l, "inv enc l")
        self._inv_check(inv, "Enc R", self.inv_enc_r, "inv enc r")

        sensors = ttk.LabelFrame(parent, text="Sensores", padding=8)
        sensors.pack(fill=X, pady=8)
        ttk.Button(sensors, text="Leer IR", command=lambda: self.send("ir")).pack(side=LEFT)
        ttk.Label(sensors, text="Cal ms").pack(side=LEFT, padx=(10, 4))
        ttk.Entry(sensors, textvariable=self.cal_ms_var, width=7).pack(side=LEFT)
        ttk.Button(sensors, text="Blanco", command=lambda: self.send(f"ircal blanco {self.cal_ms_var.get()}")).pack(side=LEFT, padx=6)
        ttk.Button(sensors, text="Negro", command=lambda: self.send(f"ircal negro {self.cal_ms_var.get()}")).pack(side=LEFT)
        ttk.Button(sensors, text="Umbral", command=lambda: self.send("threshold")).pack(side=LEFT, padx=6)

        enc_mpu = ttk.LabelFrame(parent, text="Encoders y MPU", padding=8)
        enc_mpu.pack(fill=X, pady=8)
        ttk.Label(enc_mpu, text="Enc ms").pack(side=LEFT)
        ttk.Entry(enc_mpu, textvariable=self.enc_ms_var, width=7).pack(side=LEFT, padx=4)
        ttk.Button(enc_mpu, text="Leer enc", command=lambda: self.send(f"enc {self.enc_ms_var.get()}")).pack(side=LEFT, padx=6)
        ttk.Button(enc_mpu, text="MPU", command=lambda: self.send("mpu")).pack(side=LEFT)
        ttk.Label(enc_mpu, text="Zero muestras").pack(side=LEFT, padx=(10, 4))
        ttk.Entry(enc_mpu, textvariable=self.mpu_samples_var, width=7).pack(side=LEFT)
        ttk.Button(enc_mpu, text="MPU zero", command=lambda: self.send(f"mpu zero {self.mpu_samples_var.get()}")).pack(side=LEFT, padx=6)

        misc = ttk.LabelFrame(parent, text="Ayuda", padding=8)
        misc.pack(fill=X, pady=8)
        for text, cmd in (("Pins", "pins"), ("Status", "status"), ("Help", "help")):
            ttk.Button(misc, text=text, command=lambda c=cmd: self.send(c)).pack(side=LEFT, padx=(0, 6))

        raw = ttk.LabelFrame(parent, text="Comando manual", padding=8)
        raw.pack(fill=X, pady=8)
        entry = ttk.Entry(raw, textvariable=self.command_var)
        entry.pack(side=LEFT, fill=X, expand=True)
        entry.bind("<Return>", lambda _event: self.send_manual())
        ttk.Button(raw, text="Enviar", command=self.send_manual).pack(side=RIGHT, padx=(8, 0))

    def _build_log_and_plots(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Graficas y salida", style="Panel.TLabel", font=("Segoe UI", 11, "bold")).pack(anchor="w")

        self.ir_plot = MiniPlot(parent, "Sensores IR raw", {"ir_l": "#2dd4bf", "ir_r": "#fbbf24"}, ymin=0, ymax=4095)
        self.ir_plot.pack(fill=X, pady=(8, 8))
        self.angle_plot = MiniPlot(parent, "Angulo de inclinacion MPU (grados)", {"acc_pitch": "#60a5fa", "comp_pitch": "#f472b6"})
        self.angle_plot.pack(fill=X, pady=(0, 8))
        self.pwm_plot = MiniPlot(parent, "PWM motores", {"pwm_l": "#a78bfa", "pwm_r": "#fb923c"}, ymin=-255, ymax=255)
        self.pwm_plot.pack(fill=X, pady=(0, 8))

        log_frame = ttk.Frame(parent, style="Panel.TFrame")
        log_frame.pack(fill=BOTH, expand=True)
        self.log = ttk.Treeview(log_frame, columns=("time", "line"), show="headings", height=10)
        self.log.heading("time", text="Tiempo")
        self.log.heading("line", text="Linea serial")
        self.log.column("time", width=78, anchor="w")
        self.log.column("line", width=520, anchor="w")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        self.log.pack(side=LEFT, fill=BOTH, expand=True)
        scroll.pack(side=RIGHT, fill=Y)

    def _inv_check(self, parent: ttk.Frame, text: str, var: StringVar, base_cmd: str) -> None:
        def on_toggle() -> None:
            self.send(f"{base_cmd} {var.get()}")

        ttk.Checkbutton(parent, text=text, variable=var, onvalue="1", offvalue="0", command=on_toggle).pack(side=LEFT, padx=(0, 12))

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.ser:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Puerto", "Selecciona un puerto serial.")
            return
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=1.0)
        except Exception as exc:
            messagebox.showerror("Conexion serial", str(exc))
            self.ser = None
            return

        self.stop_reader.clear()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.connected_at = time.time()
        self.connect_btn.configure(text="Desconectar")
        self.status_var.set(f"Conectado a {port}")
        self.send("status")

    def disconnect(self) -> None:
        self.stop_reader.set()
        if self.reader:
            self.reader.join(timeout=1.0)
        self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.configure(text="Conectar")
        self.status_var.set("Desconectado")

    def _reader_loop(self) -> None:
        while not self.stop_reader.is_set() and self.ser:
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.events.put(SerialEvent("error", f"Serial cerrado: {exc}"))
                break
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                self.events.put(SerialEvent("line", text))

    def _poll_events(self) -> None:
        handled = 0
        while handled < 100:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            handled += 1
            if event.kind == "error":
                self._append_log(event.text)
                self.disconnect()
            else:
                self._handle_line(event.text)
        self.after(50, self._poll_events)

    def _handle_line(self, text: str) -> None:
        self._append_log(text)
        if text.startswith("CSV,"):
            self._parse_csv(text)
        elif text.startswith("STATUS,inv_motor"):
            self._parse_status_inv(text)

    def _parse_csv(self, text: str) -> None:
        parts = text.split(",")
        if len(parts) == len(CSV_KEYS_V2) + 1:
            keys = CSV_KEYS_V2
        elif len(parts) == len(CSV_KEYS_V1) + 1:
            keys = CSV_KEYS_V1
        else:
            return
        values: dict[str, float] = {}
        for key, raw in zip(keys, parts[1:]):
            try:
                if key in INTEGER_KEYS:
                    values[key] = int(float(raw))
                else:
                    values[key] = float(raw)
            except ValueError:
                return

        self.latest = values
        for key, value in values.items():
            if key in self.value_vars:
                if isinstance(value, int):
                    self.value_vars[key].set(str(value))
                else:
                    self.value_vars[key].set(f"{value:.3f}")

        if "comp_pitch" in values:
            self.angle_deg_var.set(f"{values['comp_pitch']:+.2f} deg")
        if "acc_pitch" in values:
            self.angle_accel_var.set(f"Accel: {values['acc_pitch']:+.2f} deg")

        self.ir_plot.append(values)
        self.angle_plot.append(values)
        self.pwm_plot.append(values)

    def _parse_status_inv(self, text: str) -> None:
        for item in text.split(",")[1:]:
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            if key == "inv_motor_l":
                self.inv_motor_l.set(value)
            elif key == "inv_motor_r":
                self.inv_motor_r.set(value)
            elif key == "inv_enc_l":
                self.inv_enc_l.set(value)
            elif key == "inv_enc_r":
                self.inv_enc_r.set(value)

    def _append_log(self, text: str) -> None:
        now = time.strftime("%H:%M:%S")
        self.log.insert("", END, values=(now, text))
        children = self.log.get_children()
        if len(children) > 700:
            self.log.delete(children[0])
        self.log.yview_moveto(1.0)
        if self.log_file:
            self.log_file.write(f"{time.time():.3f},{text}\n")
            self.log_file.flush()

    def send(self, command: str) -> None:
        if not self.ser or not self.ser.is_open:
            self._append_log(f"[local] No conectado, no se envio: {command}")
            return
        try:
            self.ser.write((command.strip() + "\n").encode("utf-8"))
            self._append_log(f"> {command.strip()}")
        except Exception as exc:
            self._append_log(f"[local] Error enviando comando: {exc}")

    def send_manual(self) -> None:
        command = self.command_var.get().strip()
        if command:
            self.send(command)
            self.command_var.set("")

    def pulse(self, side: str) -> None:
        self.send(f"pulse {side} {self.volts_var.get()} {self.ms_var.get()}")

    def test_motor(self, side: str) -> None:
        self.send(f"testmotor {side} {self.volts_var.get()} {self.ms_var.get()}")

    def choose_log(self) -> None:
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.status_var.set("Log cerrado")
            return
        path = filedialog.asksaveasfilename(
            title="Guardar log serial",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Texto", "*.txt"), ("Todos", "*.*")],
        )
        if not path:
            return
        self.log_file = Path(path).open("w", encoding="utf-8")
        self.log_file.write("host_time,line\n")
        self.status_var.set(f"Log: {Path(path).name}")

    def clear_log(self) -> None:
        for item in self.log.get_children():
            self.log.delete(item)
        self.ir_plot.clear()
        self.angle_plot.clear()
        self.pwm_plot.clear()

    def on_close(self) -> None:
        self.disconnect()
        if self.log_file:
            self.log_file.close()
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="GUI serial de pruebas del balancin")
    parser.add_argument("-p", "--port", help="Puerto serial, ejemplo: COM7")
    args = parser.parse_args()

    app = BalancinGui(args.port)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
