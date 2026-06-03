#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Monitor serial ligero para el firmware BalancinESP32_Pruebas.

Permite ver la telemetria y escribir comandos al ESP32 en la misma terminal.
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200


def available_ports() -> list[str]:
    return [port.device for port in list_ports.comports()]


def choose_port() -> str:
    ports = available_ports()
    if not ports:
        raise SystemExit("No encontre puertos seriales. Conecta el ESP32 y reintenta.")
    if len(ports) == 1:
        return ports[0]

    print("Puertos disponibles:")
    for index, port in enumerate(ports, start=1):
        print(f"  {index}. {port}")
    while True:
        choice = input("Selecciona puerto: ").strip()
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return ports[int(choice) - 1]
        if choice in ports:
            return choice
        print("Seleccion no valida.")


def reader_thread(ser: serial.Serial, stop: threading.Event, log_path: Path | None) -> None:
    log_file = None
    csv_writer = None
    try:
        if log_path is not None:
            log_file = log_path.open("w", newline="", encoding="utf-8")
            csv_writer = csv.writer(log_file)
            csv_writer.writerow(["host_time", "line"])

        while not stop.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                print(f"\n[serial cerrado] {exc}")
                stop.set()
                break
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            print(f"\r{text}\n> ", end="", flush=True)
            if csv_writer is not None:
                csv_writer.writerow([f"{time.time():.3f}", text])
                log_file.flush()
    finally:
        if log_file is not None:
            log_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor serial de pruebas del balancin")
    parser.add_argument("-p", "--port", help="Puerto serial, ejemplo: COM5")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--list", action="store_true", help="Lista puertos y sale")
    parser.add_argument("--log", type=Path, help="Guarda todas las lineas recibidas")
    args = parser.parse_args()

    if args.list:
        for port in available_ports():
            print(port)
        return 0

    port = args.port or choose_port()
    stop = threading.Event()

    with serial.Serial(port, args.baud, timeout=0.2, write_timeout=1.0) as ser:
        print(f"Conectado a {port} @ {args.baud}. Escribe 'help' o 'salir'.")
        thread = threading.Thread(
            target=reader_thread,
            args=(ser, stop, args.log),
            daemon=True,
        )
        thread.start()

        try:
            while not stop.is_set():
                try:
                    command = input("> ")
                except EOFError:
                    break
                if command.strip().lower() in {"salir", "exit", "quit"}:
                    break
                ser.write((command + "\n").encode("utf-8"))
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
