#!/usr/bin/env python3
# Board run (run it from your own terminal: an agent's non-interactive shell reads 0 bytes from the
# WSL UART, see CLAUDE.md). Loads main.bin like sw_coremark/upload_capture.py, then appends UART
# output to the log as it arrives until "RRTC DONE" / "FATAL" or the timeout, so a long run that
# is interrupted still leaves every finished PREP/RUN line on disk.
#   run_board.py <main.bin> [--log path] [--timeout sec]
import argparse
import os
import sys
import time

import serial
import serial.tools.list_ports

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "hw"))
import jtag_load  # noqa: E402


def find_port():
    for p in serial.tools.list_ports.comports():
        if "USB" in p.device or "Digilent" in p.description:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--log", default=None)
    ap.add_argument("--timeout", type=float, default=6 * 3600.0)
    a = ap.parse_args()
    log_path = a.log or a.bin + ".log"
    port = find_port()
    if not port:
        sys.exit(">> [ERROR] No suitable USB serial port found.")
    captured = ""
    t_start = None
    # Do not call reset_input_buffer()/reset_output_buffer(): they wedge RX on the usbip FTDI port.
    with serial.Serial(port, 115200, timeout=0.1, rtscts=False, dsrdtr=False, xonxoff=False) as ser, \
            open(log_path, "w") as log:
        jtag_load.prepare()
        print(f">> Listening on {port} for boot banner...")
        deadline = time.time() + 300.0
        while time.time() < deadline:
            if ser.in_waiting > 0:
                captured += ser.read(ser.in_waiting).decode("utf-8", "replace")
                if "Waiting for App Size" in captured:
                    jtag_load.send_image(ser, open(a.bin, "rb").read())
                    t_start = time.time()
                    break
            time.sleep(0.01)
        else:
            sys.exit(">> [ERROR] never saw 'Waiting for App Size'")
        # bytes before the image was sent belong to whatever ran before (e.g. an earlier run still printing)
        tail = ""
        n_runs = 0
        deadline = time.time() + a.timeout
        while time.time() < deadline:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode("utf-8", "replace")
                log.write(chunk)
                log.flush()
                tail = (tail + chunk)[-4096:]
                n_runs += chunk.count("RUN,")
                sys.stdout.write(f"\r>> {n_runs} RUN lines, {time.time() - t_start:7.0f} s ")
                sys.stdout.flush()
                if "RRTC DONE" in tail or "FATAL," in tail:
                    time.sleep(0.3)
                    log.write(ser.read(ser.in_waiting).decode("utf-8", "replace"))
                    break
            time.sleep(0.05)
        else:
            print("\n>> [WARN] timeout before RRTC DONE")
    print(f"\n>> log written to {log_path} (wall {time.time() - t_start:.0f} s)")


if __name__ == "__main__":
    main()
