#!/usr/bin/env python3
"""Send repeated UDP velocity commands to rl_sar MuJoCo or Isaac play.

Packet format is plain text: "vx vy wz".
"""

from __future__ import annotations

import argparse
import configparser
import math
from pathlib import Path
import re
import socket
import sys
import threading
import time


_FLOAT_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?")
_DEFAULT_CONFIG_PATH = Path(__file__).with_name("udp_command_config.ini")


def _load_defaults(config_path: Path = _DEFAULT_CONFIG_PATH) -> dict[str, str]:
    defaults = {
        "host": "",
        "port": "5555",
        "rate": "30.0",
        "max_x": "0.5",
        "max_y": "0.5",
        "max_yaw": "0.5",
        "deadzone": "1200",
    }
    parser = configparser.ConfigParser()
    if config_path.exists():
        parser.read(config_path)
        if parser.has_section("udp"):
            defaults.update(parser["udp"])
        if parser.has_section("xbox"):
            defaults.update(parser["xbox"])
    return defaults


def _parse_command(text: str) -> list[float] | None:
    values = [float(match) for match in _FLOAT_RE.findall(text)]
    if len(values) < 3:
        return None
    return values[:3]


class XboxCommandSource:
    """Read Xbox controller axes and convert them to vx, vy, wz."""

    def __init__(self, max_x: float, max_y: float, max_yaw: float, deadzone: int):
        try:
            from inputs import get_gamepad
        except ImportError as exc:
            raise RuntimeError("Xbox mode requires the 'inputs' package. Install it on the sender PC.") from exc

        self._get_gamepad = get_gamepad
        self._max_joy_val = math.pow(2, 15)
        self._deadzone = int(deadzone)
        self._axes = {"left_y": 0.0, "left_x": 0.0, "right_x": 0.0}
        self._command = [0.0, 0.0, 0.0]
        self._limits = [float(max_x), float(max_y), float(max_yaw)]
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._monitor, daemon=True)
        self._thread.start()

    @staticmethod
    def _apply_deadzone(value: int, deadzone: int) -> int:
        if abs(value) < deadzone:
            return 0
        return value - deadzone if value > 0 else value + deadzone

    def _axis(self, state: int, sign: float) -> float:
        return sign * self._apply_deadzone(state, self._deadzone) / self._max_joy_val

    def _monitor(self):
        while not self._stop_event.is_set():
            try:
                events = self._get_gamepad()
            except Exception as exc:
                print(f"[warn] gamepad read failed: {exc}")
                time.sleep(0.2)
                continue
            with self._lock:
                for event in events:
                    if event.code == "ABS_Y":
                        self._axes["left_y"] = self._axis(event.state, -1.0)
                    elif event.code == "ABS_X":
                        self._axes["left_x"] = self._axis(event.state, 1.0)
                    elif event.code == "ABS_RX":
                        self._axes["right_x"] = self._axis(event.state, -1.0)
                    elif event.code == "BTN_START" and event.state == 1:
                        self._axes["left_y"] = 0.0
                        self._axes["left_x"] = 0.0
                        self._axes["right_x"] = 0.0
                    elif event.code == "BTN_SELECT" and event.state == 1:
                        self._stop_event.set()
                self._command[:] = [
                    self._axes["left_y"] * self._limits[0],
                    self._axes["left_x"] * self._limits[1],
                    self._axes["right_x"] * self._limits[2],
                ]

    def command(self) -> list[float]:
        with self._lock:
            return list(self._command)

    def stop_requested(self) -> bool:
        return self._stop_event.is_set()

    def stop(self):
        self._stop_event.set()


def main():
    defaults = _load_defaults()
    parser = argparse.ArgumentParser(description="Send repeated UDP velocity commands.")
    parser.add_argument(
        "host",
        nargs="?",
        default=defaults["host"],
        help="Receiver host. Defaults to scripts/udp_command_config.ini.",
    )
    parser.add_argument("--port", type=int, default=int(defaults["port"]), help="Receiver UDP port.")
    parser.add_argument("--rate", type=float, default=float(defaults["rate"]), help="Send rate in Hz.")
    parser.add_argument("--cmd", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("VX", "VY", "WZ"))
    parser.add_argument("--xbox", action="store_true", help="Read command from a locally connected Xbox controller.")
    parser.add_argument("--max-x", type=float, default=float(defaults["max_x"]), help="Xbox left-stick Y scale for vx.")
    parser.add_argument("--max-y", type=float, default=float(defaults["max_y"]), help="Xbox left-stick X scale for vy.")
    parser.add_argument("--max-yaw", type=float, default=float(defaults["max_yaw"]), help="Xbox right-stick X scale for wz.")
    parser.add_argument("--deadzone", type=int, default=int(defaults["deadzone"]), help="Raw joystick deadzone for Xbox mode.")
    args = parser.parse_args()

    if not args.host:
        parser.error("receiver host is required. Pass it as an argument or set udp.host in udp_command_config.ini.")

    command = [float(args.cmd[0]), float(args.cmd[1]), float(args.cmd[2])]
    stop_event = threading.Event()
    lock = threading.Lock()

    def _read_stdin():
        print("Type 'vx vy wz' then Enter to update, 'stop' to zero, or 'q' to quit.")
        for line in sys.stdin:
            text = line.strip()
            if text in {"q", "quit", "exit"}:
                stop_event.set()
                return
            with lock:
                if text in {"s", "stop", "zero"}:
                    command[:] = [0.0, 0.0, 0.0]
                    print("[cmd] 0.000 0.000 0.000")
                    continue
                parsed = _parse_command(text)
                if parsed is None:
                    print("[warn] expected: vx vy wz")
                    continue
                command[:] = parsed
                print(f"[cmd] {command[0]:.3f} {command[1]:.3f} {command[2]:.3f}")

    xbox = None
    if args.xbox:
        xbox = XboxCommandSource(args.max_x, args.max_y, args.max_yaw, args.deadzone)
        print("Xbox mode: left stick = vx/vy, right stick X = wz, START = zero, BACK = quit.")
    else:
        threading.Thread(target=_read_stdin, daemon=True).start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.host, int(args.port))
    period = 1.0 / max(float(args.rate), 1.0e-6)
    print(f"Sending UDP commands to {target[0]}:{target[1]} at {args.rate:g} Hz")
    try:
        while not stop_event.is_set():
            if xbox is not None:
                command = xbox.command()
                if xbox.stop_requested():
                    break
            else:
                with lock:
                    command = list(command)
            payload = f"{command[0]:.6f} {command[1]:.6f} {command[2]:.6f}\n".encode("utf-8")
            sock.sendto(payload, target)
            time.sleep(period)
    finally:
        if xbox is not None:
            xbox.stop()
        sock.sendto(b"0.0 0.0 0.0\n", target)
        sock.close()


if __name__ == "__main__":
    main()
