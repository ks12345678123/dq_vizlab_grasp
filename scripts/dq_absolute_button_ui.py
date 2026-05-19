#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk


ENV_DIR = Path(__file__).resolve().parents[1]


class TrajectoryButtonUI:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.root = tk.Tk()
        self.root.title("dq_vizlab Track Control")
        self.root.geometry("520x340")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.output_queue: queue.Queue[tuple[str, str]] = queue.Queue()
        self.process: subprocess.Popen[str] | None = None
        self.worker: threading.Thread | None = None

        self.status_var = tk.StringVar(value="Idle")
        self.target_var = tk.StringVar(
            value=(
                f"Target xyz = [{self.args.target_x:.6f}, "
                f"{self.args.target_y:.6f}, {self.args.target_z:.6f}]"
            )
        )
        self.sequence_var = tk.StringVar(value=f"Sequence = {self.args.target_sequence}")

        self._build_widgets()
        self.root.after(100, self._drain_output_queue)

    def _build_widgets(self) -> None:
        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(frame, text="dq_vizlab Trajectory Trigger", font=("TkDefaultFont", 13, "bold")).pack(anchor=tk.W)
        ttk.Label(frame, textvariable=self.target_var).pack(anchor=tk.W, pady=(8, 0))
        ttk.Label(
            frame,
            text=(
                f"namespace={self.args.namespace}   "
                f"waypoints={self.args.waypoint_count}   samples={self.args.sample_count}"
            ),
        ).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(frame, textvariable=self.sequence_var, wraplength=470, justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))

        button_row = ttk.Frame(frame)
        button_row.pack(fill=tk.X, pady=(14, 10))

        self.start_button = ttk.Button(
            button_row,
            text="单点追踪",
            command=self._start_single_trajectory,
        )
        self.start_button.pack(side=tk.LEFT)

        self.sequence_button = ttk.Button(
            button_row,
            text="连续展示",
            command=self._start_sequence_trajectory,
        )
        self.sequence_button.pack(side=tk.LEFT, padx=(8, 0))

        self.status_label = ttk.Label(button_row, textvariable=self.status_var)
        self.status_label.pack(side=tk.LEFT, padx=(12, 0))

        self.log_text = tk.Text(frame, height=14, wrap="word")
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.configure(state=tk.DISABLED)

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _build_command(self) -> list[str]:
        return [
            sys.executable,
            str(ENV_DIR / "scripts" / "dq_absolute_plan_demo.py"),
            "--namespace",
            self.args.namespace,
            "--target-x",
            str(self.args.target_x),
            "--target-y",
            str(self.args.target_y),
            "--target-z",
            str(self.args.target_z),
            "--waypoint-count",
            str(self.args.waypoint_count),
            "--sample-count",
            str(self.args.sample_count),
            "--max-vel",
            str(self.args.max_vel),
            "--max-acc",
            str(self.args.max_acc),
        ]

    def _build_sequence_command(self) -> list[str]:
        return [
            sys.executable,
            str(ENV_DIR / "scripts" / "dq_absolute_sequence_demo.py"),
            "--namespace",
            self.args.namespace,
            "--target-sequence",
            self.args.target_sequence,
            "--waypoint-count",
            str(self.args.waypoint_count),
            "--sample-count",
            str(self.args.sample_count),
            "--max-vel",
            str(self.args.max_vel),
            "--max-acc",
            str(self.args.max_acc),
        ]

    def _start_process(self, command: list[str], label: str) -> None:
        if self.process is not None and self.process.poll() is None:
            return
        self.status_var.set("Running")
        self.start_button.state(["disabled"])
        self.sequence_button.state(["disabled"])
        self._append_log(f"\n[UI] {label}\n")
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
        )
        self.worker = threading.Thread(target=self._stream_process_output, daemon=True)
        self.worker.start()

    def _start_single_trajectory(self) -> None:
        self._start_process(self._build_command(), "start single trajectory request")

    def _start_sequence_trajectory(self) -> None:
        self._start_process(self._build_sequence_command(), "start sequence trajectory request")

    def _stream_process_output(self) -> None:
        process = self.process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            self.output_queue.put(("log", line))
        return_code = process.wait()
        self.output_queue.put(("done", f"[UI] process exited with code {return_code}\n"))

    def _drain_output_queue(self) -> None:
        while True:
            try:
                kind, payload = self.output_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._append_log(payload)
            elif kind == "done":
                self._append_log(payload)
                self.status_var.set("Idle")
                self.start_button.state(["!disabled"])
                self.sequence_button.state(["!disabled"])
                self.process = None
                self.worker = None
        self.root.after(100, self._drain_output_queue)

    def _on_close(self) -> None:
        process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Small popup UI that triggers dq_vizlab trajectory execution."
    )
    parser.add_argument("--namespace", default="qpin_sim")
    parser.add_argument("--target-x", type=float, default=0.6939863951)
    parser.add_argument("--target-y", type=float, default=-0.0565121498)
    parser.add_argument("--target-z", type=float, default=0.7430594672)
    parser.add_argument(
        "--target-sequence",
        default="0.56,-0.03,0.86;0.6939863951,-0.0565121498,0.7430594672;0.62,0.04,0.81",
    )
    parser.add_argument("--waypoint-count", type=int, default=12)
    parser.add_argument("--sample-count", type=int, default=160)
    parser.add_argument("--max-vel", type=float, default=0.20)
    parser.add_argument("--max-acc", type=float, default=0.35)
    return parser


def main() -> None:
    args = _build_arg_parser().parse_args()
    ui = TrajectoryButtonUI(args)
    ui.run()


if __name__ == "__main__":
    main()
