"""
AC → SVJ Converter — Standalone tkinter GUI
Two tabs: Batch Conversion and Tire Lab.
All heavy work runs in background threads; UI stays responsive.
"""
from __future__ import annotations

import io
import json
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback
from datetime import datetime
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

from PIL import Image, ImageTk

# ── Locate package root ───────────────────────────────────────────────────────
_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))

from converter import (
    CONV_VER, SVJ_VERSION, REPO_URL, CONV_REPO,
    CORE_INI_FILES, EXTRA_INI_FILES,
    build_svj, read_car_directory, _clean, _car_stem,
)
from ac_parsers import parse_ini
from car_archives import ArchiveCar, extract_car, scan_archive_folder
from car_audit import audit_folder, write_reports
from batch_convert import _already_converted, _write_car_log, convert_one
from tire_lab import (
    ACTyreParams, parse_tyre_section, run_bench, build_svj_pacejka_blocks,
)

try:
    from kn5_reader import kn5_all_lods_to_glbs
    _KN5 = True
except ImportError:
    _KN5 = False


# ── Tiny helpers ──────────────────────────────────────────────────────────────

def _pil(b: bytes) -> Image.Image:
    return Image.open(io.BytesIO(b))


def _ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def _ts_long() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def _open_in_explorer(path: str) -> None:
    p = Path(path)
    if not p.exists():
        return
    if sys.platform == "win32":
        if p.is_file():
            subprocess.Popen(["explorer", "/select,", str(p)])
        else:
            subprocess.Popen(["explorer", str(p)])
    elif sys.platform == "darwin":
        subprocess.Popen(["open", str(p)])
    else:
        subprocess.Popen(["xdg-open", str(p.parent if p.is_file() else p)])


def _tag_for(line: str) -> str:
    """Detect log-line severity from its leading symbol."""
    s = line.lstrip()
    if s.startswith(("✓", "✔")):          return "ok"
    if s.startswith(("✗", "ERROR")):       return "err"
    if s.startswith(("⚠", "WARNING")):     return "warn"
    if s.startswith(("⊘", "SKIP")):        return "skip"
    if s.startswith(("─", "═", "──")):     return "sep"
    if s.startswith("ℹ"):                  return "info"
    return "plain"


# ── ImgLabel ──────────────────────────────────────────────────────────────────

class ImgLabel(tk.Label):
    def __init__(self, parent, max_w: int = 480, max_h: int = 320, **kw):
        kw.setdefault("relief", "sunken")
        kw.setdefault("bd", 1)
        kw.setdefault("fg", "gray")
        kw.setdefault("text", "—")
        super().__init__(parent, **kw)
        self._mw, self._mh = max_w, max_h
        self._ph: Optional[ImageTk.PhotoImage] = None

    def show(self, img: Optional[Image.Image]) -> None:
        if img is None:
            self.config(image="", text="—")
            self._ph = None
            return
        c = img.copy()
        c.thumbnail((self._mw, self._mh), Image.LANCZOS)
        self._ph = ImageTk.PhotoImage(c)
        self.config(image=self._ph, text="")


# ── CarTree — checkbox treeview ───────────────────────────────────────────────

class CarTree(ttk.Treeview):
    _ON  = "☑"
    _OFF = "☐"

    def __init__(self, parent, **kw):
        kw.setdefault("columns", ("sel", "name"))
        kw.setdefault("show", "headings")
        kw.setdefault("selectmode", "none")
        super().__init__(parent, **kw)
        self.heading("sel",  text="✓")
        self.heading("name", text="Car folder")
        self.column("sel",  width=42, anchor="center", stretch=False)
        self.column("name", anchor="w")
        self.bind("<Button-1>", self._click)
        self._all: list[str] = []
        self._on:  dict[str, bool] = {}

    def _click(self, e: tk.Event) -> None:
        if self.identify_region(e.x, e.y) != "cell": return
        if self.identify_column(e.x) != "#1": return
        row = self.identify_row(e.y)
        if not row: return
        name  = self.set(row, "name")
        state = not self._on.get(name, True)
        self._on[name] = state
        self.set(row, "sel", self._ON if state else self._OFF)

    def load(self, names: list[str]) -> None:
        self._all = list(names)
        self._on  = {n: True for n in names}
        self._redraw("")

    def _redraw(self, filt: str) -> None:
        self.delete(*self.get_children())
        ft = filt.strip().lower()
        for n in self._all:
            if ft and ft not in n.lower(): continue
            self.insert("", "end",
                        values=(self._ON if self._on.get(n, True) else self._OFF, n))

    def apply_filter(self, filt: str) -> None:
        self._redraw(filt)

    def select_all(self) -> None:
        for n in self._all: self._on[n] = True
        for iid in self.get_children(): self.set(iid, "sel", self._ON)

    def deselect_all(self) -> None:
        for n in self._all: self._on[n] = False
        for iid in self.get_children(): self.set(iid, "sel", self._OFF)

    def selected(self) -> list[str]:
        return [n for n in self._all if self._on.get(n, True)]


# ── ScrollableFrame ───────────────────────────────────────────────────────────

class ScrollableFrame(ttk.Frame):
    def __init__(self, parent, **kw):
        super().__init__(parent, **kw)
        self._c = tk.Canvas(self, highlightthickness=0)
        sb = ttk.Scrollbar(self, orient="vertical", command=self._c.yview)
        self.inner = ttk.Frame(self._c)
        self.inner.bind("<Configure>",
                        lambda _: self._c.configure(
                            scrollregion=self._c.bbox("all")))
        self._c.create_window((0, 0), window=self.inner, anchor="nw")
        self._c.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._c.pack(side="left", fill="both", expand=True)
        self._c.bind("<Enter>", lambda _: self._c.bind_all(
            "<MouseWheel>",
            lambda e: self._c.yview_scroll(-1 * (e.delta // 120), "units")))
        self._c.bind("<Leave>", lambda _: self._c.unbind_all("<MouseWheel>"))


# ── LogPane — timestamped, colour-coded log widget ───────────────────────────

class LogPane(tk.Frame):
    """
    A Text widget with:
    - Per-line timestamps
    - Colour tags: ok / err / warn / skip / sep / info / plain
    - Toolbar: Clear and Save buttons
    """

    _TAGS = {
        "ok":    {"foreground": "#1a7a1a"},
        "err":   {"foreground": "#cc2222", "font": ("Consolas", 9, "bold")},
        "warn":  {"foreground": "#b36200"},
        "skip":  {"foreground": "#888888"},
        "sep":   {"foreground": "#555555"},
        "info":  {"foreground": "#1144aa"},
        "plain": {"foreground": "#111111"},
        "ts":    {"foreground": "#999999"},
        "car":   {"foreground": "#333333", "font": ("Consolas", 9, "bold")},
    }

    def __init__(self, parent, height: int = 20, **kw):
        super().__init__(parent, **kw)

        # Toolbar
        bar = ttk.Frame(self)
        bar.pack(fill="x")
        ttk.Label(bar, text="Conversion log",
                  font=("TkDefaultFont", 9, "bold")).pack(side="left", padx=4)
        ttk.Button(bar, text="Save log…", command=self._save).pack(
            side="right", padx=2, pady=1)
        ttk.Button(bar, text="Clear", command=self.clear).pack(
            side="right", padx=2, pady=1)

        # Text + scrollbar
        wrap = tk.Frame(self)
        wrap.pack(fill="both", expand=True)
        self._t = tk.Text(
            wrap, height=height, font=("Consolas", 9),
            state="disabled", wrap="word",
            bg="#f8f8f8", relief="sunken", bd=1,
        )
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self._t.yview)
        self._t.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._t.pack(side="left", fill="both", expand=True)

        for tag, cfg in self._TAGS.items():
            self._t.tag_configure(tag, **cfg)

    def append(self, text: str, tag: str = "plain") -> None:
        self._t.configure(state="normal")
        ts = f"[{_ts()}] "
        self._t.insert("end", ts, "ts")
        self._t.insert("end", text + "\n", tag)
        self._t.configure(state="disabled")
        self._t.see("end")

    def append_auto(self, text: str) -> None:
        self.append(text, _tag_for(text))

    def append_car_header(self, car_name: str, idx: int, total: int) -> None:
        sep = "─" * 60
        self.append(sep, "sep")
        self.append(f"  [{idx}/{total}]  {car_name}", "car")
        self.append(sep, "sep")

    def clear(self) -> None:
        self._t.configure(state="normal")
        self._t.delete("1.0", "end")
        self._t.configure(state="disabled")

    def get_text(self) -> str:
        return self._t.get("1.0", "end")

    def _save(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save log",
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("Text files", "*.txt"),
                       ("All files", "*")],
            initialfile=f"ac_svj_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
        )
        if path:
            Path(path).write_text(self.get_text(), encoding="utf-8")


# ── File output helpers ───────────────────────────────────────────────────────

def _write_master_log(out_root: Path, stamp: str,
                      root_folder: Path,
                      selected: list[str],
                      all_car_logs: dict[str, list[str]],
                      summary_line: str) -> Path:
    """Write a single master log combining all cars."""
    hdr = "═" * 70
    lines: list[str] = [
        hdr,
        "AC → SVJ  BATCH MASTER LOG",
        f"Converter  : v{CONV_VER}   SVJ spec : {SVJ_VERSION}",
        f"Run started: {_ts_long()}",
        f"Cars folder: {root_folder}",
        f"Output     : {out_root}",
        f"Cars queued: {len(selected)}",
        hdr,
        "",
    ]
    for car_name, log_lines in all_car_logs.items():
        lines.append("")
        lines.append("─" * 70)
        lines.append(f"  CAR: {car_name}")
        lines.append("─" * 70)
        lines.extend(log_lines)

    lines += [
        "",
        hdr,
        f"SUMMARY: {summary_line}",
        f"Run finished: {_ts_long()}",
        hdr,
    ]
    p = out_root / f"batch_run_{stamp}.log"
    p.write_text("\n".join(lines), encoding="utf-8")
    return p


def _write_failed_log(out_root: Path, stamp: str, root_folder: Path,
                      failed: dict[str, tuple[str, str]], summary_line: str) -> Optional[Path]:
    """Second log with the failed cars only (name, reason, traceback), plus a plain
    ``batch_failed_<stamp>.txt`` with one car name per line to select them again.
    Returns the log path, or None when nothing failed."""
    if not failed:
        return None
    hdr = "═" * 70
    lines: list[str] = [
        hdr, "AC → SVJ  BATCH — FAILED CARS ONLY",
        f"Converter  : v{CONV_VER}   SVJ spec : {SVJ_VERSION}",
        f"Cars folder: {root_folder}", f"Output     : {out_root}",
        f"Failed     : {len(failed)}   ({summary_line})", hdr, "",
        "Summary:",
    ]
    lines += [f"  {name}: {reason}" for name, (reason, _tb) in failed.items()]
    for name, (reason, tb) in failed.items():
        lines += ["", "─" * 70, f"  CAR: {name}", f"  {reason}", "─" * 70, tb.rstrip()]
    p = out_root / f"batch_failed_{stamp}.log"
    p.write_text("\n".join(lines), encoding="utf-8")
    (out_root / f"batch_failed_{stamp}.txt").write_text(
        "\n".join(failed) + "\n", encoding="utf-8")
    return p


# ── Main Application ──────────────────────────────────────────────────────────

class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(f"AC → SVJ Converter  v{CONV_VER}")
        self.geometry("1440x960")
        self.minsize(1100, 750)

        self._q: queue.Queue = queue.Queue()
        self._stop_event    = threading.Event()
        self._batch_out: Optional[str] = None

        # Header
        hdr = ttk.Frame(self)
        hdr.pack(fill="x", padx=8, pady=(6, 2))
        ttk.Label(
            hdr,
            text=f"AC → SVJ Converter  v{CONV_VER}   ·   SVJ {SVJ_VERSION}   ·   SAE J670 / SI",
            font=("TkDefaultFont", 11, "bold"),
        ).pack(side="left")

        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=6, pady=4)

        self._batch_tab = ttk.Frame(nb)
        nb.add(self._batch_tab, text="  1 · Batch Conversion  ")
        self._build_batch()

        self._lab_tab = ttk.Frame(nb)
        nb.add(self._lab_tab, text="  2 · Tire Lab  ")
        self._build_tire_lab()

        self.after(80, self._poll_queue)

    # ── Queue pump ────────────────────────────────────────────────────────────

    def _poll_queue(self) -> None:
        try:
            while True:
                self._dispatch(self._q.get_nowait())
        except queue.Empty:
            pass
        self.after(80, self._poll_queue)

    def _dispatch(self, msg: tuple) -> None:
        kind = msg[0]
        if kind == "b_log":
            self._log.append_auto(msg[1])
        elif kind == "b_car":
            self._log.append_car_header(msg[1], msg[2], msg[3])
        elif kind == "b_prog":
            self._b_prog["value"] = msg[1] * 100
            self._b_prog_lbl.config(text=msg[2])
        elif kind == "b_done":
            self._b_prog["value"] = 100
            self._b_prog_lbl.config(text=msg[1])   # "Done" or "Stopped"
            self._set_running(False)
            self._batch_out = msg[2]
            if msg[2]:
                self._b_open_btn.config(state="normal")
        elif kind == "b_err":
            self._b_prog_lbl.config(text="Error")
            self._set_running(False)
            self._log.append(f"FATAL: {msg[1]}", "err")
            messagebox.showerror("Batch error", msg[1])
        elif kind == "lab_result":
            self._apply_lab_result(*msg[1])
        elif kind == "lab_busy":
            self._lab_status.config(text=msg[1])

    # =========================================================================
    # Tab 1 – Batch Conversion
    # =========================================================================

    def _build_batch(self) -> None:
        f = self._batch_tab

        # ── Path inputs ───────────────────────────────────────────────────────
        paths = ttk.LabelFrame(f, text="Folders")
        paths.pack(fill="x", padx=8, pady=(8, 2))
        paths.columnconfigure(1, weight=1)

        ttk.Label(paths, text="AC cars folder:").grid(
            row=0, column=0, sticky="w", padx=(6, 4), pady=4)
        self._b_cars_var = tk.StringVar()
        ttk.Entry(paths, textvariable=self._b_cars_var).grid(
            row=0, column=1, sticky="ew", padx=4)
        ttk.Button(paths, text="Browse…",
                   command=self._browse_cars, width=9).grid(row=0, column=2, padx=2)
        ttk.Button(paths, text="Scan",
                   command=self._scan, width=9).grid(row=0, column=3, padx=(2, 6))

        ttk.Label(paths, text="Output folder:").grid(
            row=1, column=0, sticky="w", padx=(6, 4), pady=4)
        self._b_out_var = tk.StringVar()
        ttk.Entry(paths, textvariable=self._b_out_var).grid(
            row=1, column=1, sticky="ew", padx=4)
        ttk.Button(paths, text="Browse…",
                   command=self._browse_out, width=9).grid(row=1, column=2, padx=2)
        ttk.Label(paths, text="(blank = auto-named sibling of cars folder)",
                  foreground="gray").grid(row=1, column=3, padx=(4, 6), sticky="w")

        self._b_audit_kn5_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(paths, text="also check KN5 encryption (slow)",
                        variable=self._b_audit_kn5_var).grid(
            row=2, column=1, sticky="e", padx=4, pady=(0, 4))
        ttk.Button(paths, text="Audit…", command=self._audit, width=9).grid(
            row=2, column=2, padx=2, pady=(0, 4))
        ttk.Label(paths, text="(lists cars that need unpacking / repair)",
                  foreground="gray").grid(row=2, column=3, padx=(4, 6), sticky="w")

        self._b_status = ttk.Label(f, text="← Scan a cars folder to get started",
                                   foreground="gray")
        self._b_status.pack(anchor="w", padx=12, pady=(2, 0))

        # ── Filter + car list ─────────────────────────────────────────────────
        list_frame = ttk.Frame(f)
        list_frame.pack(fill="both", expand=True, padx=8, pady=4)

        ctrl = ttk.Frame(list_frame)
        ctrl.pack(fill="x", pady=(0, 4))
        ttk.Label(ctrl, text="Filter:").pack(side="left")
        self._b_filt_var = tk.StringVar()
        self._b_filt_var.trace_add("write", lambda *_: self._apply_filter())
        ttk.Entry(ctrl, textvariable=self._b_filt_var, width=36).pack(
            side="left", padx=(4, 12))
        ttk.Button(ctrl, text="☑ Select all",
                   command=lambda: self._car_tree.select_all()).pack(
            side="left", padx=2)
        ttk.Button(ctrl, text="☐ Deselect all",
                   command=lambda: self._car_tree.deselect_all()).pack(
            side="left", padx=2)

        tree_wrap = ttk.Frame(list_frame)
        tree_wrap.pack(fill="both", expand=True)
        self._car_tree = CarTree(tree_wrap)
        vsb = ttk.Scrollbar(tree_wrap, orient="vertical",
                             command=self._car_tree.yview)
        self._car_tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self._car_tree.pack(side="left", fill="both", expand=True)

        # ── Options + action buttons ──────────────────────────────────────────
        opts = ttk.Frame(f)
        opts.pack(fill="x", padx=8, pady=(0, 2))

        self._b_plots_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opts, text="Include tire comparison PNGs",
                        variable=self._b_plots_var).pack(side="left")
        self._b_glb_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts, text="Convert KN5 → GLB",
            variable=self._b_glb_var,
            state="normal" if _KN5 else "disabled",
        ).pack(side="left", padx=16)
        self._b_skins_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            opts, text="Embed skins (liveries)",
            variable=self._b_skins_var,
            state="normal" if _KN5 else "disabled",
        ).pack(side="left", padx=4)
        self._b_flat_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts, text="Meshes next to the JSON (no meshes/ folder)",
            variable=self._b_flat_var,
            state="normal" if _KN5 else "disabled",
        ).pack(side="left", padx=4)
        # second row so nothing is pushed out of a narrow window
        opts = ttk.Frame(f)
        opts.pack(fill="x", padx=8, pady=(0, 2))
        self._b_dmg_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts, text="Include crash/damage textures",
            variable=self._b_dmg_var,
            state="normal" if _KN5 else "disabled",
        ).pack(side="left", padx=4)
        self._b_lod_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts, text="Highest LOD only",
            variable=self._b_lod_var,
            state="normal" if _KN5 else "disabled",
        ).pack(side="left", padx=4)
        self._b_skip_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts, text="Skip cars already in the output folder",
            variable=self._b_skip_var,
        ).pack(side="left", padx=4)
        ttk.Label(opts, text="Cars at once:").pack(side="left", padx=(12, 2))
        self._b_workers_var = tk.IntVar(value=max(1, min(3, (os.cpu_count() or 2) // 2)))
        ttk.Spinbox(opts, from_=1, to=max(2, os.cpu_count() or 2), width=3,
                    textvariable=self._b_workers_var).pack(side="left")
        if not _KN5:
            ttk.Label(opts, text="(kn5_reader unavailable)",
                      foreground="gray").pack(side="left")

        # action buttons on their own row: always visible, whatever the width
        btns = ttk.Frame(f)
        btns.pack(fill="x", padx=8, pady=(2, 2))
        self._b_open_btn = ttk.Button(
            btns, text="Open output folder",
            command=lambda: _open_in_explorer(self._batch_out or "."),
            state="disabled")
        self._b_open_btn.pack(side="right", padx=4)

        self._b_stop_btn = ttk.Button(
            btns, text="■  Stop",
            command=self._stop_batch,
            state="disabled")
        self._b_stop_btn.pack(side="right", padx=4)

        self._b_run_btn = ttk.Button(
            btns, text="▶  Convert selected",
            command=self._run_batch)
        self._b_run_btn.pack(side="right", padx=4)

        # ── Progress bar ──────────────────────────────────────────────────────
        pb_row = ttk.Frame(f)
        pb_row.pack(fill="x", padx=8, pady=2)
        self._b_prog = ttk.Progressbar(pb_row, mode="determinate")
        self._b_prog.pack(side="left", fill="x", expand=True)
        self._b_prog_lbl = ttk.Label(pb_row, text="", width=16, anchor="w")
        self._b_prog_lbl.pack(side="left", padx=6)

        # ── Log pane ──────────────────────────────────────────────────────────
        self._log = LogPane(f, height=18)
        self._log.pack(fill="both", expand=True, padx=8, pady=(0, 6))

    # ── Batch callbacks ───────────────────────────────────────────────────────

    def _browse_cars(self) -> None:
        d = filedialog.askdirectory(title="Select AC cars folder")
        if d: self._b_cars_var.set(d)

    def _browse_out(self) -> None:
        d = filedialog.askdirectory(title="Select output folder")
        if d: self._b_out_var.set(d)

    def _scan(self) -> None:
        root = Path(self._b_cars_var.get().strip())
        if not root.is_dir():
            messagebox.showerror("Not found", f"Directory not found:\n{root}")
            return
        subdirs     = sorted(d for d in root.iterdir() if d.is_dir())
        convertible = [d.name for d in subdirs if (d / "data").is_dir()]
        encrypted   = [d.name for d in subdirs
                       if not (d / "data").is_dir() and (d / "data.acd").is_file()]
        # downloaded cars still in .zip / .rar / .7z files: "<archive>::<car folder>"
        problems: list[str] = []
        zipped = scan_archive_folder(root, problems.append)
        for msg in problems:
            self._log.append(f"⚠ {msg}", "warn")
        convertible += [c.label for c in zipped if c.has_data]
        encrypted   += [c.label for c in zipped if not c.has_data and c.has_acd]
        self._car_tree.load(convertible)
        self._b_filt_var.set("")
        out_dir = self._b_out_var.get().strip()
        done = 0
        if out_dir and Path(out_dir).is_dir():
            done = sum(1 for n in convertible
                       if _already_converted(Path(out_dir) / n.split("::", 1)[-1], False))
        parts: list[str] = []
        if convertible:
            n_zip = sum(1 for c in zipped if c.has_data)
            parts.append(f"✓ {len(convertible)} convertible car(s)"
                         + (f" ({n_zip} in archives)" if n_zip else ""))
        if done:
            parts.append(f"{done} already in the output folder")
        if encrypted:
            parts.append(
                f"⚠ {len(encrypted)} encrypted — Content Manager → Unpack Data first")
        if not parts:
            parts.append("No AC cars found (need a car folder or a .zip/.rar/.7z with a data/ subfolder)")
        self._b_status.config(text="   ·   ".join(parts),
                              foreground="black" if convertible else "red")
        self._log.append(
            f"Scanned {root.name}: {len(convertible)} convertible, "
            f"{len(encrypted)} encrypted", "info")

    def _audit(self) -> None:
        root = Path(self._b_cars_var.get().strip())
        if not root.is_dir():
            messagebox.showerror("Not found", f"Directory not found:\n{root}")
            return
        out = self._b_out_var.get().strip()
        out_dir = Path(out) if out else root.parent
        check_kn5 = self._b_audit_kn5_var.get()
        self._log.append(f"▶ Auditing {root}" + (" (including KN5 encryption)" if check_kn5 else ""),
                         "info")

        def worker() -> None:
            put = lambda *a: self._q.put(a)
            try:
                last = [0.0]

                def progress(i: int, n: int, name: str) -> None:
                    put("b_prog", i / max(n, 1), f"audit {i}/{n}")

                rows = audit_folder(root, check_kn5, progress)
                paths = write_reports(rows, out_dir)
                c = paths["counts"]
                put("b_log", f"✓ Audit: {c['cars']} car(s), {c['ready']} ready to convert, "
                             f"{c['need_unpack']} need unpacking"
                             + (f", {c['kn5_encrypted']} with an encrypted KN5 and no clean copy"
                                if check_kn5 else "")
                             + (f", {c['decrypt_failed']} with a KN5 the decrypt tool could not recover"
                                if c["decrypt_failed"] else "")
                             + (f", {c['unreadable']} unreadable archive(s)" if c["unreadable"] else ""))
                for key in ("csv", "need_unpack", "kn5_encrypted", "decrypt_failed", "unreadable"):
                    if key in paths:
                        put("b_log", f"✓ {paths[key].name}  →  {paths[key].parent}")
                put("b_prog", 1.0, "audit done")
            except Exception:                                  # noqa: BLE001
                put("b_log", "✗ Audit failed:\n" + traceback.format_exc())

        threading.Thread(target=worker, daemon=True).start()

    def _apply_filter(self) -> None:
        self._car_tree.apply_filter(self._b_filt_var.get())

    def _set_running(self, running: bool) -> None:
        self._b_run_btn.config(state="disabled" if running else "normal")
        self._b_stop_btn.config(state="normal" if running else "disabled")

    def _workers(self) -> int:
        try:
            return max(1, int(self._b_workers_var.get()))
        except (tk.TclError, ValueError):
            return 1

    def _stop_batch(self) -> None:
        self._stop_event.set()
        self._b_stop_btn.config(state="disabled")
        self._log.append("⏹ Stop requested — will halt after the current car finishes.", "warn")

    def _run_batch(self) -> None:
        cars_folder = self._b_cars_var.get().strip()
        if not cars_folder or not Path(cars_folder).is_dir():
            messagebox.showerror("Error", "Select a valid cars folder first.")
            return
        selected = self._car_tree.selected()
        if not selected:
            messagebox.showinfo("Nothing selected", "Tick at least one car.")
            return

        self._b_prog["value"] = 0
        self._b_prog_lbl.config(text="Starting…")
        self._b_open_btn.config(state="disabled")
        self._batch_out = None
        self._stop_event.clear()
        self._set_running(True)
        self._log.append(
            f"▶ Starting batch: {len(selected)} car(s) selected from {cars_folder}",
            "info")

        threading.Thread(
            target=self._batch_worker,
            args=(
                cars_folder,
                self._b_out_var.get().strip(),
                self._b_plots_var.get(),
                self._b_glb_var.get(),
                self._b_skins_var.get(),
                selected,
                self._b_flat_var.get(),
                self._b_dmg_var.get(),
                self._b_lod_var.get(),
                self._b_skip_var.get(),
                self._workers(),
            ),
            daemon=True,
        ).start()

    # ── Batch worker (background thread) ─────────────────────────────────────

    def _batch_worker(self, cars_folder: str, out_folder: str,
                      include_plots: bool, convert_glb: bool,
                      include_skins: bool, selected: list[str],
                      flat_meshes: bool = False,
                      include_damage: bool = False,
                      highest_lod_only: bool = False,
                      skip_existing: bool = False,
                      workers: int = 1) -> None:
        # flat_meshes: put the GLBs beside the JSON instead of in meshes/
        mesh_sub = "" if flat_meshes else "meshes"
        q   = self._q
        put = lambda *a: q.put(a)

        try:
            # ── GLB dependency check ──────────────────────────────────────────
            if convert_glb and not _KN5:
                put("b_err", "kn5_reader not available — GLB export disabled.")
                return
            if convert_glb:
                try:
                    import pygltflib  # noqa: F401
                except ImportError:
                    put("b_err",
                        "pygltflib is not installed.\nRun:  pip install pygltflib")
                    return

            # ── Output folder ─────────────────────────────────────────────────
            root  = Path(cars_folder)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name  = f"ac_svj_batch_{stamp}_conv{CONV_VER}_svj{SVJ_VERSION}"
            out_root = Path(out_folder) if out_folder else root.parent / name
            out_root.mkdir(parents=True, exist_ok=True)

            zip_cars = {c.label: c for c in scan_archive_folder(root) if c.has_data}
            car_dirs: list = []
            for n in selected:
                if n in zip_cars:
                    car_dirs.append(zip_cars[n])
                elif (root / n / "data").is_dir():
                    car_dirs.append(root / n)
            # output folder per car; a name used twice gets its zip name in front
            out_names: list[str] = []
            for src in car_dirs:
                nm = src.name
                if nm in out_names and isinstance(src, ArchiveCar):
                    nm = f"{src.zip_path.stem}_{nm}"
                out_names.append(nm)
            if not car_dirs:
                put("b_log", "No convertible cars found in selection.")
                put("b_done", "Done", None)
                return

            put("b_log", f"Output → {out_root}")
            put("b_log", f"Queued : {len(car_dirs)} car(s)")
            put("b_log", "")

            csv_rows  = [
                "car_folder,svj_model,make,year,drive,compound,axle,"
                "lat_R2,lat_RMSE_N,long_R2,long_RMSE_N,Fz0_N,"
                "Fz0_front_N,DY0_front,DX0_front,status"
            ]
            all_car_logs: dict[str, list[str]] = {}
            ok = errors = skipped = 0
            failed: dict[str, tuple[str, str]] = {}
            stopped = False

            total = len(car_dirs)
            jobs: list[tuple[int, dict]] = []
            for i, src in enumerate(car_dirs):
                if skip_existing and _already_converted(out_root / out_names[i],
                                                        convert_glb and _KN5):
                    msg = f"⊘ Skipped — already in the output folder: {out_names[i]}"
                    put("b_prog", (i + 1) / total, f"{i+1}/{total}")
                    put("b_log", msg)
                    all_car_logs[out_names[i]] = [msg]
                    skipped += 1
                    continue
                jobs.append((i, {
                    "src": src, "car_out": out_root / out_names[i],
                    "include_plots": include_plots,
                    "convert_glb": bool(convert_glb and _KN5),
                    "include_skins": include_skins, "mesh_sub": mesh_sub,
                    "include_damage": include_damage,
                    "highest_lod_only": highest_lod_only,
                }))

            results: dict[int, dict] = {}
            workers = max(1, min(int(workers), len(jobs) or 1))
            if workers <= 1:
                # one car at a time, in this thread, log lines streamed as they come
                for i, job in jobs:
                    if self._stop_event.is_set():
                        stopped = True
                        put("b_log", "⏹ Batch stopped by user.")
                        break
                    name = (job["src"].name if isinstance(job["src"], ArchiveCar)
                            else Path(job["src"]).name)
                    put("b_prog", (i + 1) / total, f"{i+1}/{total}")
                    put("b_car", name, i + 1, total)
                    results[i] = convert_one(job, emit=lambda ln: put("b_log", ln))
            else:
                # several cars at once in worker processes; a car's lines are shown when
                # it is finished
                from concurrent.futures import CancelledError, ProcessPoolExecutor, as_completed
                put("b_log", f"ℹ Converting {len(jobs)} car(s) with {workers} worker processes")
                finished = skipped
                with ProcessPoolExecutor(max_workers=workers) as pool:
                    futs = {pool.submit(convert_one, job): (i, job) for i, job in jobs}
                    for fut in as_completed(futs):
                        if self._stop_event.is_set() and not stopped:
                            stopped = True
                            put("b_log", "⏹ Batch stopped by user — waiting for the cars "
                                         "in progress.")
                            for f in futs:
                                f.cancel()
                        i, job = futs[fut]
                        try:
                            r = fut.result()
                        except CancelledError:
                            continue
                        except Exception as ex_:                  # noqa: BLE001
                            nm = Path(job["src"].name if isinstance(job["src"], ArchiveCar)
                                      else job["src"]).name
                            r = {"status": "error", "name": nm, "label": nm, "stem": "",
                                 "ui": [f"✗ Error: {ex_}"], "car_log": [f"ERROR: {ex_}"],
                                 "csv": [f"{nm},,,,,,,,,,,,,,ERROR"],
                                 "error": (f"{type(ex_).__name__}: {ex_}",
                                           traceback.format_exc())}
                        results[i] = r
                        finished += 1
                        put("b_prog", finished / total, f"{finished}/{total}")
                        put("b_car", r["name"], finished, total)
                        for ln in r["ui"]:
                            put("b_log", ln)

            for i in sorted(results):                  # tallies in car order
                r = results[i]
                all_car_logs[r["name"]] = r["car_log"]
                csv_rows.extend(r["csv"])
                if r["status"] == "ok":
                    ok += 1
                elif r["status"] == "skipped":
                    skipped += 1
                else:
                    errors += 1
                    failed[r["label"]] = r["error"] or ("error", "")

            # ── Write batch summary CSV ───────────────────────────────────────
            csv_path = out_root / "batch_summary.csv"
            csv_path.write_text("\n".join(csv_rows), encoding="utf-8")

            # ── Write master log ──────────────────────────────────────────────
            summary_line = (f"{ok} converted,  {errors} errored,  "
                            f"{skipped} skipped"
                            + ("  — STOPPED" if stopped else ""))
            master_log = _write_master_log(
                out_root, stamp, root, selected, all_car_logs, summary_line)

            put("b_log", "")
            put("b_log", f"{'─'*60}")
            put("b_log", f"✓ {summary_line}")
            put("b_log", f"✓ Output folder : {out_root}")
            put("b_log", f"✓ Master log    : {master_log.name}")
            failed_log = _write_failed_log(out_root, stamp, root, failed, summary_line)
            if failed_log is not None:
                put("b_log", f"⚠ Failed cars   : {failed_log.name}  ({len(failed)} car(s); "
                             f"names in {failed_log.with_suffix('.txt').name})")
            put("b_log", f"✓ Batch summary : batch_summary.csv")

            lbl = "Stopped" if stopped else "Done"
            put("b_done", lbl, str(out_root))

        except Exception:
            put("b_err", traceback.format_exc())

    # =========================================================================
    # Tab 2 – Tire Lab
    # =========================================================================

    def _build_tire_lab(self) -> None:
        pw = ttk.PanedWindow(self._lab_tab, orient="horizontal")
        pw.pack(fill="both", expand=True)

        # ── Left pane — scrollable inputs (~370 px) ───────────────────────────
        left_outer = tk.Frame(
            pw, width=380,
            bg=ttk.Style().lookup("TFrame", "background"))
        left_outer.pack_propagate(False)
        pw.add(left_outer, weight=0)

        sf = ScrollableFrame(left_outer)
        sf.pack(fill="both", expand=True)
        inn = sf.inner

        # A — From car folder
        secA = ttk.LabelFrame(inn, text="A · From car folder")
        secA.pack(fill="x", padx=6, pady=(6, 2))
        self._l_carf_var = tk.StringVar()
        ttk.Entry(secA, textvariable=self._l_carf_var).pack(
            fill="x", padx=6, pady=(4, 2))
        row_a = ttk.Frame(secA)
        row_a.pack(fill="x", padx=6, pady=(0, 4))
        ttk.Button(row_a, text="Browse…",
                   command=self._lab_browse_car).pack(side="left")
        ttk.Button(row_a, text="Fit from folder",
                   command=self._lab_fit_folder).pack(side="right")

        ttk.Separator(inn, orient="horizontal").pack(fill="x", padx=6, pady=4)

        # B — From tyres.ini file
        secB = ttk.LabelFrame(inn, text="B · From tyres.ini file")
        secB.pack(fill="x", padx=6, pady=2)
        self._l_tyresf_var = tk.StringVar()
        ttk.Entry(secB, textvariable=self._l_tyresf_var).pack(
            fill="x", padx=6, pady=(4, 2))
        row_b = ttk.Frame(secB)
        row_b.pack(fill="x", padx=6, pady=(0, 4))
        ttk.Button(row_b, text="Browse…",
                   command=self._lab_browse_tyres).pack(side="left")
        ttk.Button(row_b, text="Fit from file",
                   command=self._lab_fit_file).pack(side="right")

        ttk.Separator(inn, orient="horizontal").pack(fill="x", padx=6, pady=4)

        # C — Manual parameters
        secC = ttk.LabelFrame(inn, text="C · Manual parameters")
        secC.pack(fill="x", padx=6, pady=2)
        self._lab_params: dict[str, tk.DoubleVar] = {}
        for lbl, key, default in [
            ("FZ0 (N)",            "fz0",   3500.0),
            ("DY0",                "dy0",    1.55),
            ("DY1",                "dy1",   -0.10),
            ("LS_EXPY",            "lsepy",  0.85),
            ("DX0",                "dx0",    1.60),
            ("DX1",                "dx1",   -0.08),
            ("LS_EXPX",            "lsepx",  0.90),
            ("K_a",                "ka",    22.0),
            ("K_k",                "kk",    18.0),
            ("FLEX",               "flex",   0.00018),
            ("CAMBER_GAIN",        "camb",   1.10),
            ("KINETIC_RATIO",      "kin",    0.92),
            ("Pressure now (psi)", "pnow",  27.0),
            ("Pressure ref (psi)", "pref",  27.0),
        ]:
            row = ttk.Frame(secC)
            row.pack(fill="x", padx=6, pady=1)
            ttk.Label(row, text=lbl, width=21, anchor="w").pack(side="left")
            v = tk.DoubleVar(value=default)
            self._lab_params[key] = v
            ttk.Entry(row, textvariable=v, width=12).pack(side="left")
        ttk.Button(secC, text="Run bench",
                   command=self._lab_fit_manual).pack(
            fill="x", padx=6, pady=(4, 6))

        ttk.Separator(inn, orient="horizontal").pack(fill="x", padx=6, pady=4)

        # Status + summary
        self._lab_status = ttk.Label(inn, text="", foreground="gray")
        self._lab_status.pack(anchor="w", padx=8)
        sum_f = ttk.LabelFrame(inn, text="Fit summary  (MF 5.2 + MF 6.2 JSON)")
        sum_f.pack(fill="x", padx=6, pady=(2, 8))
        self._l_sum = ScrolledText(
            sum_f, height=16, font=("Consolas", 8), state="normal", wrap="word")
        self._l_sum.pack(fill="x", padx=4, pady=4)

        # ── Right pane — images ───────────────────────────────────────────────
        right = ttk.Frame(pw)
        pw.add(right, weight=1)
        right_sf = ScrollableFrame(right)
        right_sf.pack(fill="both", expand=True)
        rinn = right_sf.inner

        ttk.Label(rinn, text="MF 5.2 — Pure Slip",
                  font=("TkDefaultFont", 10, "bold")).pack(
            anchor="w", padx=10, pady=(8, 2))
        row52 = ttk.Frame(rinn)
        row52.pack(fill="x", padx=6)
        self._img_lat  = ImgLabel(row52, max_w=500, max_h=330)
        self._img_long = ImgLabel(row52, max_w=500, max_h=330)
        self._img_lat .pack(side="left", fill="both", expand=True, padx=4, pady=4)
        self._img_long.pack(side="left", fill="both", expand=True, padx=4, pady=4)
        self._img_mu = ImgLabel(rinn, max_w=900, max_h=280)
        self._img_mu.pack(fill="x", padx=10, pady=(0, 6))

        ttk.Separator(rinn, orient="horizontal").pack(fill="x", padx=8, pady=6)

        ttk.Label(rinn, text="MF 6.2 — With Camber",
                  font=("TkDefaultFont", 10, "bold")).pack(
            anchor="w", padx=10, pady=(0, 2))
        row62 = ttk.Frame(rinn)
        row62.pack(fill="x", padx=6)
        self._img_mf62_lat = ImgLabel(row62, max_w=500, max_h=330)
        self._img_mf62_cam = ImgLabel(row62, max_w=500, max_h=330)
        self._img_mf62_lat.pack(side="left", fill="both", expand=True,
                                padx=4, pady=4)
        self._img_mf62_cam.pack(side="left", fill="both", expand=True,
                                padx=4, pady=4)

    # ── Tire Lab callbacks ────────────────────────────────────────────────────

    def _lab_browse_car(self) -> None:
        d = filedialog.askdirectory(title="Select AC car folder")
        if d: self._l_carf_var.set(d)

    def _lab_browse_tyres(self) -> None:
        f = filedialog.askopenfilename(
            title="Select tyres.ini",
            filetypes=[("INI files", "*.ini"), ("All files", "*")])
        if f: self._l_tyresf_var.set(f)

    def _lab_fit_folder(self) -> None:
        path = self._l_carf_var.get().strip()
        if not path:
            messagebox.showinfo("No path", "Enter an AC car folder path.")
            return
        self._lab_run(lambda: self._lab_from_folder(path))

    def _lab_fit_file(self) -> None:
        path = self._l_tyresf_var.get().strip()
        if not path or not Path(path).is_file():
            messagebox.showinfo("No file", "Select a tyres.ini file first.")
            return
        self._lab_run(lambda: self._lab_from_file(path))

    def _lab_fit_manual(self) -> None:
        self._lab_run(lambda: self._lab_from_manual())

    def _lab_run(self, fn) -> None:
        self._q.put(("lab_busy", "⏳ Computing…"))
        def worker() -> None:
            try:
                result = fn()
            except Exception:
                err = traceback.format_exc()
                result = (None, None, None, None, None, f"Error:\n{err}")
            self._q.put(("lab_result", result))
        threading.Thread(target=worker, daemon=True).start()

    # ── Tire Lab computation ──────────────────────────────────────────────────

    def _lab_from_folder(self, car_folder: str) -> tuple:
        car_path = Path(car_folder.strip())
        if not car_path.exists():
            return (None,)*5 + (f"Folder not found:\n{car_path}",)
        tyres_ini = car_path / "data" / "tyres.ini"
        if not tyres_ini.exists():
            return (None,)*5 + (
                f"data/tyres.ini not found in:\n{car_path}\n\n"
                "Unpack data.acd via AC Content Manager → Tools → Unpack Data."
            )
        text   = tyres_ini.read_text(encoding="utf-8", errors="replace")
        parsed = parse_ini(text)
        p = parse_tyre_section(parsed, section="FRONT", axle="front",
                               data_dir=car_path / "data")
        if p.source == "defaults":
            return (None,)*5 + ("No usable AC tyre parameters found in tyres.ini.",)
        return self._bench(p, header=f"Car: {car_path.name}")

    def _lab_from_file(self, path: str) -> tuple:
        text   = Path(path).read_text(encoding="utf-8", errors="replace")
        parsed = parse_ini(text)
        p = parse_tyre_section(parsed, section="FRONT", axle="front")
        if p.source == "defaults":
            return (None,)*5 + (
                "No AC tyre parameters found.\n"
                "Need a [FRONT] section with DY0, DX0, FZ0, LS_EXPY…")
        return self._bench(p, header=f"File: {Path(path).name}")

    def _lab_from_manual(self) -> tuple:
        def g(k: str) -> float:
            return float(self._lab_params[k].get())
        p = ACTyreParams(
            name="TireLab_manual", axle="front",
            FZ0=g("fz0"),  DY0=g("dy0"),   DY1=g("dy1"),   LS_EXPY=g("lsepy"),
            DX0=g("dx0"),  DX1=g("dx1"),   LS_EXPX=g("lsepx"),
            K_a=g("ka"),   K_k=g("kk"),    FLEX=g("flex"),
            CAMBER_GAIN=g("camb"), KINETIC_RATIO=g("kin"),
            PRESSURE_NOW_PSI=g("pnow"), PRESSURE_REF_PSI=g("pref"),
            source="manual",
        )
        return self._bench(p, header="Manual input")

    def _bench(self, p: ACTyreParams, header: str = "") -> tuple:
        br = run_bench(p)
        mf52, mf62 = build_svj_pacejka_blocks(br)
        sep = "─" * 60
        summary = "\n".join([
            header,
            sep,
            f"Params source    : {p.source}",
            f"FZ0              : {p.FZ0:.1f} N",
            f"Peak μ lateral   : {p.DY0:.4f}  (DY0)",
            f"Peak μ long.     : {p.DX0:.4f}  (DX0)",
            sep,
            f"Lateral   R²     : {br.fit_lateral['r2']:.4f}",
            f"Lateral   RMSE   : {br.fit_lateral['rmse_N']:.2f} N",
            f"Long.     R²     : {br.fit_longitudinal['r2']:.4f}",
            f"Long.     RMSE   : {br.fit_longitudinal['rmse_N']:.2f} N",
            sep,
            "── MF 5.2  (pacejka_mf52) ──",
            json.dumps(mf52, indent=2),
            "",
            "── MF 6.2  (pacejka) ──",
            json.dumps(mf62, indent=2),
        ])
        lat_img  = _pil(br.lateral_png)
        long_img = _pil(br.longitudinal_png)
        mu_img   = _pil(br.mu_vs_fz_png)
        cam_img  = _pil(br.mf62_camber_png)  if br.mf62_camber_png  else None
        lat6_img = _pil(br.mf62_lateral_png) if br.mf62_lateral_png else None
        return lat_img, long_img, mu_img, cam_img, lat6_img, summary

    def _apply_lab_result(
        self,
        lat:  Optional[Image.Image],
        long_: Optional[Image.Image],
        mu:   Optional[Image.Image],
        cam:  Optional[Image.Image],
        lat6: Optional[Image.Image],
        summary: str,
    ) -> None:
        self._img_lat.show(lat)
        self._img_long.show(long_)
        self._img_mu.show(mu)
        self._img_mf62_cam.show(cam)
        self._img_mf62_lat.show(lat6)
        self._l_sum.delete("1.0", "end")
        self._l_sum.insert("end", summary)
        ok = lat is not None
        self._lab_status.config(
            text="✓ Fit complete" if ok else "✗ Fit failed",
            foreground="#1a7a1a" if ok else "#cc2222",
        )


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = App()
    app.mainloop()
