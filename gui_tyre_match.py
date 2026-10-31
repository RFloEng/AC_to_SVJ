"""Tab 3 of the desktop app: find the tyres of the user's own AC library that behave like a
given tyre (a tyre set of an SVJ file, or a tyre of another car).  The logic is in
`tyre_match`; this is the window."""
from __future__ import annotations

import io
import json
import queue
import threading
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Optional

from PIL import Image, ImageTk

import ac_paths
import tyre_match as T

_COLUMNS = (
    ("score", "Match %", 62), ("beh", "Grip %", 56), ("size", "Size %", 56),
    ("car", "Car", 200), ("section", "Section", 70), ("name", "Compound", 150),
    ("size_code", "Size", 90), ("wr", "Width / radius", 100),
    ("muy", "μ lat", 52), ("mux", "μ long", 52), ("cy", "Cα /rad", 60),
)


class TyreMatchTab:
    def __init__(self, parent: ttk.Frame, app: tk.Misc, cars_var: tk.StringVar) -> None:
        self.app = app
        self.cars_var = cars_var
        self._library: list = []
        self._results: list = []
        self._target = None                    # (features, size, curves, label)
        self._svj_sets: dict = {}
        self._q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._img: Optional[ImageTk.PhotoImage] = None
        self._build(parent)
        self.app.after(100, self._poll)

    # ── layout ──────────────────────────────────────────────────────────────────

    def _build(self, f: ttk.Frame) -> None:
        box = ttk.LabelFrame(f, text="1 · Your Assetto Corsa cars")
        box.pack(fill="x", padx=8, pady=(8, 2))
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="Cars folder:").grid(row=0, column=0, sticky="w", padx=(6, 4), pady=4)
        ttk.Entry(box, textvariable=self.cars_var).grid(row=0, column=1, sticky="ew", pady=4)
        ttk.Button(box, text="Browse…", command=self._browse).grid(row=0, column=2, padx=4)
        ttk.Button(box, text="Find it", command=self._detect).grid(row=0, column=3, padx=(0, 4))
        self._scan_btn = ttk.Button(box, text="Scan tyres", command=self._scan)
        self._scan_btn.grid(row=0, column=4, padx=(0, 6))
        self._prog = ttk.Progressbar(box, mode="determinate")
        self._prog.grid(row=1, column=1, sticky="ew", pady=(0, 4))
        self._scan_lbl = ttk.Label(box, text="Not scanned yet. The first scan reads every car "
                                              "(about 2 minutes for 2,000 cars); later ones "
                                              "only what changed.")
        self._scan_lbl.grid(row=1, column=2, columnspan=3, sticky="w", padx=6)

        tgt = ttk.LabelFrame(f, text="2 · The tyre to match")
        tgt.pack(fill="x", padx=8, pady=2)
        tgt.columnconfigure(2, weight=1)
        self._mode = tk.StringVar(value="svj")
        ttk.Radiobutton(tgt, text="A tyre set of an SVJ file", variable=self._mode, value="svj"
                        ).grid(row=0, column=0, sticky="w", padx=6, pady=2)
        self._svj_var = tk.StringVar()
        ttk.Entry(tgt, textvariable=self._svj_var).grid(row=0, column=1, columnspan=2, sticky="ew")
        ttk.Button(tgt, text="Browse…", command=self._browse_svj).grid(row=0, column=3, padx=4)
        self._set_var = tk.StringVar()
        self._set_box = ttk.Combobox(tgt, textvariable=self._set_var, state="readonly", width=44)
        self._set_box.grid(row=0, column=4, padx=(0, 6))
        ttk.Radiobutton(tgt, text="A tyre of another car", variable=self._mode, value="lib"
                        ).grid(row=1, column=0, sticky="w", padx=6, pady=2)
        self._car_var = tk.StringVar()
        self._car_box = ttk.Combobox(tgt, textvariable=self._car_var, width=34)
        self._car_box.grid(row=1, column=1, sticky="w")
        self._car_box.bind("<<ComboboxSelected>>", lambda _e: self._car_chosen())
        self._car_box.bind("<KeyRelease>", lambda _e: self._car_filter())
        self._sec_var = tk.StringVar()
        self._sec_box = ttk.Combobox(tgt, textvariable=self._sec_var, state="readonly", width=40)
        self._sec_box.grid(row=1, column=2, sticky="w", padx=6)

        opt = ttk.LabelFrame(f, text="3 · What counts")
        opt.pack(fill="x", padx=8, pady=2)
        ttk.Label(opt, text="Axle:").pack(side="left", padx=(6, 2))
        self._axle = tk.StringVar(value="any")
        ttk.Combobox(opt, textvariable=self._axle, values=("any", "front", "rear"),
                     state="readonly", width=7).pack(side="left")
        ttk.Label(opt, text="   Size matters:").pack(side="left")
        self._size_w = tk.IntVar(value=25)
        ttk.Scale(opt, from_=0, to=100, variable=self._size_w, length=160,
                  command=lambda v: self._size_lbl.config(text=f"{int(float(v))} %")
                  ).pack(side="left", padx=4)
        self._size_lbl = ttk.Label(opt, text="25 %", width=6)
        self._size_lbl.pack(side="left")
        ttk.Label(opt, text="Show").pack(side="left", padx=(10, 2))
        self._top = tk.IntVar(value=25)
        ttk.Spinbox(opt, from_=5, to=100, textvariable=self._top, width=5).pack(side="left")
        ttk.Button(opt, text="Find similar tyres", command=self._find).pack(side="right", padx=8, pady=4)

        res = ttk.LabelFrame(f, text="4 · The closest tyres in your library")
        res.pack(fill="both", expand=True, padx=8, pady=2)
        self._tree = ttk.Treeview(res, columns=[c[0] for c in _COLUMNS], show="headings",
                                  selectmode="browse", height=14)
        for key, title, width in _COLUMNS:
            self._tree.heading(key, text=title)
            self._tree.column(key, width=width, anchor="w" if key in ("car", "name") else "center")
        sb = ttk.Scrollbar(res, orient="vertical", command=self._tree.yview)
        self._tree.configure(yscrollcommand=sb.set)
        self._tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")
        self._tree.bind("<Double-1>", lambda _e: self._compare())

        bar = ttk.Frame(f)
        bar.pack(fill="x", padx=8, pady=(2, 8))
        ttk.Button(bar, text="Compare curves", command=self._compare).pack(side="left")
        ttk.Button(bar, text="Copy tyres.ini section", command=self._copy).pack(side="left", padx=6)
        ttk.Button(bar, text="Save section…", command=self._save).pack(side="left")
        self._status = ttk.Label(bar, text="")
        self._status.pack(side="left", padx=12)

    # ── the cars folder and the scan ───────────────────────────────────────────────

    def _browse(self) -> None:
        d = filedialog.askdirectory(title="Select the AC cars folder (content/cars)")
        if d:
            self.cars_var.set(d)
            ac_paths.save_settings(cars_dir=d)

    def _detect(self) -> None:
        found = ac_paths.find_cars_dirs()
        if not found:
            messagebox.showinfo("Not found", "No Assetto Corsa installation was found in a Steam "
                                             "library. Use Browse to point at content/cars.")
            return
        self.cars_var.set(str(found[0]))
        ac_paths.save_settings(cars_dir=str(found[0]))
        self._say(f"Found {len(found)} installation(s); using {found[0]}")

    def _cache_path(self) -> Path:
        return ac_paths.settings_path().parent / "tyre_library.json"

    def _scan(self) -> None:
        root = Path(self.cars_var.get().strip())
        if not root.is_dir():
            messagebox.showinfo("No folder", "Choose the folder with your AC cars first "
                                             "(Browse, or Find it).")
            return
        ac_paths.save_settings(cars_dir=str(root))
        self._scan_btn.config(state="disabled")
        self._stop.clear()

        def worker() -> None:
            try:
                lib = T.scan_library(
                    root, self._cache_path(),
                    progress=lambda i, n, name: self._q.put(("prog", i, n, name)))
                self._q.put(("scanned", lib))
            except Exception:                                  # noqa: BLE001
                self._q.put(("error", traceback.format_exc()))

        threading.Thread(target=worker, daemon=True).start()

    # ── choosing the target ────────────────────────────────────────────────────────

    def _browse_svj(self) -> None:
        f = filedialog.askopenfilename(
            title="Select an SVJ file",
            filetypes=[("SVJ", "*.svj.json *.json"), ("All files", "*")])
        if not f:
            return
        self._svj_var.set(f)
        try:
            data = json.loads(Path(f).read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            messagebox.showerror("Not an SVJ file", str(e))
            return
        self._svj_sets = (data.get("tires") or {}).get("sets") or {}
        labels = []
        for key, s in self._svj_sets.items():
            has = "" if T.svj_set_curves(s) else "  (no force model)"
            labels.append(f"{key}   {s.get('size_code') or ''}{has}")
        self._set_box["values"] = labels
        if labels:
            self._set_box.current(0)
        self._mode.set("svj")

    def _car_filter(self) -> None:
        text = self._car_var.get().lower()
        cars = sorted({e["car"] for e in self._library})
        self._car_box["values"] = [c for c in cars if text in c.lower()][:200]

    def _car_chosen(self) -> None:
        car = self._car_var.get()
        secs = [f"{e['section']}   {e['name']}" for e in self._library if e["car"] == car]
        self._sec_box["values"] = secs
        if secs:
            self._sec_box.current(0)
        self._mode.set("lib")

    def _target_from_ui(self) -> Optional[tuple]:
        if self._mode.get() == "svj":
            idx = self._set_box.current()
            if idx < 0 or not self._svj_sets:
                messagebox.showinfo("No tyre", "Open an SVJ file and choose one of its tyre sets.")
                return None
            key = list(self._svj_sets)[idx]
            s = self._svj_sets[key]
            curves = T.svj_set_curves(s)
            if curves is None:
                messagebox.showinfo("No force model", f"The set {key} has no Pacejka block to "
                                                      "compare; only its size could be used.")
                return None
            return T.curve_features(*curves), T.size_of_svj_set(s), curves, key
        car = self._car_var.get()
        sec = self._sec_var.get().split()[0] if self._sec_var.get() else ""
        entry = next((e for e in self._library if e["car"] == car and e["section"] == sec), None)
        if entry is None:
            messagebox.showinfo("No tyre", "Scan the library, then choose a car and a tyre.")
            return None
        size = {k: entry.get(k) for k in ("width", "radius", "rim_radius", "size_code")}
        return entry["features"], size, T.entry_curves(entry), f"{car} {sec}"

    # ── matching ───────────────────────────────────────────────────────────────────

    def _find(self) -> None:
        if not self._library:
            messagebox.showinfo("Scan first", "Scan your cars folder first (step 1).")
            return
        tgt = self._target_from_ui()
        if tgt is None:
            return
        self._target = tgt
        axle = None if self._axle.get() == "any" else self._axle.get()
        self._results = T.rank(tgt[0], tgt[1], self._library, axle=axle,
                               size_weight=self._size_w.get() / 100.0, top=self._top.get())
        self._tree.delete(*self._tree.get_children())
        for i, r in enumerate(self._results):
            e = r["entry"]
            f = e["features"]
            wr = f"{(e['width'] or 0) * 1000:.0f} / {(e['radius'] or 0) * 1000:.0f} mm"
            self._tree.insert("", "end", iid=str(i), values=(
                f"{r['score']:.0f}", f"{r['behaviour']:.0f}",
                "" if r["size"] is None else f"{r['size']:.0f}", e["car"], e["section"],
                e["name"], e["size_code"] or "", wr, f"{f['mu_y']:.2f}", f"{f['mu_x']:.2f}",
                f"{f['c_y']:.1f}"))
        t = tgt[0]
        self._say(f"Target {tgt[3]}: μ lat {t['mu_y']:.2f}, μ long {t['mu_x']:.2f}, "
                  f"Cα {t['c_y']:.1f}/rad — {len(self._results)} candidates")
        if self._results:
            self._tree.selection_set("0")

    def _selected(self) -> Optional[dict]:
        sel = self._tree.selection()
        if not sel:
            messagebox.showinfo("Select a tyre", "Click one of the results first.")
            return None
        return self._results[int(sel[0])]["entry"]

    def _compare(self) -> None:
        e = self._selected()
        if e is None or self._target is None:
            return
        png = T.comparison_png(self._target[2], T.entry_curves(e), self._target[3],
                               f"{e['car']} {e['section']}")
        win = tk.Toplevel(self.app)
        win.title(f"{self._target[3]}  vs  {e['car']} {e['section']}")
        self._img = ImageTk.PhotoImage(Image.open(io.BytesIO(png)))
        ttk.Label(win, image=self._img).pack(padx=6, pady=6)

    def _section_text(self) -> Optional[str]:
        e = self._selected()
        if e is None:
            return None
        return T.tyres_ini_sections(Path(self.cars_var.get().strip()), e["car"], e["section"])

    def _copy(self) -> None:
        text = self._section_text()
        if text:
            self.app.clipboard_clear()
            self.app.clipboard_append(text)
            self._say("The tyres.ini section is on the clipboard.")

    def _save(self) -> None:
        text = self._section_text()
        if not text:
            return
        f = filedialog.asksaveasfilename(title="Save the tyres.ini section",
                                         defaultextension=".ini",
                                         filetypes=[("INI", "*.ini"), ("All files", "*")])
        if f:
            Path(f).write_text(text, encoding="utf-8")
            self._say(f"Saved {f}")

    # ── plumbing ───────────────────────────────────────────────────────────────────

    def _say(self, text: str) -> None:
        self._status.config(text=text)

    def _poll(self) -> None:
        try:
            while True:
                msg = self._q.get_nowait()
                if msg[0] == "prog":
                    self._prog["value"] = 100.0 * msg[1] / max(msg[2], 1)
                    self._scan_lbl.config(text=f"{msg[1]}/{msg[2]}  {msg[3]}")
                elif msg[0] == "scanned":
                    self._library = msg[1]
                    self._prog["value"] = 100
                    cars = len({e["car"] for e in self._library})
                    self._scan_lbl.config(text=f"{len(self._library)} tyres in {cars} cars")
                    self._scan_btn.config(state="normal")
                    self._car_filter()
                elif msg[0] == "error":
                    self._scan_btn.config(state="normal")
                    messagebox.showerror("Scan error", msg[1])
        except queue.Empty:
            pass
        self.app.after(100, self._poll)
