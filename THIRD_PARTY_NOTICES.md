# Third-Party Notices

This project is an **original implementation**. No third-party source code is
bundled or copied into this repository. This file credits the external
algorithms, standards, and libraries the code relies on.

## Algorithms implemented from public knowledge

### 1. Pacejka MF 5.2 tire model

`tire_lab.py` implements the Magic Formula 5.2 pure-slip equations as
published by:

> Pacejka, H.B. *Tire and Vehicle Dynamics*, 3rd ed. Butterworth-Heinemann, 2012.

The formulas themselves are scientific facts and not copyrightable. The
implementation here is original.

### 2. SAE J670 coordinate convention

The vehicle-frame axis choice (X-forward, Y-right, Z-down) follows
SAE J670 — *Vehicle Dynamics Terminology*. This is a public engineering
standard.

## Python dependencies (not bundled)

Listed in `requirements.txt`. Installed via pip at run-time. Each ships under
its own license; none of their source is included in this repository.

| Package    | License                                | SPDX ID      |
|------------|----------------------------------------|--------------|
| gradio     | Apache License 2.0                     | Apache-2.0   |
| numpy      | BSD 3-Clause                           | BSD-3-Clause |
| scipy      | BSD 3-Clause                           | BSD-3-Clause |
| matplotlib | Matplotlib License (PSF/BSD-compatible)| (BSD-style)  |
| Pillow     | Historical Permission Notice (HPND)    | HPND         |
| socksio    | ISC                                    | ISC          |

All of the above are permissively licensed and compatible with both
open-source and commercial use. None impose copyleft obligations on code
that merely imports them.

## Test fixtures

The `test_car/` folder contains a **synthetic reference car** (hand-crafted
data labeled as a Mazda MX-5 ND2 Club for realism). No real AC car data is
bundled with this repository.
