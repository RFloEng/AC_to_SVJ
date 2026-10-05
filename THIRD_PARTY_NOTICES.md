# Third-Party Notices

This project is an **original implementation**. No third-party source code is
bundled or copied into this repository. This file credits the external
algorithms, standards, and libraries the code relies on, and the projects whose
*ideas* were reimplemented here (with their licence notices, below).

## Ideas reimplemented from MIT-licensed projects

### semiloker/assetto-corsa-gltf

<https://github.com/semiloker/assetto-corsa-gltf> (`src/acgltf/convert.py`)

The KN5 → GLB exporter in `kn5_reader.py` adopts several approaches first
worked out in this project. The code here is a reimplementation against our own
parser and glTF writer, not a copy; the adapted ideas are:

- dropping runtime-variant meshes (`*_BLUR`, `*_DAMAGE`) and the low-res `_LR`
  half of in-file LOD pairs only when an `_HR` twin exists (`lowres_twins`)
- resolving a car's livery colour from the skin's flat `txDetail` map
  (`paint_slots`, `detail_tint`, `flat_colour`, `skin_dir`)
- mapping ksSpecular / ksSpecularEXP to roughness, `fresnelMaxLevel` to
  `KHR_materials_specular`, and `sunSpecular` to `KHR_materials_clearcoat`
  (`add_materials`)
- case-insensitive texture lookup (`fold_texture_case`)

No decryption is performed or adapted: encrypted `data.acd` and CSP-protected
KN5 files remain refused and flagged.

```
MIT License

Copyright (c) 2026 semiloker

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

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
| numpy      | BSD 3-Clause                           | BSD-3-Clause |
| scipy      | BSD 3-Clause                           | BSD-3-Clause |
| matplotlib | Matplotlib License (PSF/BSD-compatible)| (BSD-style)  |
| Pillow     | Historical Permission Notice (HPND)    | HPND         |
| pygltflib  | MIT                                    | MIT          |
| tkinter    | PSF (Python standard library)          | PSF-2.0      |

All of the above are permissively licensed and compatible with both
open-source and commercial use. None impose copyleft obligations on code
that merely imports them.

## Test fixtures

The `test_car/` folder contains a **synthetic reference car** (hand-crafted
data labeled as a Mazda MX-5 ND2 Club for realism). No real AC car data is
bundled with this repository.
