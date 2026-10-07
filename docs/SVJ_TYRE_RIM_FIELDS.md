# SVJ Tyre & Rim Fields — Handoff for SVJ Analyzer

> Generated from AC → SVJ converter (beta 0.9.2).  
> All 1 297 cars in the test batch have these fields populated.

---

## 1. Where tyre/rim data lives

```
tires/
  sets/
    <set_key>/          ← one entry per axle × compound
      size_code
      dimensions/
        rim_diameter_code     ← INTEGER inches  ← RIM SIZE IS HERE
        section_width
        aspect_ratio
        section_height
        overall_diameter
      rim/
        diameter              ← FLOAT meters    ← also here
        width_nominal
      pressure
      vertical_stiffness_n_m
      vertical_damping_n_s_m
      angular_inertia_kg_m2
      relaxation_length_m
      ...
      pacejka               ← MF 6.2 block
      pacejka_mf52          ← MF 5.2 block
```

There is **no** `tires.front` / `tires.rear` axle-level block.  
Everything is under `tires.sets`.

---

## 2. Set key naming convention

```
tire_{axle}[_suffix]_{width_mm}_{asp_pct}r{rim_in}
```

| Token | Meaning | Example |
|---|---|---|
| `axle` | `front` or `rear` | `front` |
| `_suffix` | compound index (`_1`, `_2` …) or empty for plain `[FRONT]`/`[REAR]` | `_1` |
| `width_mm` | section width in mm (integer) | `205` |
| `asp_pct` | aspect ratio × 100 (integer) | `45` |
| `rim_in` | rim diameter code in inches (integer) | `17` |

### Examples

| Key | Meaning |
|---|---|
| `tire_front_205_45r17` | front, no compound suffix, 205/45R17 |
| `tire_front_1_205_45r17` | front, compound 1, 205/45R17 |
| `tire_rear_2_335_30r20` | rear, compound 2, 335/30R20 |

**Rule for front vs rear:** the axle is always the first token after `tire_`.  
A car with equal-size front and rear tyres may have a single key shared by both axles; otherwise separate keys exist.

---

## 3. Field reference — full paths and units

### Rim size

| SVJ path | Type | Unit | Notes |
|---|---|---|---|
| `tires.sets.<key>.size_code` | string | — | e.g. `"205/45R17"` |
| `tires.sets.<key>.dimensions.rim_diameter_code` | int | **inches** | Nominal rim diameter code (e.g. `17`) |
| `tires.sets.<key>.rim.diameter` | float | **metres** | `rim_diameter_code × 0.0254` |

### Tyre dimensions

| SVJ path | Type | Unit | Notes |
|---|---|---|---|
| `tires.sets.<key>.dimensions.section_width` | float | metres | e.g. `0.205` = 205 mm |
| `tires.sets.<key>.dimensions.aspect_ratio` | float | ratio | e.g. `0.45` = 45 % |
| `tires.sets.<key>.dimensions.section_height` | float | metres | `section_width × aspect_ratio` |
| `tires.sets.<key>.dimensions.overall_diameter` | float | metres | `rim.diameter + 2 × section_height` |
| `tires.sets.<key>.rim.width_nominal` | float | metres | ⚠ approximated as tyre section width (AC does not store rim width) |

### Structural / physics extras

| SVJ path | Type | Unit |
|---|---|---|
| `tires.sets.<key>.pressure` | float | Pa (hardcoded 210 000 Pa ≈ 30 psi — use `pressure_model.ideal_psi` for car-specific value) |
| `tires.sets.<key>.vertical_stiffness_n_m` | float | N/m |
| `tires.sets.<key>.vertical_damping_n_s_m` | float | N·s/m |
| `tires.sets.<key>.angular_inertia_kg_m2` | float | kg·m² |
| `tires.sets.<key>.radius_angular_k` | float | — |
| `tires.sets.<key>.relaxation_length_m` | float | metres |
| `tires.sets.<key>.speed_sensitivity` | float | — |
| `tires.sets.<key>.pressure_model.static_psi` | float | PSI |
| `tires.sets.<key>.pressure_model.ideal_psi` | float | PSI |

---

## 4. Concrete JSON examples

### Single-compound car (plain `[FRONT]`/`[REAR]` sections)
Key pattern: `tire_front_180_58r15`

```json
"tires": {
  "sets": {
    "tire_front_180_58r15": {
      "description": "AC-derived tire 180/58R15 (front)",
      "size_code": "180/58R15",
      "source": "estimated",
      "dimensions": {
        "section_width": 0.18,
        "aspect_ratio": 0.582,
        "rim_diameter_code": 15,
        "overall_diameter": 0.5905,
        "section_height": 0.1048
      },
      "rim": {
        "diameter": 0.381,
        "width_nominal": 0.18
      },
      "pressure": 210000.0,
      "vertical_stiffness_n_m": 261577.0,
      "vertical_damping_n_s_m": 500.0,
      "angular_inertia_kg_m2": 1.24,
      "relaxation_length_m": 0.07051,
      ...
    },
    "tire_rear_180_58r15": { ... }
  }
}
```

### Multi-compound car with staggered sizes
Key pattern: `tire_front_1_245_18r21`, `tire_rear_1_335_15r21`

```json
"tires": {
  "sets": {
    "tire_front_1_245_18r21": {
      "size_code": "245/18R21",
      "dimensions": {
        "section_width": 0.245,
        "aspect_ratio": 0.184,
        "rim_diameter_code": 21,
        "overall_diameter": 0.6236,
        "section_height": 0.0451
      },
      "rim": { "diameter": 0.5334, "width_nominal": 0.245 }
    },
    "tire_rear_1_335_15r21": {
      "size_code": "335/15R21",
      "dimensions": {
        "section_width": 0.335,
        "aspect_ratio": 0.15,
        "rim_diameter_code": 21,
        "overall_diameter": 0.6339,
        "section_height": 0.0503
      },
      "rim": { "diameter": 0.5334, "width_nominal": 0.335 }
    }
  }
}
```

---

## 5. How rim size is derived (for accuracy assessment)

AC `tyres.ini` stores:
- `RADIUS` — total tyre radius (metres)
- `RIM_RADIUS` — stored as `(nominal_inches + 1) × 0.0254 / 2` (**Kunos +1-inch convention**)
- `WIDTH` — section width (metres)

Converter formulas:
```
rim_diameter_code  = round(RIM_RADIUS × 2 / 0.0254) − 1
aspect_ratio       = (RADIUS − (RIM_RADIUS − 0.0127)) / WIDTH
section_height     = WIDTH × aspect_ratio
rim.diameter       = rim_diameter_code × 0.0254
overall_diameter   = rim.diameter + 2 × section_height
```

Verified correct for official Kunos cars:

| Car | Expected | SVJ output |
|---|---|---|
| ks_mazda_mx5_nd | 205/45R17 | **205/45R17** ✓ |
| ks_ferrari_f2004 | 13" F1 | **13"** ✓ |
| ks_porsche_911_gt3_r_2016 | 18" GT3 | **18"** ✓ |

⚠ **Mod/community cars** may not follow the Kunos +1 convention — their `RIM_RADIUS` values can be off, producing unusual rim codes (e.g. `14"` on a 1990s supercar).  
The `_est: true` flag on every tyre set signals that dimensions are estimated from source data, not manufacturer-verified.

---

## 6. What is NOT in the SVJ (but might be expected)

| Missing field | Status |
|---|---|
| `tires.front` / `tires.rear` axle block | Not emitted — all data is in `tires.sets` |
| `rim.width_nominal` (actual rim width) | AC does not store rim width; field is set to tyre section width as approximation |
| `pressure` (car-specific) | Hardcoded 210 000 Pa; use `pressure_model.ideal_psi` for actual ideal pressure |
| `tires.sets.<key>.csp` | Present only for CSP Extended Physics cars; omitted for vanilla AC |

---

## 7. Recommended analyzer logic to find rim size

```python
# Primary path
rim_in = svj["tires"]["sets"][set_key]["dimensions"]["rim_diameter_code"]  # int, inches

# Alternate (SI)
rim_m  = svj["tires"]["sets"][set_key]["rim"]["diameter"]                  # float, metres

# Human-readable size code (always present)
size   = svj["tires"]["sets"][set_key]["size_code"]                        # e.g. "205/45R17"

# To find the default front set key:
front_sets = [k for k in svj["tires"]["sets"] if k.startswith("tire_front")]
# first entry is compound 0 (or the only compound)
```
