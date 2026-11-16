# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/setup_check.py (commit a286bbd, Apache-2.0).
# Only the import lines were changed.
#!/usr/bin/env python3
"""
SVJ adjustments and setups (spec §29).

A file describes one configuration: its numbers are the reference setup. `adjustments[]` says which of those
numbers can be changed and within what limits (a range or a list of allowed values); `setups[]` are named
alternatives, each a list of `path -> value` assignments. `active_setup` names the setup the file's own numbers
correspond to.

Paths address a value in the document: keys joined by '.', an array element selected by its `id` in brackets:

    constraints[spring_fl].preload        wheel_stations[FL].tire.pressure        tires.sets.front_205_45r17.pressure

An id may contain '.' (qualified ids of attached units); the dot inside brackets is not a separator.

Functions:
    parse_path(path)            -> list of (key, id or None), raises ValueError
    get_path(doc, path)         -> value, raises KeyError
    set_path(doc, path, value)  -> sets in place, raises KeyError
    check(doc)                  -> (errors, warnings)
    apply_setup(doc, setup_id)  -> new document with the setup's values (the input is not modified)

Usage:
    python tools/setup_check.py file.svj.json              # checks, lists adjustments and setups
    python tools/setup_check.py file.svj.json --apply track -o out.svj.json
"""

import copy
import json
import math
import re
import sys

_SEG = re.compile(r"^([^\[\]]+)(?:\[([^\[\]]+)\])?$")
_TOL = 1e-9


# ── paths ──────────────────────────────────────────────────────────────────────
def parse_path(path):
    """'constraints[spring_fl].preload' -> [('constraints', 'spring_fl'), ('preload', None)]."""
    if not isinstance(path, str) or not path:
        raise ValueError("a path is a non-empty string")
    parts, depth, cur = [], 0, ""
    for ch in path:
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
        if ch == "." and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    parts.append(cur)
    out = []
    for p in parts:
        m = _SEG.match(p)
        if not m:
            raise ValueError(f"bad path segment '{p}'")
        out.append((m.group(1), m.group(2)))
    return out


def _step(node, key, ident):
    if not isinstance(node, dict) or key not in node:
        raise KeyError(key)
    child = node[key]
    if ident is None:
        return child
    if not isinstance(child, list):
        raise KeyError(f"{key}[{ident}]: '{key}' is not an array")
    for item in child:
        if isinstance(item, dict) and item.get("id") == ident:
            return item
    raise KeyError(f"{key}[{ident}]: no element with that id")


def get_path(doc, path):
    node = doc
    for key, ident in parse_path(path):
        node = _step(node, key, ident)
    return node


def set_path(doc, path, value):
    segs = parse_path(path)
    node = doc
    for key, ident in segs[:-1]:
        node = _step(node, key, ident)
    key, ident = segs[-1]
    if ident is not None:
        raise KeyError(f"{path}: a path ends at a value, not at an array element")
    if not isinstance(node, dict) or key not in node:
        raise KeyError(key)
    node[key] = copy.deepcopy(value)


# ── value tests ────────────────────────────────────────────────────────────────
def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _same(a, b):
    if _num(a) and _num(b):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=_TOL)
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    return a == b


def adjustment_paths(adj):
    """The paths one adjustment drives (a linked adjustment names several)."""
    ps = []
    if isinstance(adj.get("path"), str):
        ps.append(adj["path"])
    if isinstance(adj.get("paths"), list):
        ps.extend(p for p in adj["paths"] if isinstance(p, str))
    return ps


def allows(adj, value):
    """True when `value` is inside the adjustment's range or in its list of values."""
    if isinstance(adj.get("values"), list):
        return any(_same(value, v) for v in adj["values"])
    if not _num(value):
        return False
    lo, hi = adj.get("min"), adj.get("max")
    if _num(lo) and value < lo - _TOL * max(1.0, abs(lo)):
        return False
    if _num(hi) and value > hi + _TOL * max(1.0, abs(hi)):
        return False
    return True


def on_step(adj, value):
    st, lo = adj.get("step"), adj.get("min")
    if not _num(st) or st <= 0 or not _num(lo) or not _num(value):
        return True
    k = (value - lo) / st
    return abs(k - round(k)) < 1e-6


def _label(kind, ident, i):
    return f"{kind} '{ident}'" if isinstance(ident, str) else f"{kind}[{i}]"


# ── checks ─────────────────────────────────────────────────────────────────────
def check(doc):
    errors, warnings = [], []
    advs = doc.get("adjustments")
    sets = doc.get("setups")
    if advs is None and sets is None and doc.get("active_setup") is None:
        return errors, warnings
    if advs is not None and not isinstance(advs, list):
        return ["adjustments must be an array"], warnings
    if sets is not None and not isinstance(sets, list):
        return ["setups must be an array"], warnings
    advs, sets = advs or [], sets or []

    # adjustments
    seen, by_path = set(), {}
    for i, a in enumerate(advs):
        if not isinstance(a, dict):
            errors.append(f"adjustments[{i}]: not an object")
            continue
        where = _label("adjustment", a.get("id"), i)
        if not isinstance(a.get("id"), str) or not a["id"]:
            errors.append(f"{where}: id is required")
        elif a["id"] in seen:
            errors.append(f"{where}: duplicate id")
        else:
            seen.add(a["id"])
        paths = adjustment_paths(a)
        if not paths:
            errors.append(f"{where}: needs `path` or `paths`")
            continue
        has_values, has_range = "values" in a, any(k in a for k in ("min", "max", "step"))
        if has_values and has_range:
            errors.append(f"{where}: give either `values` or `min`/`max`/`step`, not both")
        if not has_values and not has_range:
            errors.append(f"{where}: needs `values` or `min`/`max`")
        if has_values:
            if not isinstance(a["values"], list) or not a["values"]:
                errors.append(f"{where}: `values` must be a non-empty array")
            elif a.get("labels") is not None and (not isinstance(a["labels"], list) or len(a["labels"]) != len(a["values"])):
                errors.append(f"{where}: `labels` must have one entry per value")
        if has_range:
            lo, hi, st = a.get("min"), a.get("max"), a.get("step")
            for k, v in (("min", lo), ("max", hi), ("step", st)):
                if v is not None and not _num(v):
                    errors.append(f"{where}: `{k}` must be a number")
            if _num(lo) and _num(hi) and lo > hi:
                errors.append(f"{where}: min {lo} is above max {hi}")
            if _num(st) and st <= 0:
                errors.append(f"{where}: step must be positive")
        for p in paths:
            by_path.setdefault(p, []).append(a)
            try:
                cur = get_path(doc, p)
            except (KeyError, ValueError) as e:
                errors.append(f"{where}: path '{p}' does not resolve ({e})")
                continue
            if has_range and not has_values and not _num(cur):
                errors.append(f"{where}: '{p}' is not a number, so it cannot have a range (use `values`)")
                continue
            if (has_range or has_values) and not errors_for(a, where):
                if not allows(a, cur):
                    errors.append(f"{where}: the file's value {json.dumps(cur)[:60]} at '{p}' is outside the adjustment")
                elif has_range and not on_step(a, cur):
                    warnings.append(f"{where}: the file's value at '{p}' is not on the step grid (min + k·step)")

    # setups
    sseen = set()
    for i, s in enumerate(sets):
        if not isinstance(s, dict):
            errors.append(f"setups[{i}]: not an object")
            continue
        where = _label("setup", s.get("id"), i)
        if not isinstance(s.get("id"), str) or not s["id"]:
            errors.append(f"{where}: id is required")
        elif s["id"] in sseen:
            errors.append(f"{where}: duplicate id")
        else:
            sseen.add(s["id"])
        vals = s.get("set")
        if not isinstance(vals, dict):
            errors.append(f"{where}: `set` must be an object {{path: value}}")
            continue
        for p, v in vals.items():
            try:
                cur = get_path(doc, p)
            except (KeyError, ValueError) as e:
                errors.append(f"{where}: path '{p}' does not resolve ({e})")
                continue
            if _num(cur) != _num(v) and not isinstance(cur, (dict, list)) and not isinstance(v, (dict, list)):
                warnings.append(f"{where}: '{p}' changes type ({type(cur).__name__} -> {type(v).__name__})")
            cover = by_path.get(p, [])
            if not cover:
                warnings.append(f"{where}: changes '{p}', which no adjustment declares")
            elif not any(allows(a, v) for a in cover):
                errors.append(f"{where}: {json.dumps(v)[:60]} at '{p}' is outside the adjustment '{cover[0].get('id')}'")
        # a linked adjustment moved on one side only
        for a in advs:
            ps = adjustment_paths(a) if isinstance(a, dict) else []
            if len(ps) > 1 and any(p in vals for p in ps) and not all(p in vals for p in ps):
                warnings.append(f"{where}: sets only some of the linked paths of adjustment '{a.get('id')}'")

    act = doc.get("active_setup")
    if act is not None:
        if act not in sseen:
            errors.append(f"active_setup '{act}' is not a setup id")
        else:
            s = next(s for s in sets if isinstance(s, dict) and s.get("id") == act)
            for p, v in (s.get("set") or {}).items():
                try:
                    if not _same(get_path(doc, p), v):
                        warnings.append(f"active_setup '{act}': the file's value at '{p}' differs from the setup's")
                except (KeyError, ValueError):
                    pass
    return errors, warnings


def errors_for(a, where):
    """True when the adjustment's own definition is already broken (its range checks would only add noise)."""
    has_values = "values" in a
    if has_values:
        return not isinstance(a["values"], list) or not a["values"]
    lo, hi = a.get("min"), a.get("max")
    return (lo is not None and not _num(lo)) or (hi is not None and not _num(hi)) or (_num(lo) and _num(hi) and lo > hi)


def apply_setup(doc, setup_id):
    """A copy of `doc` with the values of one setup written in; `active_setup` becomes that setup."""
    s = next((x for x in doc.get("setups") or [] if isinstance(x, dict) and x.get("id") == setup_id), None)
    if s is None:
        raise KeyError(f"no setup '{setup_id}'")
    out = copy.deepcopy(doc)
    for p, v in (s.get("set") or {}).items():
        set_path(out, p, v)
    out["active_setup"] = setup_id
    return out


# ── CLI ────────────────────────────────────────────────────────────────────────
def main(argv):
    if not argv:
        print(__doc__)
        return 2
    with open(argv[0], "r", encoding="utf-8") as f:
        doc = json.load(f)
    if "--apply" in argv:
        sid = argv[argv.index("--apply") + 1]
        out = apply_setup(doc, sid)
        if "-o" in argv:
            with open(argv[argv.index("-o") + 1], "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2, ensure_ascii=False)
                f.write("\n")
        else:
            print(json.dumps(out, indent=2, ensure_ascii=False))
        return 0
    errors, warnings = check(doc)
    for a in doc.get("adjustments") or []:
        rng = f"values {a['values']}" if "values" in a else f"{a.get('min')} .. {a.get('max')}" + (f" step {a['step']}" if a.get("step") else "")
        print(f"  adjustment {a.get('id')}: {', '.join(adjustment_paths(a))}  {rng} {a.get('unit', '')}".rstrip())
    for s in doc.get("setups") or []:
        print(f"  setup {s.get('id')}{' (active)' if doc.get('active_setup') == s.get('id') else ''}: {len(s.get('set') or {})} value(s)")
    for w in warnings:
        print(f"  WARN   {w}")
    for e in errors:
        print(f"  ERROR  {e}")
    print("  OK" if not errors else f"  FAILED {len(errors)} error(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
