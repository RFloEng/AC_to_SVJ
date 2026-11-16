# Vendored from RFloEng/SVJ-standard-vehicle-json, tools/variant_check.py (commit a286bbd, Apache-2.0).
# Only the import lines were changed.
#!/usr/bin/env python3
"""
SVJ variants (spec §29.5).

A file describes one car: its own content is the **base**. `variants[]` are named alternatives of the car, each a
small edit of the base: another engine, other parts, a different suspension geometry. A variant has three kinds of
edit, applied in this order:

    remove    list of paths, deleted (a key, or an array element selected by its id)
    replace   {path: value}, the value at the path is replaced as a whole: an object, an array, an element of an
              array selected by id (the new element keeps that id), or a key that does not exist yet
    set       {path: value}, a value of the setup kind (spec §29.3), applied after the two above

Paths are those of spec §29.1 (see setup_check.py). Applying a variant gives a complete, standalone document with no
`variants` and `_metadata.variant` set to the variant's id; it must validate like any other file. `adjustments`,
`setups` and `active_setup` stay in the applied document, so a variant that changes what they address must also
remove or replace them.

Functions:
    check(doc)                  -> (errors, warnings)   structure and paths (the full check of each variant is in validate.py)
    apply_variant(doc, id)      -> new document (the input is not modified)

Usage:
    python tools/variant_check.py file.svj.json                       # checks, lists the variants
    python tools/variant_check.py file.svj.json --apply gt -o gt.svj.json
"""

import copy
import json
import sys

from svj_migrate import setup_check as sc

_KINDS = ("remove", "replace", "set")


def _locate(doc, path):
    """(parent, key, ident) of the last segment of `path`; the parent must resolve."""
    segs = sc.parse_path(path)
    node = doc
    for key, ident in segs[:-1]:
        node = sc._step(node, key, ident)
    key, ident = segs[-1]
    return node, key, ident


def _element(node, key, ident):
    arr = node.get(key) if isinstance(node, dict) else None
    if not isinstance(arr, list):
        raise KeyError(f"{key}[{ident}]: '{key}' is not an array")
    for i, item in enumerate(arr):
        if isinstance(item, dict) and item.get("id") == ident:
            return arr, i
    raise KeyError(f"{key}[{ident}]: no element with that id")


def _remove(doc, path):
    node, key, ident = _locate(doc, path)
    if ident is None:
        if not isinstance(node, dict) or key not in node:
            raise KeyError(key)
        del node[key]
    else:
        arr, i = _element(node, key, ident)
        del arr[i]


def _replace(doc, path, value):
    node, key, ident = _locate(doc, path)
    if ident is None:
        if not isinstance(node, dict):
            raise KeyError(f"{path}: the parent is not an object")
        node[key] = copy.deepcopy(value)
    else:
        arr, i = _element(node, key, ident)
        if not isinstance(value, dict) or value.get("id") != ident:
            raise ValueError(f"{path}: the replacement element must be an object with id '{ident}'")
        arr[i] = copy.deepcopy(value)


def _edits(variant):
    """(kind, path, value) in application order."""
    out = [("remove", p, None) for p in variant.get("remove") or []]
    out += [("replace", p, v) for p, v in (variant.get("replace") or {}).items()]
    out += [("set", p, v) for p, v in (variant.get("set") or {}).items()]
    return out


def _apply_edits(doc, variant):
    for kind, path, value in _edits(variant):
        if kind == "remove":
            _remove(doc, path)
        elif kind == "replace":
            _replace(doc, path, value)
        else:
            sc.set_path(doc, path, value)


def check(doc):
    errors, warnings = [], []
    vs = doc.get("variants")
    if vs is None:
        return errors, warnings
    if not isinstance(vs, list):
        return ["variants must be an array"], warnings
    seen = set()
    for i, v in enumerate(vs):
        if not isinstance(v, dict):
            errors.append(f"variants[{i}]: not an object")
            continue
        where = f"variant '{v['id']}'" if isinstance(v.get("id"), str) and v["id"] else f"variants[{i}]"
        if not isinstance(v.get("id"), str) or not v["id"]:
            errors.append(f"{where}: id is required")
        elif v["id"] in seen:
            errors.append(f"{where}: duplicate id")
        else:
            seen.add(v["id"])
        ok = True
        if not any(v.get(k) for k in _KINDS):
            warnings.append(f"{where}: has no `remove`, `replace` or `set`, so it equals the base")
        if v.get("remove") is not None and not (isinstance(v["remove"], list) and all(isinstance(p, str) for p in v["remove"])):
            errors.append(f"{where}: `remove` must be an array of paths")
            ok = False
        for k in ("replace", "set"):
            if v.get(k) is not None and not isinstance(v[k], dict):
                errors.append(f"{where}: `{k}` must be an object {{path: value}}")
                ok = False
        if not ok:
            continue
        paths = [p for _, p, _ in _edits(v)]
        for p in {p for p in paths if paths.count(p) > 1}:
            errors.append(f"{where}: the path '{p}' is edited more than once")
        top = [p for p in (v.get("replace") or {}) if p in ("_metadata", "variants")]
        if top:
            errors.append(f"{where}: `{top[0]}` cannot be replaced")
        work = copy.deepcopy(doc)
        work.pop("variants", None)
        for kind, path, value in _edits(v):
            try:
                if kind == "remove":
                    _remove(work, path)
                elif kind == "replace":
                    _replace(work, path, value)
                else:
                    sc.get_path(work, path)
                    sc.set_path(work, path, value)
            except KeyError as e:
                errors.append(f"{where}: {kind} '{path}' does not resolve ({str(e).strip(chr(39))})")
                break
            except ValueError as e:
                errors.append(f"{where}: {kind} '{path}': {e}")
                break
    return errors, warnings


def apply_variant(doc, variant_id):
    """A copy of `doc` with one variant applied; it has no `variants` and carries `_metadata.variant`."""
    v = next((x for x in doc.get("variants") or [] if isinstance(x, dict) and x.get("id") == variant_id), None)
    if v is None:
        raise KeyError(f"no variant '{variant_id}'")
    out = copy.deepcopy(doc)
    out.pop("variants", None)
    _apply_edits(out, v)
    out.setdefault("_metadata", {})["variant"] = variant_id
    return out


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    with open(argv[0], "r", encoding="utf-8") as f:
        doc = json.load(f)
    if "--apply" in argv:
        out = apply_variant(doc, argv[argv.index("--apply") + 1])
        text = json.dumps(out, indent=2, ensure_ascii=False) + "\n"
        if "-o" in argv:
            with open(argv[argv.index("-o") + 1], "w", encoding="utf-8") as f:
                f.write(text)
        else:
            print(text)
        return 0
    errors, warnings = check(doc)
    for v in doc.get("variants") or []:
        n = {k: len(v.get(k) or []) for k in _KINDS}
        print(f"  variant {v.get('id')}: {v.get('name', '')}  (remove {n['remove']}, replace {n['replace']}, set {n['set']})")
    for w in warnings:
        print(f"  WARN   {w}")
    for e in errors:
        print(f"  ERROR  {e}")
    print("  OK" if not errors else f"  FAILED {len(errors)} error(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
