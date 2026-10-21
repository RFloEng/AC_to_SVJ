"""The SVJ standard's suspension migration (corner section -> bodies, joints, elements),
vendored from RFloEng/SVJ-standard-vehicle-json tools/ (commit 045bb40, Apache-2.0)."""
from .migrate_suspension import migrate

__all__ = ["migrate"]
