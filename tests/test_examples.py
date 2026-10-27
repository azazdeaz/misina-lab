"""The pruning demo and the greenhouse demo keep `shear.py` the same. Each
demo is a uv project of its own and imports nothing from the other, so the
module is copied, and this keeps the two copies in step."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_the_demos_share_one_shear() -> None:
    pruning, greenhouse = (
        (ROOT / "examples" / demo / "shear.py").read_bytes()
        for demo in ("pruning_demo", "greenhouse_demo")
    )
    assert pruning == greenhouse, "edit one shear.py and copy it over the other"
