"""Tests for the physics a greenhouse runs under: its bent trusses.

Requires Isaac Lab, and the greenhouse's extension; skipped where either is
missing. What is pinned here is what the cfg cannot show: that Newton's rod
importer takes a bent truss as a rod held at the stem, carrying the weight of
its fruit.
"""

from __future__ import annotations

import json
import re

import pytest

# Not `pytest.importorskip`: `greenhouse_lab.isaaclab` re-raises a missing
# Isaac Lab under its own message, and pytest skips only when the error names
# the module it was asked to import.
try:
    import greenhouse_lab
    from greenhouse_lab.isaaclab import GreenhouseCfg, StemCfg
except ImportError:
    pytest.skip("Isaac Lab or greenhouse_lab is not installed", allow_module_level=True)

from newton import ModelBuilder  # noqa: E402

from misina_lab.usd.build import build_usd  # noqa: E402

BELOW = 1.8
"""A height that bends the two lowest trusses on every stem at the defaults."""


def test_a_greenhouse_bends_trusses_only_below_its_height():
    """Its static fallback is the same greenhouse with nothing below the
    height."""
    bent = GreenhouseCfg(stem=StemCfg(flexible_below=BELOW))
    assert GreenhouseCfg().without_rods() is None
    assert bent.without_rods().stem.flexible_below == 0.0
    assert bent.stem.flexible_below == BELOW, "the caller's cfg is left as it was"


def test_a_bent_truss_is_a_rod_held_at_the_stem_carrying_its_fruit(tmp_path):
    """Its first body is static, the way the importer bolts a rod down, and the
    rest weigh the fruit's loads and a few grams of rachis on top."""
    params = greenhouse_lab.GreenhouseParams()
    layout, stem = params.layout, params.stem
    (layout.rows, layout.row_length), stem.flexible_below = (1, 2.0), BELOW
    params.layout, params.stem = layout, stem
    doc = json.loads(params.generate_scene_json())
    path = tmp_path / "greenhouse.usda"
    build_usd(doc, str(path))
    builder = ModelBuilder()
    builder.add_usd(str(path))

    # Every curve's loads, by the name of the truss it stands in for.
    loads = {}

    def walk(node, at):
        if node.get("cable"):
            loads[at] = sum(node["cable"].get("loads", ()))
        for child in node.get("children", ()):
            walk(child, f"{at}/{child['name']}")

    walk(doc["root"], "")
    bodies = {}
    for body, label in enumerate(builder.body_label):
        found = re.search(r"/Rows/(.*)_edge_body_(\d+)$", label)
        if found:
            bodies.setdefault(found[1], []).append((int(found[2]), builder.body_mass[body]))
    loads = {re.search(r"/Rows/(.*)$", at)[1]: load for at, load in loads.items()}

    assert loads and bodies.keys() == loads.keys()
    for truss, chain in bodies.items():
        masses = [mass for _, mass in sorted(chain)]
        assert masses[0] == 0.0 and min(masses[1:]) > 0.0, truss
        assert 0.0 < sum(masses) - loads[truss] < 0.02, f"{truss}: {masses} for {loads[truss]} kg"
