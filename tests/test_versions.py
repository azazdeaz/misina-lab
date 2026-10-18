"""The three Python packages release together under the workspace version,
`[workspace.package]` in the root `Cargo.toml`. The generators' wheels take it
from their crates; `misina-lab`'s version and the generators' pin on it are
written out by hand, and this keeps them in step."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import tomllib

ROOT = Path(__file__).resolve().parent.parent


def load(path: str) -> dict[str, Any]:
    return tomllib.loads((ROOT / path).read_text())


def test_versions_follow_the_workspace() -> None:
    version = load("Cargo.toml")["workspace"]["package"]["version"]
    assert load("python/misina-lab/pyproject.toml")["project"]["version"] == version
    for generator in ("crates/vinerylab", "crates/greenhouse-lab"):
        dependencies = load(f"{generator}/pyproject.toml")["project"]["dependencies"]
        assert f"misina-lab=={version}" in dependencies, generator
