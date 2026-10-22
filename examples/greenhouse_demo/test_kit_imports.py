"""The script's module-level imports must leave `pxr` unloaded.

Kit ships its own `pxr` and wins the import only if nothing loaded the pip one
first; when one is already in `sys.modules`, Kit's extensions bind against it
and the app dies during startup. So every `pxr` user reachable from `shot`
imports it inside a function, after `launch_simulation` has run. The same
holds for `bend`.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

pytest.importorskip("isaaclab", reason="Isaac Lab is not installed")


@pytest.mark.parametrize("script", ["shot", "bend"])
def test_importing_a_script_does_not_load_pxr(script):
    # A subprocess, so the check is not fooled by another test's imports.
    subprocess.run(
        [sys.executable, "-c", f"import {script}, sys; assert 'pxr' not in sys.modules"],
        cwd=pathlib.Path(__file__).parent,
        check=True,
    )
