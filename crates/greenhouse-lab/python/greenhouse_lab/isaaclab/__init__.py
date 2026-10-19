"""Isaac Lab integration: spawn a generated greenhouse as a scene asset.

Importing this subpackage requires Isaac Lab, so `greenhouse_lab/__init__.py`
does not import it. `import isaaclab` from in here reaches the real Isaac Lab
package: Python 3 imports are absolute.
"""

try:
    import isaaclab  # noqa: F401
except ImportError as err:  # pragma: no cover - depends on the environment
    raise ImportError(
        "greenhouse_lab.isaaclab requires Isaac Lab. Install it alongside greenhouse_lab,"
        " or use greenhouse_lab.GreenhouseParams.write_usd() to generate a scene"
        " without it."
    ) from err

# >>> generated: imports
from .greenhouse_cfg import (
    FruitCfg,
    GreenhouseCfg,
    GutterCfg,
    LayoutCfg,
    LeafCfg,
    RailCfg,
    SceneCfg,
    StemCfg,
    TrussCfg,
    WireCfg,
)

# <<< generated: imports

__all__ = [
    # >>> generated: all
    "FruitCfg",
    "GreenhouseCfg",
    "GutterCfg",
    "LayoutCfg",
    "LeafCfg",
    "RailCfg",
    "SceneCfg",
    "StemCfg",
    "TrussCfg",
    "WireCfg",
    # <<< generated: all
]
