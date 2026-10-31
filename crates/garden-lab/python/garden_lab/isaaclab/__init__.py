"""Isaac Lab integration: spawn a generated garden as a scene asset.

Importing this subpackage requires Isaac Lab, so `garden_lab/__init__.py`
does not import it. `import isaaclab` from in here reaches the real Isaac Lab
package: Python 3 imports are absolute.
"""

try:
    import isaaclab  # noqa: F401
except ImportError as err:  # pragma: no cover - depends on the environment
    raise ImportError(
        "garden_lab.isaaclab requires Isaac Lab. Install it alongside garden_lab,"
        " or use garden_lab.GardenParams.write_usd() to generate a scene without it."
    ) from err

# >>> generated: imports
from .garden_cfg import (
    BedCfg,
    BushBeanCfg,
    CarrotCfg,
    GardenCfg,
    KaleCfg,
    LettuceCfg,
    PhysicsCfg,
    PlantingCfg,
    PoleBeanCfg,
    SceneCfg,
)

# <<< generated: imports

__all__ = [
    # >>> generated: all
    "BedCfg",
    "BushBeanCfg",
    "CarrotCfg",
    "GardenCfg",
    "KaleCfg",
    "LettuceCfg",
    "PhysicsCfg",
    "PlantingCfg",
    "PoleBeanCfg",
    "SceneCfg",
    # <<< generated: all
]
