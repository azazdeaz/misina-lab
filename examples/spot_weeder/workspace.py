"""Prints the robot's workspace at ground level beside the paper's: the area
where the bit reaches the ground over planted feet with every leg joint in
its range, the paper's K. It is printed on flat ground, as in the paper's
Fig. 5b, and with feet on blocks, as in its Table 3 and Appendix B. Last, it
prints how much of the paper's E, measured on the real robot, is inside K.

    python workspace.py
"""

import numpy as np
import spot_weeder

STEP = 0.005
"""The spacing of the grid the areas are counted on, in metres."""

CASES = {
    "flat": ({}, 0.402, 0.402),
    "case 1": ({"fl": 0.05}, 0.420, 0.452),
    "case 2": ({"fl": 0.05, "fr": 0.10}, 0.455, 0.444),
    "case 3": ({"fl": 0.05, "hl": 0.10}, 0.459, 0.568),
    "case 4": ({"fl": 0.05, "hr": 0.10}, 0.557, 0.459),
}
"""The paper's cases: the feet raised on blocks, in metres, and the paper's
area of K in m2, as its text and Table 3 give it and as the outline in its
figure for the case encloses."""


def main():
    x, y = np.meshgrid(np.arange(0.2, 1.2, STEP), np.arange(-0.8, 0.8, STEP))
    print("K at z = 0, m2   ours    paper   paper's figure")
    for name, (raised, text, figure) in CASES.items():
        area = spot_weeder.reachable(x, y, raised=raised).sum() * STEP**2
        print(f"{name:15}  {area:.3f}   {text:.3f}   {figure:.3f}")
    in_e = spot_weeder.inside(spot_weeder.E, x, y)
    held = (in_e & spot_weeder.reachable(x, y)).sum() / in_e.sum()
    print(f"E ({in_e.sum() * STEP**2:.3f} m2) inside K on flat ground: {held:.0%}")


if __name__ == "__main__":
    main()
