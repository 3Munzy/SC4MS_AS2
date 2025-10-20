from spatialmath import SE3
import numpy as np

def get_T_e_c():
    """
    Approximate camera->tool transform (^eT_c).
    Adjust once camera mount is known.
    """
    # Example: camera 8cm forward, facing same direction as tool
    T_e_c = SE3(0.08, 0, 0)  # pure translation for now
    return T_e_c
