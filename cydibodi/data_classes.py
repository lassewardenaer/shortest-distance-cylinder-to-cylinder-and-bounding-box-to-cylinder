from dataclasses import dataclass
import numpy as np

@dataclass
class Cylinder:
    R: np.ndarray
    scaling: np.ndarray
    translation: np.ndarray

@dataclass
class Box:
    scaling: np.ndarray
    translation: np.ndarray