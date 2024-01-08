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

@dataclass
class Line:
    pointA: np.ndarray
    pointB: np.ndarray

@dataclass
class SimplifiedCylinder:
    R: np.ndarray
    lines: list
    translation: np.ndarray