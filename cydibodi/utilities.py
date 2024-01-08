import numpy as np

class Utilities:

    @staticmethod
    def getT(R: np.ndarray, translation: np.ndarray, scaling: np.ndarray):
        """
        Calculates the transformation matrix.

        Args:
            R: Rotation matrix.
            translation: Translation vector.
            radius: Radius of the cylinder.
            height: Height of the cylinder.

        Returns:
            Transformation matrix.

        """
        R_homogeneous = np.vstack((R, np.array([0, 0, 0])))
        R_homogeneous = np.hstack((R_homogeneous, np.array([[0], [0], [0], [1]])))

        translation_matrix = np.array([
            [1, 0, 0, translation[0]],
            [0, 1, 0, translation[1]],
            [0, 0, 1, translation[2]],
            [0, 0, 0, 1]
        ])

        scaling_matrix = np.array([
            [scaling[0], 0,          0,          0],
            [0,          scaling[1], 0,          0],
            [0,          0,          scaling[2], 0],
            [0,          0,          0,          1]
        ])

        return translation_matrix @ scaling_matrix @ R_homogeneous

    @staticmethod
    def getT_cylinder(R: np.ndarray, translation: np.ndarray, radius: float, height: float):
        """
        Calculates the transformation matrix.

        Args:
            R: Rotation matrix.
            translation: Translation vector.
            radius: Radius of the cylinder.
            height: Height of the cylinder.

        Returns:
            Transformation matrix.

        """
        scaling = np.array([radius, radius, height])

        return Utilities.getT(R, translation, scaling)