import numpy as np
from data_classes import Line

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

    @staticmethod
    def line_to_line_distance(line1: Line, line2: Line) -> float:
        """
        Calculates the shortest distance between two lines.

        Args:
            line1: The first line.
            line2: The second line.

        Returns:
            The shortest distance between the two lines and the closest points for line1 and line2.

        """
        direction1 = (line1.pointB - line1.pointA)
        direction2 = (line2.pointB - line2.pointA)

        perpendicular_vec = np.cross(direction1, direction2)

        # if the lines are parallel, the solution can be found with point to line distance
        if np.linalg.norm(perpendicular_vec) == 0:
            return (np.Inf, np.array([np.Inf, np.Inf, np.Inf]), np.array([np.Inf, np.Inf, np.Inf]))

        n1 = np.cross(direction1, perpendicular_vec)
        n2 = np.cross(direction2, perpendicular_vec)

        point1 = line1.pointB + np.dot((line2.pointB - line1.pointB), n2) / np.dot(direction1, n2) * direction1
        point2 = line2.pointB + np.dot((line1.pointB - line2.pointB), n1) / np.dot(direction2, n1) * direction2

        distance = np.linalg.norm(point1 - point2)

        return (distance, point1, point2)

    @staticmethod
    def point_to_line_distance(point: np.ndarray, line: Line) -> float:
        """
        Calculates the shortest distance between a point and a line.

        Args:
            point: The point.
            line: The line.

        Returns:
            The shortest distance between the point and the line.

        """
        direction = (line.pointB - line.pointA)
        t = - (np.dot(line.pointA - point, direction) / np.norm(direction) ** 2)
        closest_point = line.pointA + t * direction
        return (np.linalg.norm(point - closest_point), closest_point)