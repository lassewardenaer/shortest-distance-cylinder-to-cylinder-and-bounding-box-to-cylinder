import numpy as np
from data_classes import Line

class GeometryUtilities:

    @staticmethod
    def get_box_points(self, T_box) -> np.ndarray:
        """
        Calculates the points of the box.

        Input:
            T: Transformation matrix.

        Returns:
            The points of the box.

        """
        unit_cube_corners = np.array([
            [-1, -1, -1, 1],
            [1, -1, -1, 1],
            [-1, 1, -1, 1],
            [1, 1, -1, 1],
            [-1, -1, 1, 1],
            [1, -1, 1, 1],
            [-1, 1, 1, 1],
            [1, 1, 1, 1]
        ])
        transformed_corners = [np.dot(T_box, corner)[:3] for corner in unit_cube_corners]
        return transformed_corners

    @staticmethod
    def get_box_lines():
        """
        Calculates the lines of the box.

        Returns:
            The lines of the box.

        """
        box_points = GeometryUtilities.get_box_points()
        lines = []
        for i, point in enumerate(box_points):
            for j in range(i + 1, len(box_points)):
                lines.append(Line(point, box_points[j]))
        return lines

    @staticmethod
    def get_plane_from_three_points(pointA: np.ndarray, pointB: np.ndarray, pointC: np.ndarray) -> np.ndarray:
        """
        Calculates the plane defined by three points.

        Args:
            pointA: The first point.
            pointB: The second point.
            pointC: The third point.

        Returns:
            The plane defined by the three points. (a, b, c, d) where ax + by + cz + d = 0.

        """
        abc = np.cross(pointB - pointA, pointC - pointA)
        plane = np.array([abc[0], abc[1], abc[2], -np.dot(abc, pointA)])
        return plane

    @staticmethod
    def get_plane_from_two_lines(line1: Line, line2: Line) -> np.ndarray:
        """
        Calculates the plane defined by two lines.

        Args:
            line1: The first line.
            line2: The second line.

        Returns:
            The plane defined by the two lines. (a, b, c, d) where ax + by + cz + d = 0.

        """
        return GeometryUtilities.get_plane_from_three_points(line1.pointA, line1.pointB, line2.pointB)

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

        return GeometryUtilities.getT(R, translation, scaling)

    @staticmethod
    def line_to_line_distance(line1: Line, line2: Line) -> float:
        """
        Calculates the shortest distance between two lines.

        Args:
            line1: The first line.
            line2: The second line.

        Returns:
            The shortest distance between the two lines, the corresponding points and the line factors t.

        """
        direction1 = (line1.pointB - line1.pointA)
        direction2 = (line2.pointB - line2.pointA)

        perpendicular_vec = np.cross(direction1, direction2)

        # if the lines are parallel, the solution can be found with point to line distance
        if np.linalg.norm(perpendicular_vec) == 0:
            return (np.Inf, np.array([np.Inf, np.Inf, np.Inf]), np.array([np.Inf, np.Inf, np.Inf]))

        n1 = np.cross(direction1, perpendicular_vec)
        n2 = np.cross(direction2, perpendicular_vec)

        t1 = np.dot((line2.pointB - line1.pointB), n2) / np.dot(direction1, n2)
        t2 = np.dot((line1.pointB - line2.pointB), n1) / np.dot(direction2, n1)

        point1 = line1.pointB + t1 * direction1
        point2 = line2.pointB + t2 * direction2

        distance = np.linalg.norm(point1 - point2)

        return (distance, point1, point2, t1, t2)

    @staticmethod
    def point_to_line_distance(point: np.ndarray, line: Line) -> float:
        """
        Calculates the shortest distance between a point and a line.

        Args:
            point: The point.
            line: The line.

        Returns:
            The shortest distance between the point and the line, the corresponding points and the line factors t.

        """
        direction = (line.pointB - line.pointA)
        t = - (np.dot(line.pointA - point, direction) / np.norm(direction) ** 2)
        closest_point = line.pointA + t * direction
        return (np.linalg.norm(point - closest_point), closest_point, t)

    def point_to_surface_distance(point: np.ndarray, surface: np.ndarray) -> float:
        """
        Calculates the shortest distance between a point and a surface.

        Args:
            point: The point.
            surface: The surface.

        Returns:
            The shortest distance between the point and the surface.

        """
        