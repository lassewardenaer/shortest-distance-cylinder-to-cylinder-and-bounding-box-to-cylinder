from data_classes import Cylinder, Box, Line, SimplifiedCylinder
import numpy as np
from utilities import GeometryUtilities

class CylinderToBoxDistance:

    def __init__(self, cylinder: Cylinder, box: Box, simplify_cylinder: bool=False, number_of_lines: int=4) -> None:
        """
        Attributes:
            cylinder (Cylinder): The cylinder.
            box (Box): The box.
            simplify_cylinder (bool): Whether to simplify the cylinder or not.
            number_of_lines (int): Number of lines to use when simplifying the cylinder.
        """
        self.cylinder = cylinder
        self.box = box

        self.simplify_cylinder = simplify_cylinder
        self.number_of_lines = number_of_lines
        self.simplified_cylinder: SimplifiedCylinder = None
        if self.simplify_cylinder:
            self.simplified_cylinder = self.create_simplified_cylinder()

        self.T_cylinder = GeometryUtilities.getT_cylinder(self.cylinder.R, self.cylinder.translation, self.cylinder.scaling[0], self.cylinder.scaling[2])
        self.T_box = GeometryUtilities.getT(self.box.translation, self.box.scaling)

    def shortest_distance(self) -> float:
        """
        Calculates the shortest distance between the cylinder and the box.

        Returns:
            The shortest distance between the cylinder and the box.

        """
        if self.simplify_cylinder:
            return self.shortest_distance_simplified_cyinder_to_box()
        else:
            return self.shortest_distance_cylinder_to_box()

    def shortest_distance_simplified_cyinder_to_box(self) -> float:
        """
        Calculates the shortest distance between the simplified cylinder and the box.

        Returns:
            The shortest distance between the simplified cylinder and the box.

        """
        shortest_distance_between_lines = []
        for cylinder_line in self.simplified_cylinder.lines:
            for box_line in GeometryUtilities.get_box_lines(self.box):
                shortest_distance_result = GeometryUtilities.line_to_line_distance(cylinder_line, box_line)
                _, _, _, t1, t2 = shortest_distance_result
                if t1 < 0 or t2 > 1:
                    continue
                shortest_distance_between_lines.append(shortest_distance_result)

        shortest_distance_lines_to_points = []
        for cylinder_line in self.simplified_cylinder.lines:
            for box_point in GeometryUtilities.get_box_points(self.box):
                shortest_distance_result = GeometryUtilities.line_to_point_distance(cylinder_line, box_point)
                _, _, _, t = shortest_distance_result
                if t < 0 or t > 1:
                    continue
                shortest_distance_lines_to_points.append(shortest_distance_result)

    def shortest_distance_cylinder_to_box(self) -> float:
        """
        Calculates the shortest distance between the cylinder and the box.

        Returns:
            The shortest distance between the cylinder and the box.

        """
        pass

    def create_simplified_cylinder(self) -> Cylinder:
        """
        Creates a simplified cylinder.

        Returns:
            A simplified cylinder.

        """
        cylinder_lines = []
        for radians in np.linspace(0, 2*np.pi, self.number_of_lines, endpoint=False):
            line = Line(
                pointA=np.array([np.cos(radians), np.sin(radians), -1]),
                pointB=np.array([np.cos(radians), np.sin(radians), 1])
            )
            cylinder_lines.append(line)
        return SimplifiedCylinder(self.cylinder.R, cylinder_lines, self.cylinder.translation)

    def transform_cylinder_line(self, line: Line) -> Line:
        """
        Transforms the line of the cylinder.

        Args:
            line: The line to transform.

        Returns:
            The transformed line.

        """
        line_transformed = Line(self.T_cylinder@line.pointA, self.T_cylinder@line.pointB)
        return line_transformed

    def get_simplified_cylinder_planes(self) -> list:
        """
        Gets the planes of the simplified cylinder.

        Returns:
            The planes of the simplified cylinder.

        """
        planes = []
        for i in range(len(self.simplified_cylinder.lines)-1):
            line1: Line = self.simplified_cylinder.lines[i]
            line2: Line = self.simplified_cylinder.lines[i+1]

            line1_transformed = self.transform_cylinder_line(line1)
            line2_transformed = self.transform_cylinder_line(line2)

            planes.append(GeometryUtilities.get_plane_from_two_lines(line1_transformed, line2_transformed))
        return planes
