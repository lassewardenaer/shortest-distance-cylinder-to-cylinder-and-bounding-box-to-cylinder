import numpy as np

class Figure3D:
    """
    A class representing a 3D figure.

    Attributes:
    - figure: The figure object used for plotting.
    - ax: The 3D axes object used for plotting.

    Methods:
    - __init__(self, figure): Initializes the Figure3D object.
    - draw_cylinder(self, cylinderToCyilinderDistance): Draws cylinders on the plot.
    - draw_cylinder_lines(self, cordinates_top, cordinates_bottom, c): Draws lines connecting the top and bottom points of the cylinders.
    - show_axis(self): Shows the x, y, and z axes on the plot.
    """

    def __init__(self, figure):
        """
        Initializes the Figure3D object.

        Parameters:
        - figure: The figure object used for plotting.
        """
        self.figure = figure
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_aspect('equal', adjustable='box')

    def draw_cylinder(self, cylinderToCyilinderDistance):
        """
        Draws cylinders on the plot.

        Parameters:
        - cylinderToCyilinderDistance: An object representing the distance between two cylinders.
        """
        pointsA_top = [cylinderToCyilinderDistance.get_point_cylinder_circle(angle, 'A', 'top')  for angle in np.linspace(0, 2*np.pi, 10)]
        pointsA_bottom = [cylinderToCyilinderDistance.get_point_cylinder_circle(angle, 'A', 'bottom') for angle in np.linspace(0, 2*np.pi, 10)]
        pointsB_top = [cylinderToCyilinderDistance.get_point_cylinder_circle(angle, 'B', 'top') for angle in np.linspace(0, 2*np.pi, 10)]
        pointsB_bottom = [cylinderToCyilinderDistance.get_point_cylinder_circle(angle, 'B', 'bottom') for angle in np.linspace(0, 2*np.pi, 10)]

        # visualize the cylinders using plt
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_zlim(-5, 5)

        x_coords_top = [x[0] for x in pointsA_top]
        y_coords_top = [x[1] for x in pointsA_top]
        z_coords_top = [x[2] for x in pointsA_top]

        self.ax.scatter(x_coords_top, y_coords_top, z_coords_top, c='r', marker='o')

        x_coords_bottom = [x[0] for x in pointsA_bottom]
        y_coords_bottom = [x[1] for x in pointsA_bottom]
        z_coords_bottom = [x[2] for x in pointsA_bottom]

        self.ax.scatter(x_coords_bottom, y_coords_bottom, z_coords_bottom, c='r', marker='o')
        self.draw_cylinder_lines(pointsA_top, pointsA_bottom, 'r')

        x_coords_top = [x[0] for x in pointsB_top]
        y_coords_top = [x[1] for x in pointsB_top]
        z_coords_top = [x[2] for x in pointsB_top]

        self.ax.scatter(x_coords_top, y_coords_top, z_coords_top, c='b', marker='o')

        x_coords_bottom = [x[0] for x in pointsB_bottom]
        y_coords_bottom = [x[1] for x in pointsB_bottom]
        z_coords_bottom = [x[2] for x in pointsB_bottom]

        self.ax.scatter(x_coords_bottom, y_coords_bottom, z_coords_bottom, c='b', marker='o')
        self.draw_cylinder_lines(pointsB_top, pointsB_bottom, 'b')

    def draw_cylinder_lines(self, cordinates_top, cordinates_bottom, c):
        """
        Draws lines connecting the top and bottom points of the cylinders.

        Parameters:
        - cordinates_top: A list of top points of the cylinders.
        - cordinates_bottom: A list of bottom points of the cylinders.
        - c: The color of the lines.
        """
        for i in range(len(cordinates_top)):
            self.ax.plot([cordinates_top[i][0], cordinates_bottom[i][0]], [cordinates_top[i][1], cordinates_bottom[i][1]], [cordinates_top[i][2], cordinates_bottom[i][2]], c=c)


    def show_axis(self):
        """
        Shows the x, y, and z axes on the plot.
        """
        self.ax.quiver(0, 0, 0, 1, 0, 0, length=1, normalize=True, color='k')
        self.ax.quiver(0, 0, 0, 0, 1, 0, length=1, normalize=True, color='r')
        self.ax.quiver(0, 0, 0, 0, 0, 1, length=1, normalize=True, color='b')