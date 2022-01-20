import imp
import numpy as np
import math

class PointCloudMapping():
    """
    Class used for various tasks surrounding pointcloud mappng
    """
    def points_to_plane(self, points_list):
        """
        Function will give you an estimated plane from points in a 3D plane

        Args:
            points_list: list of points on the form [[x1,y1,z1],[x2,y2,z2]] or [(x1,y1,z1),(x2,y2,z2)], contents must be floats.

        Returns:
            vectordata: the equation as list of [a, b, c]
            middle_point: middle point cord as a list of points [x_center, y_center, z_center]
        """
        xs = []
        ys = []
        zs = []
        for point in points_list:
            xs.append(point[0])
            ys.append(point[1])
            zs.append(point[2])


        # do fit
        tmp_A = []
        tmp_b = []
        for i in range(len(xs)):
            tmp_A.append([xs[i], ys[i], 1])
            tmp_b.append(zs[i])
        b = np.matrix(tmp_b).T
        A = np.matrix(tmp_A)
        fit = (A.T * A).I * A.T * b
        errors = b - A * fit
        residual = np.linalg.norm(errors)
        planar_equation = ("%f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
        
        vectordata = []
        for i in range(3):
            numb = np.matrix.item(fit,i)
            if not math.isnan(numb):
                vectordata.append(numb)

        middle_point = self.get_middle_point(points_list)

        return vectordata, middle_point

    def get_middle_point(self, points_list):
        """
        Creates a middle point of n given points

        Args:
            points_list: list of points to get the middlepoint of. Must be on the form [[x1,y1,z1],[x2,y2,z2]] or [(x1,y1,z1),(x2,y2,z2)], contents must be floats.

        Returns:
            middle_point: cord as a list of points [x, y, z]
        """
        x_sum = 0.0
        y_sum = 0.0
        z_sum = 0.0
        for point in points_list:
            x_sum += point[0]
            y_sum += point[1]
            z_sum += point[2]
        
        n_points = len(points_list)
        x_middle_pos = x_sum/n_points
        y_middle_pos = y_sum/n_points
        z_middle_pos = z_sum/n_points

        middle_point = [x_middle_pos, y_middle_pos, z_middle_pos]
        return middle_point
