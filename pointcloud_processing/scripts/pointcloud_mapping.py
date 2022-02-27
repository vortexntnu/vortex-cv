#!/usr/bin/env python

import numpy as np
import math

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

class PointCloudMapping():
    """
    Class used for various tasks surrounding pointcloud mappng
    """
    def object_orientation_from_xy_area(self, area_with_limits, pointcloud_data):
        """
        Reads the point cloud data from a given area

        Args:
            area_with_limits: list of data [xmin, xmax, ymin, ymax]
            pointcloud_data: poiintcloud data extracted from camera

        Returns:
            orientationdata = [x, y, z, w]
            positiondata = [x, y, z]
        """
        # Generates a readable version of the point cloud data
        assert isinstance(pointcloud_data, PointCloud2)

        xmin = area_with_limits[0]
        xmax = area_with_limits[1]
        ymin = area_with_limits[2]
        ymax = area_with_limits[3]

        # loops through the area data and adds points to a list
        point_list = []
        for x in range(xmin  -1, xmax -1):
            for y in range(ymin - 1, ymax - 1):
                pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[x,y]])
                for pt in pt_gen:
                    point_list.append([pt[0], pt[1], pt[2]])

        orientationdata, positiondata = self.points_to_plane(point_list)
        return orientationdata, positiondata

    def object_position_from_xy_point(self, x_pixel, y_pixel, pointcloud_data):
        """
        Reads the point cloud data from a given x, y coordinate

        Args:
            x_pixel: position in x direction of point you want clouddata from
            y_pixel: position in y direction of point you want clouddata from
            pointcloud_data: poiintcloud data extracted from camera

        Returns:
            Point cloud data for a point in the camera frame as list [x, y, z]
        """
        # Generates a readable version of the point cloud data
        is_pointcloud = isinstance(pointcloud_data, PointCloud2)
        if is_pointcloud:
            # Reads the point cloud data at given uvs: u = x cord, v = y cord
            pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=False, uvs=[[x_pixel, y_pixel]])
            for pt in pt_gen:
                self.pointcloud_x = pt[0]
                self.pointcloud_y = pt[1]
                self.pointcloud_z = pt[2]

        x, y, z = self.pointcloud_x, self.pointcloud_y, self.pointcloud_z
        return [x, y, z]

    def object_orientation_from_point_list(self, point_list, pointcloud_data):
        """
        Uses known points to find object and its orientation
        
        Args:
            point_list: list of points as tuples [(x,y),(x,y)]
            pointcloud_data: poiintcloud data extracted from camera
        
        Returns:
            orientationdata = [x, y, z, w] \n
            positiondata = [x, y, z]
        """
        assert isinstance(pointcloud_data, PointCloud2)
        new_point_list = []
        for point in point_list:
            pt_gen = point_cloud2.read_points(pointcloud_data, skip_nans=True, uvs=[[point[0],point[1]]])
            for pt in pt_gen:
                # new_point_list.append(pt)
                new_point_list.append([pt[0], pt[1], pt[2]])
        
        orientationdata, positiondata = self.points_to_plane(new_point_list)
        return orientationdata, positiondata

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
        Creates a middle point of n given points. n is the amount of points in points_list

        Args:
            points_list: list of points to get the middlepoint of. Must be on the form [[x1,y1,z1],[x2,y2,z2]] or [(x1,y1,z1),(x2,y2,z2)], contents must be floats.

        Returns:
            middle_point: coordinates as a list of points [x, y, z]
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
