"""
path_planner.py

PathPlanner should cannibalize both histogram_grid and polar_histogram. There
is no reason that
"""
import warnings
import math
from itertools import groupby
from operator import itemgetter

# PolarHistogram class creates an object to represent the Polar Histogram
class PathPlanner:
    def __init__(self, histogram_grid, polar_histogram, robot_location, target_location, a=200, b=1, l=5,
                 s_max=15, valley_threshold=500):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            polar_histogram: Object used to store the polar histogram.
            histogram_grid: Object used to store the grid/map of obstacles.
            a, b, l: Hyperparameters for the smoothing the polar histogram.
            s_max: Hyperparameter: the maximum number of nodes that define a wide valley
        """
        self.polar_histogram = polar_histogram
        self.histogram_grid = histogram_grid
        self.set_target_discrete_location(target_location)
        self.a = a
        self.b = b
        self.l = l
        self.s_max = s_max
        self.valley_threshold = valley_threshold
        self.target_location = target_location
        # self.robot_location = histogram_grid.get_robot_location()
        # self.target_discrete_location = histogram_grid.get_target_discrete_location()
        self.set_robot_location(robot_location)

#     //TODO: Add ability to dynamically set certainty value
#     //TODO This function may be deprecated as we restructure the robot code for ROSMOD
    def set_robot_location(self, robot_location):
        """new_location: a tuple (x, y)."""
        # self.histogram_grid.set_robot_location(robot_location)
        self.generate_histogram(robot_location)


    def set_target_discrete_location(self, target_discrete_location):
        self.histogram_grid.set_target_discrete_location(target_discrete_location)


    def generate_histogram(self, robot_location):
        # robot_location = self.histogram_grid.get_robot_location()
        robot_to_target_angle = self.histogram_grid.get_angle_between_discrete_points(self.target_location, robot_location)
        # print("path_planner: robot_to_target_angle =", robot_to_target_angle)

        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        print("path_planner: generate_histogram: robot_location =", robot_location)
        active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.histogram_grid.get_active_region(robot_location)
        print("path_planner: generate_histogram: active_region =", (active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y))
        histogram_grid = self.histogram_grid
        polar_histogram = self.polar_histogram

        # robot_location = histogram_grid.get_robot_location()

        for x in range(active_region_min_x, active_region_max_x):
            for y in range(active_region_min_y, active_region_max_y):
                node_considered = (x, y)
                certainty = histogram_grid.get_certainty_at_discrete_point(node_considered)
                distance = histogram_grid.get_continuous_distance_between_discrete_points(node_considered, robot_location)
                delta_certainty = (certainty ** 2) * (self.a - self.b * distance)
                robot_to_node_angle = histogram_grid.get_angle_between_discrete_points(robot_location, node_considered)
                polar_histogram.add_certainty_to_bin_at_angle(robot_to_node_angle, delta_certainty)
                histogram_grid.get_certainty_at_discrete_point(node_considered)

        polar_histogram.smooth_histogram(self.l)


    # TODO: We need to reorganize the polar histogram, starting NOT with the
    # target angle but with first bin closest to the target angle which does
    # not have a certainty (due to distance) and ending at max length.

    def get_filtered_polar_histogram(self):
        # print("path_planner: unfiltered =", self.polar_histogram._polar_histogram)
        filtered = [bin_index for bin_index, certainty in enumerate(self.polar_histogram._polar_histogram) if certainty < self.valley_threshold]
        # print("path_planner: filtered < %s =" % self.valley_threshold, filtered)
        return filtered


    def get_sectors(self, filtered_polar_histogram):
        # TODO: each sector needs to be sorted by wrapped angle.
        # for k, g in groupby(enumerate(self._polar_histogram), lambda (i, x): i-x):
        return [list(map(itemgetter(1), g)) for k, g in groupby(enumerate(filtered_polar_histogram), lambda ix : ix[0] - ix[1])]

    def get_obstacles(self):
        return self.histogram_grid.get_obstacles()

    def get_best_angle(self, robot_to_target_angle):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        sectors = self.get_sectors(filtered_polar_histogram)
        # print("path_planner: after filtering, sectors =", sectors)

        if len(sectors) == 0:
            # raise ValueError('path_planner: the entire histogram is a valley, given vallye threshold ' + str(self.valley_threshold))
            least_likely_bin = sorted(range(len(self.polar_histogram._polar_histogram)), key=lambda k: self.polar_histogram._polar_histogram[k])[0]
            middle_angle = self.polar_histogram.get_middle_angle_of_bin(least_likely_bin)
            warnings.warn("path_planner: the entire polar histogram is above valley threshold = %s, setting best angle to least likely bin middle angle = %s" % (self.valley_threshold, middle_angle))
            return middle_angle
        # Edge Case: there is only one sector. Raise error and cede control.
        if len(sectors) == 1:
            # raise ValueError('path_planner: the entire histogram is a valley, given vallye threshold ' + str(self.valley_threshold))
            warnings.warn("path_planner: the entire histogram is below valley_threshold = %s, setting best angle to robot_to_target_angle = %s" % (self.valley_threshold, robot_to_target_angle))
            return robot_to_target_angle / math.pi * 180

        angles = []
        for sector in sectors:
            if len(sector) > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                # k_n is the bin closest to the target direction
                if abs(sector[0] - robot_to_target_angle) > abs(sector[-1] - robot_to_target_angle):
                    k_n = 0
                    k_f = k_n + s_max - 1
                else:
                    k_n = len(sector) - 1
                    k_f = k_n - s_max + 1

            else:
                # Case 2: Narrow valley. Include all bins.
                # Order doesn't matter.
                k_n = sector[0]
                k_f = sector[-1]


            # print("k_n =", k_n)
            # print("k_f =", k_f)
            angle = (self.polar_histogram.get_middle_angle_of_bin(k_n) + self.polar_histogram.get_middle_angle_of_bin(k_f)) / 2
            # print("path_planner: angle =", angle)
            angles.append(angle)


        # print("robot_to_target_angle =", robot_to_target_angle)
        distances = [(angle, abs(robot_to_target_angle - angle)) for angle in angles]
        # print("path_planner: distances =", distances)
        smallest_distances = sorted(distances, key=itemgetter(1))
        # print("path_planner: smallest_distances =", smallest_distances)
        return smallest_distances[0][0]


    def print_histogram(self):
        print(self.polar_histogram)

    def get_object_grid(self):
        return self.histogram_grid.get_object_grid()

    def get_cell_value(self, i, j):
        return self.histogram_grid.get_cell_value(i, j)

    def get_i_max(self):
        return self.histogram_grid.get_i_max()

    def get_j_max(self):
        return self.histogram_grid.get_j_max()
