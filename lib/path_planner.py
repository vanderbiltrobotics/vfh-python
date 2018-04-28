"""
path_planner.py

PathPlanner should cannibalize both histogram_grid and polar_histogram. There
is no reason that
"""
from itertools import groupby
from operator import itemgetter

# PolarHistogram class creates an object to represent the Polar Histogram
class PathPlanner:
    def __init__(self, histogram_grid, polar_histogram, robot_location, target_location, a=200, b=1, l=5,
                 s_max=15, valley_threshold=200):
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
        self.robot_to_target_angle = histogram_grid.get_angle_between_discrete_points(robot_location, target_location)
        self.set_robot_location(robot_location)

#     //TODO: Add ability to dynamically set certainty value
#     //TODO This function may be deprecated as we restructure the robot code for ROSMOD
    def set_robot_location(self, new_location):
        """new_location: a tuple (x, y)."""
        self.histogram_grid.set_robot_location(new_location)
        self.generate_histogram()


    def set_target_discrete_location(self, target_discrete_location):
        self.histogram_grid.set_target_discrete_location(target_discrete_location)


    def generate_histogram(self):
        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.histogram_grid.get_active_region()

        histogram_grid = self.histogram_grid
        polar_histogram = self.polar_histogram

        robot_location = histogram_grid.get_robot_location()

        for x in range(active_region_min_x, active_region_max_x):
            for y in range(active_region_min_y, active_region_max_y):
                node_considered = (x, y)
                certainty = histogram_grid.get_certainty_at_discrete_point(node_considered)
                distance = histogram_grid.get_continuous_distance_between_discrete_points(node_considered, robot_location)
                delta_certainty = (certainty ** 2) * (self.a - self.b * distance)
                polar_histogram.add_certainty_to_bin_at_angle(histogram_grid.get_angle_between_discrete_points(robot_location, node_considered), delta_certainty)
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

    def get_best_angle(self):
        filtered_polar_histogram = self.get_filtered_polar_histogram()
        sectors = self.get_sectors(filtered_polar_histogram)
        # print("path_planner: after filtering, sectors =", sectors)

        # Edge Case: there is only one sector. Raise error and cede control.
        if len(sectors[0]) == 1:
            raise ValueError('path_planner: the entire histogram is a valley, given vallye threshold ' + str(self.valley_threshold))

        angles = []
        for sector in sectors:
            if len(sector) > self.s_max:
                # Case 1: Wide valley. Include only s_max bins.
                # k_n is the bin closest to the target direction
                if abs(sector[0] - self.robot_to_target_angle) > abs(sector[-1] - self.robot_to_target_angle):
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


        distances = [(angle, abs(self.robot_to_target_angle - angle)) for angle in angles]
        # print("path_planner: distances =", distances)
        smallest_distances = sorted(distances, key=itemgetter(1))
        # print("path_planner: smallest_distances =", smallest_distances)
        return smallest_distances[0][0]


    def get_best_angle_old(self):
        """
        Determines the optimal travel direction for the robot based on the generated polar histogram.
        Works by finding the valley whose direction most closely matches the direction of the target.

        Returns:
            (double) angle
        """
        self.generate_histogram() # Computes the polar histogram

        polar_histogram = self.polar_histogram
        histogram_grid = self.histogram_grid
        valley_threshold = self.valley_threshold
        s_max = self.s_max
        num_bins = polar_histogram.num_bins



        robot_location = histogram_grid.get_robot_location()
        target_discrete_location = histogram_grid.get_target_discrete_location()
        robot_to_target_angle = histogram_grid.get_angle_between_discrete_points(robot_location, target_discrete_location)
        start_bin = polar_histogram.get_bin_index_from_angle(robot_to_target_angle) # Determine the bin in which the target falls

        # EDGE CASE: Determining if the target direction falls within a bin
        # This handled by finding the edges of the valleys
        if polar_histogram.get(start_bin) < valley_threshold:
            # Edge Case: Desired travel direction is within a valley - we need to search.

            # Store the indices of the edges of the valley
            leftIndex = -1
            rightIndex = -1

            negative = 1 # Used to the flip the direction of search
            divide = 2 # Used to control the direction search occurs

            count = 1; # Counter to store number of bins tested
#             int i; //Stores index to search
            # Iterating over the histogram to find valley.
            # Iteration occurs alternating in left & right direction of target
            while count <= num_bins and count <= s_max:
                # TODO: covering to int floor or ceiling?
                i = start_bin + int(negative * count / divide) # Index of bin to check next
                if polar_histogram.get(i) > valley_threshold:
                    if negative == 1:
                        rightIndex = i + 1
                    else:
                        leftIndex = i

                    # One bound of index is found. Only need to search in other direction
                    negative = -negative
                    divide = 1

                # Flipping search direction if both ends of the valley have not been found
                if leftIndex == -1 and rightIndex == -1:
                    negative = -negative

                count += 1

            if count > s_max:
                # The maximum size of a valley was reached. Write the edges at the last searched bins

                # //Stores the edges of the valley if the size has reached maxNumNodesForValley.
                # //If divide is 2, neither edge of the boundary has been found.
                # //If divide is 1, one edge of the boundary has been found.
                for j in range(divide):
                    count -= 1;
                    i = start_bin + int(negative * count / divide) # Index of bin to check next

                    if negative == 1:
                        rightIndex = i + 1 # Storing edge of valley
                    else:
                        leftIndex = i
            #  std::cout << leftIndex << " " << rightIndex << "\n";
            # Returns the average value of the bins
            bins = (rightIndex + leftIndex)//2
            return polar_histogram.get_middle_angle_of_bin(bins);

        else:
            # Normal Case: Process histogram normally.

            # Travel direction for valleys nearest to target on left and right of target
            rightTravelDir = -1
            leftTravelDir = -1

            # Finding nearest left and right valleys
            # Looping over histogram to left of target, then to the right of target
            # Stores the suggested travel direction for each of side of the histogram and selects direction closest to target
            # int i;
            # Checking left side
            # print("\n Left Side parameters: \n")
            for i in range(start_bin + 1, num_bins + 1):
                if polar_histogram.get(i) < valley_threshold:
                    # Found valley
                    rightIndex = i
                    # Iterating over valley to find other edge of the valley.
                    while polar_histogram.get(i) < valley_threshold and abs(i - rightIndex) < s_max:
                        i += 1

                    leftIndex = i
                    leftTravelDir = (rightIndex + leftIndex)/2;
                    # print("lol"<< polarHist.getIndex(leftIndex) << " " << polarHist.getIndex(rightIndex) << " " << polarHist.getIndex(leftTravelDir) << "\n")
                    break # Since loop begins iterating from the target direction, the valley must be the closest valley

            if i < num_bins / 2:
                i = num_bins - int(num_bins//2); # setting max iteration for left side
            i -= num_bins
            # // std::cout << "\ni: " <<  i << " " << polarHist.getIndex(i) << std::endl;
            # int j;
            # // Checking right side
            # // std::cout << "\n Right Side parameters: \n";
            for j in range(start_bin - 1, i, -1):
                # // std::cout << "\nj: " << j << " " << i << " " << polarHist.getValue(j) << "\n";
                if polar_histogram.get(j) < valley_threshold:
                    # Found valley
                    rightIndex = j + 1
                    # Iterating over valley to find other edge of the valley.
                    while(polar_histogram.get(j) < valley_threshold and abs(j-rightIndex) < s_max):
                        j -= 1

                    leftIndex = j + 1
                    leftTravelDir = (rightIndex + leftIndex) / 2;
                    # std::cout << "lol"<<polarHist.getIndex(leftIndex) << " " << polarHist.getIndex(rightIndex) << " " << polarHist.getIndex(rightTravelDir) << "\n";
                    break # Since loop begins iterating from the target direction, the valley must be the closest valley
            if abs(rightTravelDir - start_bin) < abs(leftTravelDir - start_bin):
                # // std::cout << "\nSelected Direction: " << polarHist.getIndex(rightTravelDir) << "\n";
                return polar_histogram.get_middle_angle_of_bin(rightTravelDir)
            else:
                # // std::cout << "\nSelected Direction: " << polarHist.getIndex(leftTravelDir) << "\n";
                return polar_histogram.get_middle_angle_of_bin(leftTravelDir)

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
