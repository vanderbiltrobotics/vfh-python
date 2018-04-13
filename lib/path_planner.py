"""
path_planner.py

A polar histogram means this, assuming bin_width=36
(therefore num_bins = 360 / 36 = 10):


index, corresponding_angle, histogram_angle
0, 0, 123
1, 36, 0
2, 72, 30
...
9, 324, 0

(equation: i * bin_width = angle)

However, we only keep index in a flat array for histograms, so we don't natively get/set by angle
but instead translate to and from angle.
"""
# PolarHistogram class creates an object to represent the Polar Histogram
class PathPlanner:
    def __init__(self, polar_histogram, histogram_grid, a=200, b=1, l=5,
                 max_num_nodes_for_valley=15, valley_threshold=10):
        """
        Creates a Polar Histogram object with the number of bins passed.

        Args:
            polar_histogram: Object used to store the polar histogram.
            histogram_grid: Object used to store the grid/map of obstacles.
            a, b, l: Hyperparameters for the smoothing the polar histogram.
            max_num_nodes_for_valley: Hyperparameter: the maximum number of nodes that define a wide valley
        """
        self.polar_histogram = polar_histogram
        self.histogram_grid = histogram_grid
        self.a = a
        self.b = b
        self.l = l
        self.max_num_nodes_for_valley = max_num_nodes_for_valley
        self.valley_threshold = valley_threshold

#     //TODO: Add ability to dynamically set certainty value
#     //TODO This function may be deprecated as we restructure the robot code for ROSMOD
    def update_robot_position(new_location):
        self.histogram_grid.setRobotLoc(pos)


    def generate_histogram(self):
        """Builds the vector field histogram based on current position of robot and surrounding obstacles"""
        self.polar_histogram.reset()

        active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.histogram_grid.get_active_region();

        histogram_grid = self.histogram_grid
        polar_histogram = self.polar_histogram

        robot_location = histogram_grid.get_robot_location()

        for x in range(active_region_min_x, active_region_max_x):
            for y in range(active_region_min_y, active_region_max_y):
                node_considered = (x, y)
                certainty = histogram_grid.get_certainty(node_considered)
                distance = histogram_grid.get_continuous_distance_between_discrete_points(node_considered, robot_location)
                delta_certainty = (certainty ** 2) * (a - b * distance)
                polar_histogram.addValue(histogram_grid.getAngle(robot_location, node_considered), delta_certainty)
                # std::cout << curNode.x << " " << curNode.y << " " << val << "\n";

                histogram_grid.get_certainty(node_considered)
                # std::cout << curNode.x << " " << curNode.y << " " << pow( (*grid).getCertainty(curNode),2)*(a-b* (*grid).getDistance(curNode,  (*grid).getRobotLoc())) << "\n";
        print("End Histogram Generation\n")


    def get_best_angle(self):
        """
        Determines the optimal travel direction for the robot based on the generated polar histogram.
        Works by finding the valley whose direction most closely matches the direction of the target.

        Returns:
            (double) angle
        """
        self.generate_histogram() # Computes the polar histogram
        self.print_histogram()

        polar_histogram = self.polar_histogram
        histogram_grid = self.histogram_grid
        valley_threshold = self.valley_threshold
        max_num_nodes_for_valley = self.max_num_nodes_for_valley
        num_bins = polar_histogram.num_bins

        polar_histogram.smooth_histogram(self.l) # Smoothing histogram

        robot_location = histogram_grid.get_robot_location()
        target_location = histogram_grid.get_target_location()
        robot_to_target_angle = histogram_grid.get_angle_between_discrete_points(robot_location, target_location)
        # startBin represent bin at which valley finding starts
        start_bin = polar_histogram.get_bin_index_from_angle(robot_to_target_angle) # Determine the bin in which the target falls

        # EDGE CASE: Determining if the target direction falls within a bin
        # This handled by finding the edges of the valleys
        if polar_histogram.get_certainty(start_bin) < valley_threshold:
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
            while count <= num_bins and count <= max_num_nodes_for_valley:
                i = start_bin + negative * count / divide # Index of bin to check next
                if polar_histogram.get_certainty(i) > valley_threshold:
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

            if count > max_num_nodes_for_valley:
                # The maximum size of a valley was reached. Write the edges at the last searched bins

                # //Stores the edges of the valley if the size has reached maxNumNodesForValley.
                # //If divide is 2, neither edge of the boundary has been found.
                # //If divide is 1, one edge of the boundary has been found.
                for j in range(divide):
                    count -= 1;
                    i = startBin + negative * count / divide # Index of bin to check next

                    if negative == 1:
                        rightIndex = i + 1 # Storing edge of valley
                    else:
                        leftIndex = i
            #  std::cout << leftIndex << " " << rightIndex << "\n";
            # Returns the average value of the bins
            bins = (rightIndex + leftIndex)/2
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
            for i in range(startBin + 1, num_bins + 1):
                if polar_histogram.get_certainty(i) < valley_threshold:
                    # Found valley
                    rightIndex = i
                    # Iterating over valley to find other edge of the valley.
                    while polar_histogram.get_certainty(i) < valley_threshold and abs(i - rightIndex) < max_num_nodes_for_valley:
                        i += 1

                    leftIndex = i
                    leftTravelDir = (rightIndex + leftIndex)/2;
                    # print("lol"<< polarHist.getIndex(leftIndex) << " " << polarHist.getIndex(rightIndex) << " " << polarHist.getIndex(leftTravelDir) << "\n")
                    break # Since loop begins iterating from the target direction, the valley must be the closest valley

            if i < num_bins / 2:
                i = num_bins - num_bins/2; # setting max iteration for left side
            i -= num_bins
            # // std::cout << "\ni: " <<  i << " " << polarHist.getIndex(i) << std::endl;
            # int j;
            # // Checking right side
            # // std::cout << "\n Right Side parameters: \n";
            for j in range(start_bin - 1, i, -1):
                # // std::cout << "\nj: " << j << " " << i << " " << polarHist.getValue(j) << "\n";
                if polar_histogram.get_certainty(j) < valley_threshold:
                    # Found valley
                    rightIndex = j + 1
                    # Iterating over valley to find other edge of the valley.
                    while(polar_histogram.get_certainty(j) < valley_threshold and abs(j-rightIndex) < max_num_nodes_for_valley):
                        j -= 1

                    leftIndex = j + 1
                    leftTravelDir = (rightIndex + leftIndex) / 2;
                    # std::cout << "lol"<<polarHist.getIndex(leftIndex) << " " << polarHist.getIndex(rightIndex) << " " << polarHist.getIndex(rightTravelDir) << "\n";
                    break # Since loop begins iterating from the target direction, the valley must be the closest valley
            # // std::cout << "\nTarget Direction: " << startBin << "\n";
            if abs(rightTravelDir - start_bin) < abs(leftTravelDir - start_bin):
                # // std::cout << "\nSelected Direction: " << polarHist.getIndex(rightTravelDir) << "\n";
                return polar_histogram.get_middle_angle_of_bin(rightTravelDir)
            else:
                # // std::cout << "\nSelected Direction: " << polarHist.getIndex(leftTravelDir) << "\n";
                return polar_histogram.get_middle_angle_of_bin(leftTravelDir)

    def printHistogram(self):
        self.polar_histogram.printHistogram()

    def get_object_grid(self):
        return self.histogram_grid.get_object_grid()

    def get_cell_value(self, i, j):
        return self.histogram_grid.get_cell_value(i, j)

    def get_i_max(self):
        return self.histogram_grid.get_i_max()

    def get_j_max(self):
        return self.histogram_grid.get_j_max()
