# /*
# * NOTE: it is important to distinguish between the same variables at
# * t versus t-1. Instance variables are shared across two timesteps.
# */

# NOTE: all speed and velocity units are continuous distance per timestep.tgm
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math
import time
import numpy as np
from IPython import display
import warnings


from lib.path_planner import PathPlanner
from lib.histogram_grid import HistogramGrid
from lib.polar_histogram import PolarHistogram

class Robot:
    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_speed):
        # CHANGED: we shouldn't need polar_histogram, only histogram_grid
        self.path_planner = PathPlanner(histogram_grid, polar_histogram, init_location, target_location)
#         // discretePoint location; //Stores the location of the robot
#         contPoint location; //Stores the location of the robot
        self.target_location = target_location
        self.location = init_location
        self.speed = init_speed
        self.update_angle()

    @classmethod
    def from_map(cls, map_fname, init_location, target_location, init_speed, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
        polar_histogram = PolarHistogram(num_bins)
        return cls(histogram_grid, polar_histogram, init_location, target_location, init_speed)




    def update_angle(self):
        continuous_displacement = (self.target_location[0] - self.location[0], self.target_location[1] - self.location[1])
        continuous_robot_to_target_angle = math.atan2(continuous_displacement[1], continuous_displacement[0])
        self.angle = self.path_planner.get_best_angle(continuous_robot_to_target_angle)
        self.continuous_robot_to_target_angle = continuous_robot_to_target_angle

    def set_speed(self, speed):
        self.speed = speed


    def update_velocity(self):
        angle_radian = self.angle * math.pi/180
        # old_v_x, old_v_y = self.velocity
        self.velocity = (self.speed * math.cos(angle_radian), self.speed * math.sin(angle_radian))


    def update_location(self):
        angle_radian = self.angle * math.pi/180
        velocity_x, velocity_y = self.velocity

#         delta_x = self.speed * math.cos(angle_radian)
#         delta_y = self.speed * math.sin(angle_radian)

        old_x, old_y = self.location
        self.location = (old_x + velocity_x, old_y + velocity_y)

        # Why does path_planner need discrete location?
        discrete_location = self.path_planner.histogram_grid.continuous_point_to_discrete_point(self.location)
        print("robot: discrete_location =", discrete_location)
        self.path_planner.set_robot_location(discrete_location)


#     def talk(self):
#         discrete_target_location = self.histogram_grid.get_discrete_target_location();
#         std::cout << "location is (" << location.x << ", "
#                   << location.y << "); Desired Position is ("
#                   << targetLoc.x << ", " << targetLoc.y << ")\n";
#     //Constructor for the RobotTest class
#     //CHANGED: no need for initAngle or initSpeed: robot should figure out.
#     // RobotTest(discretePoint initPos, double initAngle, double initSpeed):
#     RobotTest(discretePoint initPos):
#         grid("../map.txt", initPos, ACTIVE_REGION_SIZE_I, ACTIVE_REGION_SIZE_J, HIST_WIDTH, HIST_LENGTH, NODE_SIZE_IN),
#         hist(NUM_BINS),
#         pather(hist, &grid, A_IN, B_IN, L_IN, MAX_NUM_NODES_FOR_VALLEY_IN, VALLEY_THRESHOLD_IN)
#     {
#         location.x = initPos.x;
#         location.y = initPos.y;

#         pather.updateRobotPosition(discreteLocation);
#         grid.setTargetLoc({TARGET_X, TARGET_Y});
#     }


    #//main function per timestep
    #// 1. Get angle from nothing at t=0, then
    #// 2. get speed from nothing at t=0.
    #// 3. Given position at 0, draw simulation at t=0,
    #// 4. Now move from t=0 to t=1 by only updating the robot's position.
    def step(self, draw=True):
        self.print_histogram()
        # print("robot: angle =", self.angle)
        self.update_angle() # // angle: Null (or optionally, t-1) => t
        # self.set_speed() # // speed: Null (or optionally, t-1) => t
        self.update_velocity()
         # // at t=0, angle: t=0, speed, position: t

        self.update_location() #// position: t => t+1

    def loop(self, num_steps, draw=True):
        plt.ion()

        if draw == True:
            figure, ax = plt.subplots()
            obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles() # get a list of points [(x1, y1), (x2, y2), ...]
            paths_robot = ax.scatter(*self.location, color='blue')
            paths_target = ax.scatter(*self.path_planner.target_location, color='green')
            paths_obstacles = ax.scatter(obstacles_x, obstacles_y, color='red')
            active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
            # print("robot: loop: active_region =", (active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y))
            rectangle = ax.add_patch(
                patches.Rectangle(
                    (active_region_min_x, active_region_min_y),
                    active_region_max_x - active_region_min_x,
                    active_region_max_y - active_region_min_y,
                    fill=False      # remove background
                )
            )
            # self.draw(ax)
            display.clear_output(wait=True)
            display.display(plt.gcf())
            # plt.draw()
            # bnext = Button(ax, 'Next')
            # bnext.on_clicked(self.step_callback)
            # bprev = Button(ax, 'Previous')
            # bprev.on_clicked(callback.prev)
            warnings.warn("robot: loop: active_region = (%s, %s, %s, %s)" % (active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y))

            print("robot: loop: active_region =", (active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y))


        for i in range(num_steps):
            print("robot: loop: active_region =", self.path_planner.histogram_grid.get_active_region(self.location))
            self.step()
            if draw == True:
                # time.sleep(1)
                obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles()
                paths_robot = ax.scatter(*self.location, color='blue')
                paths_target = ax.scatter(*self.path_planner.target_location, color='green')
                paths_obstacles = ax.scatter(obstacles_x, obstacles_y, color='red')
                active_region_min_x, active_region_min_y, active_region_max_x, active_region_max_y = self.path_planner.histogram_grid.get_active_region(self.location)
                rectangle.set_bounds(active_region_min_x, active_region_min_y, active_region_max_x - active_region_min_x, active_region_max_y - active_region_min_y)
                display.clear_output(wait=True) # NOTE: Uncomment this for animation
                display.display(plt.gcf())
                # self.print_histogram()
                # figure.canvas.draw_idle()
                # plt.pause(0.1)

                # TODO: pie chart

    def step_callback(self, event):
        # print("lol")
        self.step()
        self.print_histogram()


    # def draw(self, ax):
    #     # figure = plt.gcf()
    #     # figure.clf()
    #
    #     # figure = plt.figure()
    #     # FIXME: we are now using the robot's perceived obstacles.
    #     obstacles_x, obstacles_y = self.path_planner.histogram_grid.get_obstacles() # get a list of points [(x1, y1), (x2, y2), ...]
    #     paths_robot = ax.scatter(*self.location, color='blue')
    #     paths_target = ax.scatter(*self.path_planner.target_location, color='green')
    #     paths_obstacles = ax.scatter(obstacles_x, obstacles_y, color='red')
    #

    def print_histogram(self):
        self.path_planner.print_histogram()


    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()
