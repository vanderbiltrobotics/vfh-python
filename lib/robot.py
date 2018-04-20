# /*
# * NOTE: it is important to distinguish between the same variables at
# * t versus t-1. Instance variables are shared across two timesteps.
# */

# NOTE: all speed and velocity units are continuous distance per timestep.tgm
import matplotlib.pyplot as plt
import math

from lib.path_planner import PathPlanner
from lib.histogram_grid import HistogramGrid
from lib.polar_histogram import PolarHistogram

class Robot:
    def get_cell_location(self):
        return


    def __init__(self, histogram_grid, polar_histogram, init_location, target_location, init_speed):
        # CHANGED: we shouldn't need polar_histogram, only histogram_grid
        self.path_planner = PathPlanner(histogram_grid, polar_histogram, init_location, target_location)
#         // discretePoint location; //Stores the location of the robot
#         contPoint location; //Stores the location of the robot
        self.location = init_location
        self.speed = init_speed
        self.update_angle()

    @classmethod
    def from_map(cls, map_fname, init_location, target_location, init_speed, active_region_dimension, resolution, num_bins):
        histogram_grid = HistogramGrid.from_map(map_fname, active_region_dimension, resolution, init_location)
        polar_histogram = PolarHistogram(num_bins)
        return cls(histogram_grid, polar_histogram, init_location, target_location, init_speed)


    def viz_step(self):
        figure = plt.figure()
        obstacles = self.histogram_grid.get_obstacles() # get a list of points [(x1, y1), (x2, y2), ...]
        target = self.histogram_grid.get_target_location() # TODO: does a robot have a target or does a histogram have one?

        plt.plot(obstacles)

    def update_angle(self):
        self.angle = self.path_planner.get_best_angle()

    def set_speed(self, speed):
        self.speed = speed


    def update_velocity(self):
        old_v_x, old_v_y = self.velocity
        self.velocity = ()


    def update_location(self):
        angle_radian = angle * math.pi/180
        velocity_x, velocity_y = self.velocity

#         delta_x = self.speed * math.cos(angle_radian)
#         delta_y = self.speed * math.sin(angle_radian)

        old_x, old_y = self.location
        self.location = (old_x + velocity_x, old_y + velocity_y)

        # Why does path_planner need discrete location?
        discrete_location = self.histogram_grid.cont_to_discrete_point(self.location)
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
    def move(self, currentTimestep, numTimesteps):
        self.set_angle() # // angle: Null (or optionally, t-1) => t
        self.set_speed() # // speed: Null (or optionally, t-1) => t

        self.draw(currentTimestep, numTimesteps) # // at t=0, angle: t=0, speed, position: t

        self.update_location() #// position: t => t+1

    def get_polar_bins(self):
        return self.polar_histogram.getPolarBins()
