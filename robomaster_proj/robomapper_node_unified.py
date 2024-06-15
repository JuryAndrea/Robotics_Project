import rclpy
from rclpy.node import Node
import tf_transformations
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import sys
import math


class RobomasterNode(Node):
    def __init__(self):
        super().__init__('robomaster_node')

        self.range_limit = 5.0 + 0.15   # range limit plus distance from ICR
        self.scaling = 20               # grid size
        self.speed_damper = 5.0         # the higher the slower

        self.discrete = 0.25            # computation distance of walkable points

        self.initial_pose = None
        self.current_pose = None

        # list of triples (last deltax, last deltay, movementtype)
        self.translations = []

        # Defining the path to go Upper triangle UT or lower triangle LT
        self.target_approach = None
        self.current_map = 0

        self.resetting = False  # If we have to backtrack to earlier mapping position

        self.delt_target_pose = None

        self.xtrav = True
        self.ytrav = True

        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0

        self.fig_continue = None         # figure for realtime plotting
        self.ax_continue = None          # axis for realtime plotting

        self.realtime = True             # if we are in realtime plotting mode
        self.create_fig = True           # if we have to create a new real time figure

        self.counter = 0                 # counter for 360 rotation
        self.range_f = -1                # front range of sensor

        # For 360 rotation
        self.previous_angle = 0          # previous yaw in rad
        self.spins = 0                   # how many full spins the robot has done
        self.points = []                 # points list saved as [range, yaw]

        # For mapping
        self.global_visited_points = []
        self.global_wall_points = []
        self.global_line_visited = []

        self.state = "scanning"

        # Key: state, Val: velocities [lin x, lin y, ang z]
        self.state_dict = {"scanning": [0.0, 0.0, 1.0],
                           "done": [0.0, 0.0, 0.0],
                           "target": [0.0, 0.0, 0.0],
                           "target_def": [0.0, 0.0, 0.0],
                           "stop": [0.0, 0.0, 0.0],
                           "correcting": [0.0, 0.0, 0.0]
                           }

        self.odom_pose = None
        self.odom_velocity = None

        self.vel_publisher = self.create_publisher(
            Twist, '/RM0001/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/RM0001/odom', self.odom_callback, 10)

        # Get sensor data
        self.proximity_f = self.create_subscription(
            Range, '/RM0001/range_0', self.prox_callback_f, 10)

        self.timer_counter = 0

    def start(self):
        self.timer = self.create_timer(1/100, self.timer_callback)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist

        pose2d = self.pose3d_to_2d(self.odom_pose)

        self.current_pose = pose2d

        if self.initial_pose is None:
            self.initial_pose = pose2d

    def prox_callback_f(self, msg):
        # 10.0 is the coppelia standard reading for out of range
        self.range_f = self.range_limit if msg.range == 10.0 else msg.range + 0.15

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )

        return pose2

    # Rotates robot by 360 degrees and maps the room in real time
    def rotate_360(self):
        cmd_vel = Twist()

        # Start 360 rotation under certain conditions
        if self.state == 'scanning' and self.initial_pose is not None:
            self.xtrav = True
            self.ytrav = True

            angle = self.current_pose[2]*180 / \
                np.pi if self.current_pose[2] > 0 else 360 + \
                self.current_pose[2]*180/np.pi
            angle = round(angle + (self.spins * 360.00), 2)

            cmd_vel.angular.z = (
                self.state_dict[self.state][-1]/self.speed_damper)

            # Stopping condition for 360 rotation
            if abs(angle - self.previous_angle) > 180 and self.counter > 150:  # 5 seconds minimum
                print("COMPLETED 360 ROTATION")
                self.state = "correcting"
                self.counter = 0

            # For (real time) mapping
            elif self.counter % 2 == 0 and self.range_f > 0:
                self.points.append([self.range_f, self.current_pose[2]])
                x0, y0, t = self.current_pose
                x1 = x0 + self.range_f * np.cos(t)
                y1 = y0 + self.range_f * np.sin(t)

                self.global_line_visited.append([[x0, y0], [x1, y1]])

            self.previous_angle = angle

        # Performs correcting operating of pose to move into perfect horizontal (or vertical) position
        # as we slighlty drift during 360 rotation
        if self.state == "correcting":
            if np.isclose(self.current_pose[-1], 0.0, atol=0.006) and self.counter > 50:
                plt.ioff()
                plt.close('all')
                self.spins += 1
                self.state = "done"
                self.realtime = False
                cmd_vel.angular.z = 0.0
            elif self.current_pose[-1] > 0.0:
                cmd_vel.angular.z = -0.01
            elif self.current_pose[-1] < 0.0:
                cmd_vel.angular.z = 0.01

        self.vel_publisher.publish(cmd_vel)

    # Core of our node. Responsible for the mapping and the movement of the robot
    def timer_callback(self):

        # Waiting for first callback on pose for further processing, else error
        if self.current_pose == None:
            return

        self.counter += 1

        # Start 360 rotation
        self.rotate_360()

        # Initialize cmd_vel
        cmd_vel = Twist()

        # Real time plotting
        if len(self.global_line_visited) > 0 and self.realtime:

            if self.create_fig:
                self.fig_continue, self.ax_continue = plt.subplots(
                    figsize=(5, 5))
                self.create_fig = False

            x0, y0, _ = self.current_pose

            self.ax_continue.clear()
            self.ax_continue.set_title("Realtime map " + str(self.current_map))

            global_line_visited = np.array(self.global_line_visited)
            X = global_line_visited[:, :, 0].T
            Y = global_line_visited[:, :, 1].T

            # draw lines for real time plotting between robot and sensor endpoints
            # We use plt.ion and plt.pause to update the plot in real time
            plt.plot(X, Y, color="green")
            self.ax_continue.scatter(x0, y0, marker='D')
            self.ax_continue.scatter(x0, y0, marker='D')
            self.ax_continue.set_aspect('equal')
            plt.ion()
            plt.show()
            plt.pause(interval=0.0001)

        # If we are done with 360 rotation, we compute the discretized map and save the figures based on index
        # Calls function compute all which is responsible for the discretization and mapping for all outputs
        if self.state == 'done':
            if len(self.points) >= 2:
                self.fig_continue.savefig(
                    'robomaster_proj/robomaster_proj/plot/realtime_plot_'+str(self.current_map)+'.png')
                plt.close(self.fig_continue)
                self.global_line_visited = []

                self.compute_all()

                self.realtime = True
                self.create_fig = True
                self.fig_continue = None
                self.ax_continue = None

        # If we are in target_def state, we compute the target pose and set it
        if self.state == "target_def":
            sx, sy, t = self.current_pose
            d_x = self.delt_target_pose[0]
            d_y = self.delt_target_pose[1]

            fx = sx + d_x
            fy = sy + d_y

            self.target_pos = fx, fy, 0
            self.state = "target"

        # If we are in target state, we move towards the target pose
        if self.state == "target":
            sx, sy, _ = self.current_pose
            dir_x = np.sign(self.delt_target_pose[0])
            dir_y = np.sign(self.delt_target_pose[1])

            # Moves in Upper Triangular fashion (UT)
            if self.target_approach == "UT":
                if abs(self.target_pos[1] - sy) > 0.02 and self.ytrav:
                    self.state_dict["target"] = [0.0, dir_y, 0.0]
                elif abs(self.target_pos[0] - sx) > 0.02 and self.xtrav:
                    self.ytrav = False
                    self.state_dict["target"] = [dir_x, 0.0, 0.0]
                else:
                    self.xtrav = False
                    self.state = "scanning"
                    self.resetting = False

                    self.points = []
                    self.initial_pose = None
                    self.counter = 0

                    plt.ioff()
                    plt.close('all')

            # Moves in Lower Triangular fashion (LT)
            elif self.target_approach == "LT":
                if abs(self.target_pos[0] - sx) > 0.02 and self.xtrav:
                    self.state_dict["target"] = [dir_x, 0.0, 0.0]
                elif abs(self.target_pos[1] - sy) > 0.02 and self.ytrav:
                    self.xtrav = False
                    self.state_dict["target"] = [0.0, dir_y, 0.0]
                else:
                    self.ytrav = False
                    self.state = "scanning"
                    self.resetting = False

                    self.points = []
                    self.initial_pose = None
                    self.counter = 0

                    plt.ioff()
                    plt.close('all')

        # We always publish the velocities according to the state
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z = np.array(
            self.state_dict[self.state])/self.speed_damper

        self.vel_publisher.publish(cmd_vel)

    # Populate all visited points and all wall points. Discretizes basd on a factor
    def pop_visited_wall_p(self):

        x0, y0, _ = self.current_pose

        visited = []
        wall = []

        for dist, theta in self.points:

            x1 = x0 + dist * np.cos(theta)
            y1 = y0 + dist * np.sin(theta)

            # Determines wether sensor endpoint is walkable or wall point
            if dist < self.range_limit:
                wall.append([x1, y1])
                self.global_wall_points.append([x1, y1])

            step = self.discrete

            # Compute all points between robot and sensor endpoint
            while (step < dist):
                x_mid = x0 + step * np.cos(theta)
                y_mid = y0 + step * np.sin(theta)
                visited.append([x_mid, y_mid])
                self.global_visited_points.append([x_mid, y_mid])
                step += self.discrete

            # Compute boarders for plot and discretization
            if x1 > self.max_x:
                self.max_x = round(x1, 2)
            elif x1 < self.min_x:
                self.min_x = round(x1, 2)

            if y1 > self.max_y:
                self.max_y = round(y1, 2)
            elif y1 < self.min_y:
                self.min_y = round(y1, 2)

        self.state = 'map'

        return visited, wall

    # Computes and translates all points into the positive quadrant
    def comp_offset(self, min_x, min_y, points, scale):
        max_x = 0
        max_y = 0
        offset_points = []

        for point in points:
            x, y = point

            # translate points into the positive quadrant
            x += min_x
            y += min_y

            # scale points into coordinate system
            x /= scale
            y /= scale

            x = int(round(x, 0))
            y = int(round(y, 0))

            if x > max_x:
                max_x = x
            if y > max_y:
                max_y = y

            offset_points.append((x, y))

        return offset_points, max_x, max_y

    # Populates a grid with cumulative statistics (based on votes e.g. how many times a pixel has been "seen" by the sensor)
    # according to discretized votes for both wall points (postiive values) and visited points (negative values)
    def pop_grid(self, wall_offset, visited_offset, max_x, max_y, square_grid=False):

        if square_grid:
            square_dimension = max(max_x, max_y)+1
            grid = np.zeros((square_dimension, square_dimension))
        else:
            grid = np.zeros((max_y+1, max_x+1))

        for point in wall_offset:
            x, y = point            # inverted points to conform to array logic
            grid[y][x] += 1

        # only map internal points and do not overlap with wall
        internal_only = set(visited_offset).difference(set(wall_offset))

        for point in visited_offset:
            if point in internal_only:
                x, y = point  # inverted points to conform to array logic
                grid[y][x] -= 1

        return grid.astype(int)

    # takes the cummulative grid and transforms it into an binary representation.
    # "x" if the point is a wall, "." if the point has been visited and is walkable. 0 otherwise.
    # Lastly, we cap the cumulative probability for both states according to observations
    def pop_binary_grid(self, acc_grid, x0, y0, cap_wall=1, cap_visited=-2, print_cumulative=False, print_binary=True):

        # Print cumulative grid to console
        if print_cumulative:
            print(np.flip(acc_grid, axis=0))

        binary_grid = np.where(acc_grid >= cap_wall, 'x',
                               (np.where(acc_grid <= cap_visited, '.', '0')))

        binary_grid[y0, x0] = 'ﾂ'  # This is us

        binary_grid = np.flip(binary_grid, axis=0)

        # Print binary grid to console
        if print_binary:
            print(np.array2string(binary_grid, separator=' ',
                                  formatter={'str_kind': lambda x: x}))

        return binary_grid

    # Plots the points on the map
    def map_plot(self, points, ax, marker, color):
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        ax.scatter(x, y, marker=marker, color=color)
        ax.set_aspect('equal')

    # Computes all the steps for mapping
    def compute_all(self):

        visited_points, wall_points = self.pop_visited_wall_p()
        x0, y0, _ = self.initial_pose

        # Grid for 3 sub-plots
        gs = gridspec.GridSpec(2, 2)
        figure = plt.figure(figsize=(10, 10))

        # axises for uppwer left and upper right
        ax1 = figure.add_subplot(gs[0, 0])
        ax2 = figure.add_subplot(gs[0, 1])

        # Mapping of our current position (robots position)
        ax1.scatter(x0, y0, marker='D')
        ax2.scatter(x0, y0, marker='D')

        # Plot of the current map
        self.map_plot(visited_points, ax1, marker='.', color="silver")
        self.map_plot(wall_points, ax1, marker='+', color="lightcoral")

        # Plot of the combined map
        self.map_plot(self.global_visited_points,
                      ax2, marker='.', color="gray")
        self.map_plot(self.global_wall_points, ax2, marker='+', color="red")

        # Offset points (put them in a square box)
        x_delta = self.max_x - self.min_x
        y_delta = self.max_y - self.min_y

        # Correct scaling of map
        min_x = abs(self.min_x)
        min_y = abs(self.min_y)
        abs_delta = x_delta if x_delta > y_delta else y_delta
        scale = abs_delta/self.scaling

        # Compute offset for wall points and visited points
        wall_offset, max_x_index_w, max_y_index_w = self.comp_offset(
            min_x, min_y, self.global_wall_points, scale)

        visited_offset, max_x_index_v, max_y_index_v = self.comp_offset(
            min_x, min_y, self.global_visited_points, scale)

        # Discretize initial position
        x0_d = int(round((x0 + min_x)/scale, 0))
        y0_d = int(round((y0 + min_y)/scale, 0))

        # Max index observed for grid creation
        max_x_index = max(max_x_index_v, max_x_index_w)
        max_y_index = max(max_y_index_v, max_y_index_w)

        # Get cumulative grid with votes
        grid = self.pop_grid(wall_offset, visited_offset,
                             max_x_index, max_y_index, square_grid=True)

        # inner function for normalization
        def normalize_value(x):
            norm = x.copy().astype(float)
            norm[x < 0] /= - (x[x < 0].min())
            norm[x > 0] /= x[x > 0].max()
            return norm

        normalized_data = normalize_value(grid)

        # Plotting of the heat map based on votes per pixel (lower plot in grid figure)
        ax_pcolormesh = figure.add_subplot(gs[1, :])
        c = ax_pcolormesh.pcolormesh(normalized_data, cmap='RdBu_r')
        figure.colorbar(c, ax=ax_pcolormesh)

        # Get binary grid and print
        binary_grid = self.pop_binary_grid(grid, x0_d, y0_d)

        # Get closes unobservable point
        result = self.select_route(binary_grid)

        ax1.set_title("Current map " + str(self.current_map))
        ax2.set_title("Combined map " + str(self.current_map))
        ax_pcolormesh.set_title("Heatmap " + str(self.current_map))
        figure.savefig(
            'robomaster_proj/robomaster_proj/plot/mapping_plot_'+str(self.current_map)+'.png')
        self.current_map += 1
        plt.ion()
        plt.show()
        plt.pause(interval=2)

        # If it is a tuple, meaning we have a valid route, we can proceed
        if type(result) is not type("String"):
            _, _, walkable, vertical_delta, horizontal_delta = result

        # Else we mapped everything from our current position and return to the previous for further scanning
        else:
            if len(self.translations) != 0:
                self.resetting = True
                tx, ty, approach = self.translations.pop()

                # Inversion due to discretization
                dfx = -tx
                dfy = -ty
                self.target_approach = "LT" if approach == "UT" else "UT"
                self.delt_target_pose = (dfx, dfy, 0)
                self.state = "target_def"
                return

            else:
                self.state = "stop"

                print("Node Shutdown")
                rclpy.shutdown()

                return

        case1 = walkable[0]  # Move vertically first then horizontally

        # Determine if we have to move in upper or lower triangle
        if case1:
            self.target_approach = "UT"
        else:
            self.target_approach = "LT"

        # Inverted
        dfx = horizontal_delta * scale
        dfy = -vertical_delta * scale

        self.translations.append([dfx, dfy, self.target_approach])

        # Delta from our current position to the target position
        self.delt_target_pose = (dfx, dfy, 0)

        self.state = "target_def"

    # Determines the next route to take based on the binary grid
    def select_route(self, binary):
        position = get_current_pos(binary)
        keep_looping = True
        plausible_pos = unseen_neighbors(binary)

        # If no plausible candidates are left, we have mapped everything
        if len(plausible_pos) == 0:
            print("WE HAVE MAPPED EVERYTHING")
            print(np.array2string(binary, separator=' ',
                  formatter={'str_kind': lambda x: x}))
            self.state = "stop"
            return "Mapped_All"

        # Check for nearest candidate
        else:
            while keep_looping:

                nearest = min_dist(plausible_pos, position)
                nearest_known = get_known(nearest, binary)
                walkable = check_path(position, nearest_known, binary)

                if np.all(np.asarray(walkable) == False):
                    print("KEEP SEARCHING CANDIDATES")
                    plausible_pos.remove(nearest)
                    if len(plausible_pos) == 0:
                        print("WE HAVE MAPPED EVERYTHING")
                        print(np.array2string(binary, separator=' ',
                              formatter={'str_kind': lambda x: x}))
                        return "Mapped_All"
                else:
                    print("STOP LOOPING: CANDIDATE FOUND")
                    keep_looping = False

            binary[nearest_known] = '◎'
            print(np.array2string(binary, separator=' ',
                                  formatter={'str_kind': lambda x: x}))

            dx = nearest_known[0]-position[0]
            dy = nearest_known[1]-position[1]

            print()
            print("array coordinates")
            print("OUR POSITION ﾂ:", position)
            print("NEAREST POSITION ◎:", nearest_known)
            print("WALKABLE | UT:",
                  walkable[0], " | LT:", walkable[1], " | Diag:", walkable[-1])
            print("VERTICAL TRASLATION (rounded up):", dx)
            print("HORIZONTAL TRASLATION (rounded up):", dy)
            return nearest_known, position, walkable, dx, dy


# Retrieves all plausible candidates, e.g. that have a 0 neighbor and a reachable neighbor.
def unseen_neighbors(binary):
    unseen = np.where(binary == "0")
    x, y = unseen
    plausible_pos = []
    for elem in zip(x, y):
        if candidate(binary, elem):
            plausible_pos.append(elem)
    return plausible_pos

# Retrieves the element with min distance amongst all plausible candidates


def min_dist(plausible_cand, current_pos):
    sx, sy = current_pos
    nearest = None
    dist = 100

    for cand in plausible_cand:
        fx, fy = cand
        temp_dist = math.sqrt((fx-sx)**2 + (fy-sy)**2)
        if temp_dist < dist:
            dist = temp_dist
            nearest = cand

    return nearest

# Checks whether a position is a candidate


def candidate(binary, elem):

    max_x = len(binary)
    min_x = 0
    max_y = len(binary)
    min_y = 0
    in_0_neihborhood = False
    in_reach_neighborhood = False

    x, y = elem

    elems = []
    # Border element
    if min_x < x < max_x - 1 and min_y < y < max_y - 1:
        elems.append((x+1, y))
        elems.append((x-1, y))
        elems.append((x, y+1))
        elems.append((x, y-1))
        elems.append((x+1, y+1))
        elems.append((x-1, y-1))
        elems.append((x+1, y-1))
        elems.append((x-1, y+1))

    if len(elems) == 0:
        return False

    for el in elems:
        x_t, y_t = el
        if binary[x_t, y_t] == "0":
            in_0_neihborhood = True
        if binary[x_t, y_t] == ".":
            in_reach_neighborhood = True
        if binary[x_t, y_t] == "x":
            return False

    return in_0_neihborhood and in_reach_neighborhood


# Either LT or UT
def check_path(current_pos, target_pos, binary):
    sx, sy = current_pos
    fx, fy = target_pos

    csx = sx
    csy = sy

    dx = np.sign(fx - sx)
    dy = np.sign(fy - sy)

    LT = True
    UT = True

    # UTriangular
    sx = csx
    sy = csy
    while sx != fx:
        sx += dx
        if binary[sx, sy] == "x":
            UT = False
    if UT:
        while sy != fy:
            sy += dy
            elem = binary[sx, sy]
            if elem == "x":
                UT = False

    # LTriangular
    sx = csx
    sy = csy
    while sy != fy:
        sy += dy
        elem = binary[sx, sy]
        if elem == "x":
            LT = False
    if LT:
        while sx != fx:
            sx += dx
            # CMD.Velangular.x
            if binary[sx, sy] == "x":
                LT = False

    return UT, LT

# Retrieves from the array the position we currenly have


def get_current_pos(binary):
    px, py = np.where(binary == 'ﾂ')
    position = (px[0], py[0])  # array[int] -> int
    return position

# Get closest point that is known to unknown point


def get_known(point, binary):
    x, y = point
    neighbours = [(x+1, y), (x-1, y), (x, y+1), (x, y-1),
                  (x+1, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1)]
    for elem in neighbours:
        if binary[elem[0], elem[1]] == '.':
            return elem

# Main
def main():
    np.set_printoptions(linewidth=150, legacy="1.13")
    rclpy.init(args=sys.argv)
    node = RobomasterNode()

    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
