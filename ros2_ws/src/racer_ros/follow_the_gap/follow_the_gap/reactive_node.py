import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        # /////////// declaring the params \\\\\\\\\\
        self.declare_parameter('bubble_radius', 0.3) # in meters
        self.declare_parameter('preporcess_conv_size', 3) # the window to average values
        self.declare_parameter('max_lidar_range', 3.0) # meters
        self.declare_parameter('speed_max', 2.0)
        self.declare_parameter('steering_gain', 1.0) # proportional gain
        self.declare_parameter('max_steering', 30.0) # proportional gain

        # ── straightaway detection (predictive lookahead) ──
        # fraction of max_lidar_range, not an absolute meters value — ranges
        # are clipped to max_lidar_range in preprocess_lidar(), so an
        # absolute threshold above max_lidar_range would be unsatisfiable
        # and silently disable straight mode entirely
        self.declare_parameter('straight_min_forward_frac', 0.75)
        self.declare_parameter('straight_max_angle_deg', 6.0) # current steering must be under this too
        self.declare_parameter('straight_confirm_frames', 8) # consecutive frames before locking in straight mode

        # ── lap tracking ──
        self.declare_parameter('lap_leave_dist', 3.0) # meters — must get this far from start before a lap can arm
        self.declare_parameter('lap_return_dist', 1.5) # meters — within this counts as crossing the line
        self.declare_parameter('lap_min_time', 5.0) # seconds — guards against re-triggering on the same crossing

        # ── dashboard lidar view / predicted path ──
        self.declare_parameter('wheelbase', 0.25) # meters — matches f1tenth_stack/config/vesc.yaml
        self.declare_parameter('lidar_view_every_n', 10) # publish /lidar_view every Nth scan (throttle)

        # ////////// get parameters (just retrieving what was previously declared) \\\\\\\\\
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preporcess_conv_size = self.get_parameter('preporcess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_max = self.get_parameter('speed_max').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.current_speed = 0.0
        self.max_steering = np.deg2rad(self.get_parameter('max_steering').value)
        self.fov = np.deg2rad(100) # how much we want the car to see infront of it
        self.prev_steering = 0.0
        self.steering_smoothing = 0.6
        self.recovery_start_time = None

        self.straight_min_forward_frac = self.get_parameter('straight_min_forward_frac').value
        self.straight_max_angle = np.deg2rad(self.get_parameter('straight_max_angle_deg').value)
        self.straight_confirm_frames = self.get_parameter('straight_confirm_frames').value
        self.straight_streak = 0
        self.in_straight_mode = False

        self.lap_leave_dist = self.get_parameter('lap_leave_dist').value
        self.lap_return_dist = self.get_parameter('lap_return_dist').value
        self.lap_min_time = self.get_parameter('lap_min_time').value
        self.start_pos = None
        self.has_left_start = False
        self.lap_count = 0
        self.lap_start_time = None
        self.last_lap_time = 0.0
        self.best_lap_time = None

        self.wheelbase = self.get_parameter('wheelbase').value
        self.lidar_view_every_n = self.get_parameter('lidar_view_every_n').value
        self.lidar_view_counter = 0
        
        # /////////// FOR ODOM \\\\\\\\\\\\
        self.actual_speed = 0.0
        self.stuck_counter = 0
        self.stuck_threshold = 25 # hz scan rate
        self.is_recovering = False
        self.recovery_counter = 0
        self.post_recovery_bias = 0.0
        self.post_recovery_counter = 0
        
        # /////////// topic names \\\\\\\\\\\\\\
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # /////// subs \\\\\\\
        # what the lidar produces
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10) # 10 for the refresh rate
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom',self.odom_callback, 10)
        
        # /////// pubs \\\\\\\
        # the commands for driving (steering / speed)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10) # 10 is refresh rate
        self.debug_pub = self.create_publisher(Float32MultiArray, '/car_info_debug', 10)
        self.lap_pub = self.create_publisher(Float32MultiArray, '/lap_info', 10)
        self.lidar_view_pub = self.create_publisher(Float32MultiArray, '/lidar_view', 10)

        
        # ////////// DEBUGGER \\\\\\\\\
        self.get_logger().info("\/"*15)
        self.get_logger().info(f"Bubble: {self.bubble_radius}m")
        self.get_logger().info(f"MAX liDAR Range: {self.max_lidar_range}m")
        self.get_logger().info(f"max speed: {self.speed_max}m/s")
        self.get_logger().info(f"\/"*15)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.actual_speed = msg.twist.twist.linear.x

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        now = self.get_clock().now()

        if self.start_pos is None:
            # first odom message we've ever seen — treat this pose as the
            # start/finish line rather than reading it from sim.yaml, so
            # this works unchanged on real hardware too.
            self.start_pos = (pos_x, pos_y)
            self.lap_start_time = now
            return

        dist_from_start = np.hypot(pos_x - self.start_pos[0], pos_y - self.start_pos[1])

        if not self.has_left_start:
            if dist_from_start > self.lap_leave_dist:
                self.has_left_start = True
        else:
            if dist_from_start < self.lap_return_dist:
                elapsed = (now - self.lap_start_time).nanoseconds / 1e9
                if elapsed > self.lap_min_time:
                    self.lap_count += 1
                    self.last_lap_time = elapsed
                    if self.best_lap_time is None or elapsed < self.best_lap_time:
                        self.best_lap_time = elapsed
                    self.lap_start_time = now
                    self.has_left_start = False
                    self.get_logger().info(
                        f'LAP {self.lap_count} complete: {elapsed:.2f}s (best: {self.best_lap_time:.2f}s)'
                    )

        self._publish_lap_info(now)

    def _publish_lap_info(self, now):
        current_lap_elapsed = 0.0
        if self.lap_start_time is not None:
            current_lap_elapsed = (now - self.lap_start_time).nanoseconds / 1e9

        lap_msg = Float32MultiArray()
        lap_msg.data = [
            float(self.lap_count),
            float(current_lap_elapsed),
            float(self.last_lap_time),
            float(self.best_lap_time if self.best_lap_time is not None else 0.0),
        ]
        self.lap_pub.publish(lap_msg)

    def preprocess_lidar(self, ranges):
        # preprocess the lidar scan array
        
        # 1. set each value to mean over some window
        processed_ranges = np.array(ranges) 
        
        # get rid of all the nan values
        processed_ranges = np.nan_to_num(processed_ranges, nan=0.0)
        
        # mean each value in range -> [inside that window]
        # moving average
        weights = np.repeat(1.0, self.preporcess_conv_size) / self.preporcess_conv_size
        processed_ranges = np.convolve(processed_ranges, weights, 'same') # makes sure that average is only calculated when window fully overlaps
        
        # ----------------------------------------------
        
        # 2. reject high values (> max lidar range)
        
        # for loop to go over every value in the range
        # if statement to check if value is in lidar range
        # if not in range set to set to max range if out of range
        
        # or just use clip
        processed_ranges = np.clip(processed_ranges, 0, self.max_lidar_range) # clip between 
        
        return processed_ranges #TODO: implement preprocessing

    
    def lidar_callback(self, data):
        ranges = np.array(data.ranges)
        # ------------------------ {PROCESSING} ------------------------

        # step 1: preprocess
        processed_ranges = self.preprocess_lidar(ranges) # we want the values to be between 0 and self.max_lidar_range. 
                                                         # NO nan NO inf NO less than 0 or greater than max_lidar_range
        
        # we also want to keep the car from looking at too much 
        center_index = int((0 - data.angle_min) / data.angle_increment)
        beams_per_side = int(self.fov / data.angle_increment)
        start = max(0, center_index - beams_per_side)
        end = min(len(processed_ranges), center_index+beams_per_side)
        processed_ranges = processed_ranges[start:end]
        
        # TODO: find closest points
        
        # whatever value is the smallest
        copied_ranges = processed_ranges.copy() # dont want to make changes to the current ranges
        non_zero = copied_ranges[copied_ranges>0] # we only want the nonzero values
        
        if len(non_zero) == 0:
            self.get_logger().warn('NO VALID LIDAR RANGES')
            self.publish_drive(0.0, 0.0)
            return
        
        smallest_dist = np.min(non_zero) # only care about the non zero valeus to find the smallest
        smallest_dist = smallest_dist if smallest_dist != 0 else 0.1 # small angle approx
        smallest_index = np.argmin(copied_ranges) # fining the index
        
        # TODO: eliminate points inside bubble
              
        # we want to know where the bubble will start and where it ends
          
        bubble_indices = self.calc_bubble(data, smallest_dist, smallest_index, len(copied_ranges))
        
        # then set the index to 0 
        copied_ranges[bubble_indices] = 0.0 # this is to set all values within the bubble to 0. MAKE THEM KNOWN THEY ARE CLOSE AND UNSAFE
        # this way when follow the gap procedure is done, everything is excluded within the bubble
        

        # stuck checker
        start_gap_index, end_gap_index = self.find_max_gap(copied_ranges)


        if self.current_speed > 0.3:
            moving_correctly = self.actual_speed > 0.1
        elif self.current_speed < -0.3:
            moving_correctly = self.actual_speed < -0.1  # reversing successfully
        else:
            moving_correctly = True  # not commanding motion, not stuck

        commanded_motion = abs(self.current_speed) > 0.3

        if commanded_motion and not moving_correctly:
            self.stuck_counter += 1
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)  # decay instead of hard reset

        if self.recovery_logic(): return

        # TODO: find best point in gap
        # find the max value in the gap
        best_index = self.max_val_in_gap(copied_ranges, start_gap_index, end_gap_index)
        
        # TODO: calc steering angle to best point
        # ------------------------ {STEERING AND SPEED CALCULATIONS} ------------------------
        # convert index to angle
        angle_to_best = data.angle_min + (best_index+start) * data.angle_increment
        
        # gain is more aggressive at sharper turns
        gain = self.steering_gain * (1.5 - 0.5 * (abs(angle_to_best) / self.max_steering))
        
        # steering gain
        steering_angle = gain * angle_to_best
        
        # clamp the steering angle to not go out of bounds
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        # LOW PASS FILTER (used in sensor fusion)
        # allows slow, steady signals to pass while blocking higher frequency noise
        
        # angle = alpha * current + (1 - alpha) * angle
        alpha = 0.8 # smoothing on straights / non on corners


        old_steering = self.prev_steering


        steering_angle = alpha * self.prev_steering + (1 - alpha) * steering_angle
        # TODO: calc speed based on steering angle and gap size
        
        # take anything infront of the car
        forward_beams = int(np.deg2rad(25) / data.angle_increment)
        mid = len(processed_ranges) // 2
        forward_slice = processed_ranges[mid - forward_beams : mid + forward_beams  ]
        min_forward = np.min(forward_slice[forward_slice > 0]) if np.any(forward_slice > 0) else self.max_lidar_range  # find the minimum nonzero value infront of car
        
        # if min_forward / max(self.current_speed, 0.1)  < 0.5:
        #     self.get_logger().warn('ABOUT TO CRASH')
        #     # left_half = processed_ranges[len(processed_ranges)//2:]
        #     # right_half = processed_ranges[:len(processed_ranges)//2]
            
        #     # left_space = np.sum(left_half)
        #     # right_space = np.sum(right_half)
        #     # self.get_logger().info(f"left_space info: {left_space} | right_space info: {right_space}")

            
        #     # recovery_steering = self.max_steering if left_space >right_space else -self.max_steering
        #     # self.get_logger().info(f"steering this way: {np.rad2deg(recovery_steering)}")
        #     # self.publish_drive(recovery_steering, -0.5) #slow down
        #     return

        # steeper distance scaling, breaking distance grows with speed
        # distance_factor = np.clip((min_forward / self.max_lidar_range) ** 1.5, 0.2, 1.0)
        gap_depth = np.mean(copied_ranges[start_gap_index:end_gap_index+1])
        distance_factor = np.clip(gap_depth / self.max_lidar_range, 0.3, 1.0)
        # more steering -> slower speeds
        # PURE PURSUIT METHOD (speed = max * cos(angle)^2 * distance)

        # ── straightaway detection: predictive lookahead ──
        # min_forward already looks ~25deg ahead of the car. Combined with
        # a small CURRENT steering angle, a long clear forward range means
        # the track is straight here — not just that this one frame
        # happened to compute a small angle. straight_streak debounces it
        # so a single noisy frame can't unlock/relock max speed; without
        # this, distance_factor (capped by max_lidar_range, which is often
        # short) keeps throttling speed even on genuine straights.
        straight_frame = (min_forward > self.straight_min_forward_frac * self.max_lidar_range) and (abs(steering_angle) < self.straight_max_angle)
        self.straight_streak = min(self.straight_streak + 1, self.straight_confirm_frames) if straight_frame else 0
        self.in_straight_mode = self.straight_streak >= self.straight_confirm_frames

        if self.in_straight_mode:
            speed = self.speed_max
        else:
            speed = self.speed_max * (np.cos(steering_angle)**4) * distance_factor # we want to lower the speed based on how close the object infront of us is

        steering_delta = steering_angle - old_steering

        # steering rate limiter ...
        max_steer_rate = np.deg2rad(4.0)  # per lidar update

        delta = steering_angle - self.prev_steering

        delta = np.clip(
            delta,
            -max_steer_rate,
            max_steer_rate
        )

        steering_angle = self.prev_steering + delta

        self.prev_steering = steering_angle
        if not self.in_straight_mode:
            if abs(steering_angle) > 0.2 and steering_delta * steering_angle > 0:
                speed *= 0.6 # entering a turn -> must break
            elif abs(steering_angle) < 0.15 and abs(self.prev_steering) > 0.3:
                speed = min(self.speed_max, speed*1.3) # boost on exit
        
        
        # post recovery steering so that we dont pick the bad gap again
        if self.post_recovery_counter > 0:
            steering_angle = np.clip(steering_angle +0.5 * self.post_recovery_bias, -self.max_steering, self.max_steering)
            self.post_recovery_counter -= 1

        # dashboard lidar view + predicted path — throttled, this is the
        # full scan window (not the per-message debug summary), no need
        # to send it at full scan rate
        self.lidar_view_counter += 1
        if self.lidar_view_counter >= self.lidar_view_every_n:
            self.lidar_view_counter = 0
            window_angle_min = data.angle_min + start * data.angle_increment
            view_msg = Float32MultiArray()
            view_msg.data = [
                float(window_angle_min),
                float(data.angle_increment),
                float(steering_angle),
                float(speed),
                float(self.wheelbase),
                float(self.max_lidar_range),
            ] + processed_ranges.tolist()
            self.lidar_view_pub.publish(view_msg)

        # TODO: pubolish drive message
        self.publish_drive(steering_angle, speed)
        
        # DEBUGGER
        # self.get_logger().info("<"+"-"*20+">", throttle_duration_sec=0.5)
        # self.get_logger().info(f'CURRENT SPEED: {speed}| TURNING ANGLE: {np.rad2deg(steering_angle)}', throttle_duration_sec=0.5)
        # self.get_logger().info(f'GAP: [{start_gap_index, end_gap_index}] | BEST: {best_index}', throttle_duration_sec=0.5)
        debugging_message = Float32MultiArray()
        debugging_message.data = [
            float(speed),
            float(self.actual_speed),
            float(np.rad2deg(steering_angle)),
            float(start_gap_index),
            float(end_gap_index),
            float(best_index),
        ]
                
        self.debug_pub.publish(debugging_message)


    def recovery_logic(self, steering_angle = np.deg2rad(30)):
        if self.stuck_counter > self.stuck_threshold and not self.is_recovering:
            self.get_logger().warning("WE ARE STUCK - locking the angle...")
            self.is_recovering = True
            self.recovery_counter = 0
            self.recovery_start_time = self.get_clock().now()

            # # mirror the last steering direction in reverse
            # self.recovery_steering = -self.prev_steering
            # if abs(self.recovery_steering) < 0.15:
            #     self.recovery_steering = self.max_steering
        
        # backup briefly then resume normal driving
        if self.is_recovering:
            elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
            
            reverse_speed = -0.5

            if elapsed < 1.0: # reverse straight
                self._publish_reverse(0.0, reverse_speed)
                self.get_logger().warn(f'reversing straight {self.current_speed} | {self.actual_speed}')
                self.get_logger().warn(
                    f"RECOVERY CMD: {np.rad2deg(steering_angle):.1f}"
                )
                return True
            elif elapsed < 3.0: # reverse straight
                self._publish_reverse(steering_angle, reverse_speed)
                self.get_logger().warn(f'reversing at angle {np.rad2deg(steering_angle)} | {self.current_speed} | {self.actual_speed}')
                self.get_logger().warn(
                    f"RECOVERY CMD: {np.rad2deg(steering_angle):.1f}"
                )
                return True
            else:
                self.is_recovering = False
                self.stuck_counter = -100
                self.post_recovery_bias = 0.0  # capture BEFORE zeroing
                self.post_recovery_counter = 40
                # self.recovery_steering = 0.0  # zero AFTER
                self.get_logger().warn(f'done recovering')
                return False
        return False
                
        
    def calc_bubble(self, data, smallest_dist, smallest_index, array_length):
        dynamic_radius = self.bubble_radius * (1.0 + self.current_speed / self.speed_max) # at full speed it is double the size
        #calc the angle opposite by bubble
        bubble_angle = dynamic_radius / max(smallest_dist,0.5) # prevents bubble from blowing up to infinity when somehting is next to you
        # number of indices 
        bubble_index_range = int(np.ceil(bubble_angle / data.angle_increment)) # get how many lidar beams that angle covers
        start_index = max(0, smallest_index - bubble_index_range)
        end_index = min(array_length - 1, smallest_index + bubble_index_range)
        return np.arange(start_index, end_index +1) # positions in lidar array that fall inside danger bubble
    
    def find_max_gap(self, data):
        # return the start and end index of the max gap
        # a gap is continuous sequence of non-zero values
        
        # find where free space is
        # create a mask to show the nonzero values
        mask = data > 0 
        
        # find each edge index of the gaps
        # set bounds of array
        edges = np.diff(np.concatenate(([0], mask.astype(int), [0])))

        # [0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0]
        # for np.diff it finds changes with 0->1 as 1 and 1->0 as -1
        # [0, 1, 0,-1, 1,-1, 1, 0, 0, -1]
        
        
        # we want the largest gap of non zero values so we check
        # check where edge starts and ends
        start = np.where(edges == 1)[0] # looking or vals of 1
        end = np.where(edges == -1)[0] # looking or vals of -1
        
        
        # make sure the start exists
        if len(start) == 0:
            return 0, 0
        # calc that gap
        gap_length = end - start # this is an array
        
        # keep the longest gap
        largest_gap_index = np.argmax(gap_length)
        
        return start[largest_gap_index], end[largest_gap_index] -1
    
    def max_val_in_gap(self, data, start_index, end_index):
        # find the max val between indices
        # for loop starting 
        if start_index >= end_index or end_index >= len(data):
            return len(data) // 2
        gap_ranges = data[start_index:end_index+1]
        
        gap_width = end_index-start_index
        gap_average_depth = np.mean(gap_ranges)
        gap_min_depth = np.min(gap_ranges)
        
        # plateau aware of furthest point 
        max_val = gap_ranges.max()
        plateau = np.where(gap_ranges >= max_val - 0.05)[0]
        furthest_index = start_index + int(plateau.mean())

        centered_index = (start_index + end_index) // 2
        
        
        # Normalize each factor to roughly [0, 1]
        width_score = gap_width / 200.0           # 200 beams = "wide"
        depth_score = gap_average_depth / self.max_lidar_range
        safety_score = gap_min_depth / self.max_lidar_range  # punishes gaps with close obstacles
        
        # aggression score chooses how to tune the individual weights
        aggression = 0.4 * width_score + 0.4 * depth_score + 0.2 * safety_score
        
        # alpha should depend on current speed
        speed_factor = 1.0 - (self.current_speed / self.speed_max) # slow == 1, fast == 0
        alpha = np.clip(aggression * 0.5 + speed_factor * 0.5, 0.0, 1.0)
        
        return int(alpha * furthest_index + (1 - alpha) * centered_index)
    
    def publish_drive(self, steering_angle, speed):
        self.current_speed = speed
        
        drive_msg = AckermannDriveStamped() # how to talk to the f1tenth car by giving steering angle and speed
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        
        self.drive_pub.publish(drive_msg)

    def _publish_reverse(self, steering_angle, speed):
        self.current_speed = speed

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)   # HARD ZERO — no exceptions
        drive_msg.drive.speed = float(speed)

        self.drive_pub.publish(drive_msg)

        # recovery_logic() early-returns out of lidar_callback before the
        # normal debug_pub.publish() at the bottom, so without this the
        # dashboard/monitor go dark for the whole reversing maneuver —
        # exactly when you most want to see what the car is doing.
        debugging_message = Float32MultiArray()
        debugging_message.data = [
            float(speed),
            float(self.actual_speed),
            float(np.rad2deg(steering_angle)),
            -1.0,
            -1.0,
            -1.0,
        ]
        self.debug_pub.publish(debugging_message)
        
def main(args=None):
    rclpy.init(args=args)
    
    print("initialized...")
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
                
if __name__ == "__main__":
    main()
        