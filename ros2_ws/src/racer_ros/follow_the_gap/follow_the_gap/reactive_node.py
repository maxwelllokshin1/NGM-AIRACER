import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        
        # /////////// declaring the params \\\\\\\\\\
        self.declare_parameter('bubble_radius', 0.3) # in meters
        self.declare_parameter('preporcess_conv_size', 3) # the window to average values
        self.declare_parameter('max_lidar_range', 3.0) # meters
        self.declare_parameter('speed_min',0.5)
        self.declare_parameter('speed_max', 2.0)
        self.declare_parameter('steering_gain', 1.0) # proportional gain
        self.declare_parameter('max_steering', 30.0) # proportional gain
        
        # ////////// get parameters (just retrieving what was previously declared) \\\\\\\\\
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preporcess_conv_size = self.get_parameter('preporcess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.steering_gain = self.get_parameter('steering_gain').value
        self.current_speed = 0.0
        self.max_steering = np.deg2rad(self.get_parameter('max_steering').value)
        self.fov = np.deg2rad(70) # how much we want the car to see infront of it
        self.prev_steering = 0.0
        self.steering_smoothing = 0.6
        
        # /////////// topic names \\\\\\\\\\\\\\
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # /////// subs \\\\\\\
        # what the lidar produces
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10) # 10 for the refresh rate
        
        # /////// pubs \\\\\\\
        # the commands for driving (steering / speed)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10) # 10 is refresh rate
        self.debug_pub = self.create_publisher(Float32MultiArray, '/car_info_debug', 10)

        
        # ////////// DEBUGGER \\\\\\\\\
        self.get_logger().info("\/"*15)
        self.get_logger().info(f"Bubble: {self.bubble_radius}m")
        self.get_logger().info(f"MAX liDAR Range: {self.max_lidar_range}m")
        self.get_logger().info(f"min/max speed: {self.speed_min}m/s | {self.speed_max}m/s")
        self.get_logger().info(f"\/"*15)
        
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
        copied_ranges[bubble_indices] = 0.0 # this is to set all values within the bubble to 0. MAKE THEM KNOWN THEY ARE CLOSE
        
        
        # TODO: find max length gap
        
        start_gap_index, end_gap_index = self.find_max_gap(copied_ranges)
        if end_gap_index - start_gap_index < 35:
            self.get_logger().warn('NO GAP FOUND... REVERSING')
            left_half = processed_ranges[len(processed_ranges)//2:]
            right_half = processed_ranges[:len(processed_ranges)//2]
            
            left_space = np.sum(left_half)
            right_space = np.sum(right_half)
            
            recovery_steering = self.max_steering if left_space >right_space else -self.max_steering
            
            self.publish_drive(recovery_steering, -0.5) #slow down
            return
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
        alpha = 0.6 if abs(steering_angle) < 0.1 else 0.2 # smoothing on straights / non on corners
        steering_angle = alpha * self.prev_steering + (1 - alpha) * steering_angle
        self.prev_steering = steering_angle
        # TODO: calc speed based on steering angle and gap size
        
        # take anything infront of the car
        forward_beams = int(np.deg2rad(25) / data.angle_increment)
        mid = len(processed_ranges) // 2
        forward_slice = processed_ranges[mid - forward_beams : mid + forward_beams  ]
        min_forward = np.min(forward_slice[forward_slice > 0]) if np.any(forward_slice > 0) else self.max_lidar_range  # find the minimum nonzero value infront of car
        
        # steeper distance scaling, breaking distance grows with speed
        distance_factor = np.clip((min_forward / self.max_lidar_range) ** 1.5, 0.2, 1.0)
        # more steering -> slower speeds
        # PURE PURSUIT METHOD (speed = max * cos(angle)^2 * distance)
        
        speed = self.speed_max * (np.cos(steering_angle)**0.5) * distance_factor # we want to lower the speed based on how close the object infront of us is
        
        steering_delta = steering_angle - self.prev_steering
        if abs(steering_angle) > 0.2 and steering_delta * steering_angle > 0:
            speed *= 0.6 # entering a turn -> must break
        elif abs(steering_angle) < 0.15 and abs(self.prev_steering) > 0.3:
            speed = min(self.speed_max, speed*1.3) # boost on exit
        
        
        # TODO: pubolish drive message
        self.publish_drive(steering_angle, speed)
        
        # DEBUGGER
        # self.get_logger().info("<"+"-"*20+">", throttle_duration_sec=0.5)
        # self.get_logger().info(f'CURRENT SPEED: {speed}| TURNING ANGLE: {np.rad2deg(steering_angle)}', throttle_duration_sec=0.5)
        # self.get_logger().info(f'GAP: [{start_gap_index, end_gap_index}] | BEST: {best_index}', throttle_duration_sec=0.5)
        debugging_message = Float32MultiArray()
        debugging_message.data = [
            float(speed),
            float(np.rad2deg(steering_angle)),
            float(start_gap_index),
            float(end_gap_index),
            float(best_index),
        ]
                
        self.debug_pub.publish(debugging_message)
        
    def calc_bubble(self, data, smallest_dist, smallest_index, array_length):
        dynamic_radius = self.bubble_radius * (1.0 + self.current_speed / self.speed_max)
        #calc the angle opposite by bubble
        bubble_angle = dynamic_radius / max(smallest_dist,0.5)
        # number of indices 
        bubble_index_range = int(np.ceil(bubble_angle / data.angle_increment))
        start_index = max(0, smallest_index - bubble_index_range)
        end_index = min(array_length - 1, smallest_index + bubble_index_range)
        return np.arange(start_index, end_index +1)
    
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
        