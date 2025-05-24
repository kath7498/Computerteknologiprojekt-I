pt 2
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

import smbus # For LED sensor.
import time

# ---- IMPORT AF RPI.GPIO TIL LED ----
import RPi.GPIO as GPIO # For LED porten.


##############################################################################
#                       N O D E  : Turtlebot3ObstacleDetection
##############################################################################
class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection')
        print('--------------------------------------------------')
        print('Stop distance: 0.25 m | Safety distance: 0.5 m | Early react distance: 0.8 m')
        print('Kollisionsgrænser: 12 cm front, 10 cm sider')
        print('Auto-stop efter 120 sekunder')
        print('--------------------------------------------------')

        # --- ROS-related ---
        self.scan_ranges = []
        self.has_scan_received = False
        self.stop_distance = 0.25
        self.safety_distance = 0.5
        self.early_react_distance = 0.8

        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0


        self.speed_accumulation = 0.0 
        self.speed_updates = 0 
        self.dt = 0.05
        self.collision_counter = 0 
        self.in_collision = False 

        # --- Victim-related ---
        self.victim_counter = 0 
        self.on_victim = False 
        self.last_victim_time = 0

        # Setup ROS-publishers and -subscribers
        qos = QoSProfile(depth=10) 

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        # Timer (0.05 s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Timer for RGB Sensor
        self.RGB_timer = self.create_timer(0.2, self.read_rgb_sensor)

        # Auto-stop after 120 seconds
        self.runtime = 120.0
        self.shutdown_timer = self.create_timer(self.runtime, self.shutdown_node)

        # --- LED-configuration (RPi GPIO) ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.GPIO_LED = 17
        GPIO.setup(self.GPIO_LED, GPIO.OUT)
        # Turn off LED in the start
        GPIO.output(self.GPIO_LED, False)

        # Setup RGB sensor.
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x44, 0x01, 0x05)
        time.sleep(1)

    # ********************************************************************
    #                    Implementation of "victim"
    # ********************************************************************
    
    def read_rgb_sensor(self):
        try:
            data = self.bus.read_i2c_block_data(0x44, 0x09, 6)  # Read RGB data
            green = data[1] + data[0] / 256
            red = data[3] + data[2] / 256
            blue = data[5] + data[4] / 256

            current_time = time.time()

            # Determine the color/ if victim is detected:
            if red > green and red > blue:
                if not self.on_victim or (current_time - self.last_victim_time > 5.0):
                    self.victim_counter += 1
                    self.on_victim = True
                    self.get_logger().info('Victim detected')
                    GPIO.output(self.GPIO_LED, True) # Turns on the LED.
                    time.sleep(2) # Led tændt i to sekunder.
                    GPIO.output(self.GPIO_LED, False) # Turns off the LED.
                    self.last_victim_time = current_time
            else:
                self.on_victim = False

        # In case we get a weird value
        except Exception as e:
            self.get_logger().error(f"Fejl i læsning af sensor {e}")

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

    # ********************************************************************
    #                    Scanning/avoidance-functions
    # ********************************************************************
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if not self.has_scan_received:
            return
        
        obstacle_distances = self.detect_obstacle()
        twist = self.avoid_obstacle(obstacle_distances)
        self.cmd_vel_pub.publish(twist)

        # Update average speed
        self.speed_accumulation += abs(twist.linear.x) * self.dt
        self.speed_updates += 1

    def detect_obstacle(self):
        zones = {
            "left_outer": list(range(54, 91)),
            "left_inner": list(range(18, 54)),
            "mid": list(range(342, 360)) + list(range(0, 18)),
            "right_inner": list(range(306, 342)),
            "right_outer": list(range(270, 306))
        }

        def get_valid_range(indices):
            valid = [self.scan_ranges[i] for i in indices if 0 < self.scan_ranges[i] < float('inf')]
            return min(valid) if valid else float('inf')

        obstacle_distances = {
            zone: get_valid_range(indices) for zone, indices in zones.items()
        }

        FRONT_LIMIT = 0.12
        SIDE_LIMIT = 0.10

        # Tjek for om der er tale om collision
        obstacle_detected = (
            obstacle_distances["mid"] <= FRONT_LIMIT 
            or any (
                obstacle_distances[z] <= SIDE_LIMIT
                for z in ("left_inner", "right_inner", "left_outer", "right_outer") 
            )
        )
        
        if obstacle_detected: 
            if not self.in_collision:
                self.in_collision = True
                self.collision_counter += 1 
        else:
            self.in_collision = False

        return obstacle_distances

    def avoid_obstacle(self, obstacle_distances):
        twist = Twist()

        # Tidlig blød drejning
        if (obstacle_distances["mid"] < self.early_react_distance and
            obstacle_distances["mid"] > self.safety_distance):

            if obstacle_distances["left_inner"] > obstacle_distances["right_inner"]:
                twist.linear.x = 0.18
                twist.angular.z = 0.1
            else:
                twist.linear.x = 0.18
                twist.angular.z = -0.1
            return twist

        # Forhindring meget tæt
        if obstacle_distances["mid"] < self.stop_distance:
            if obstacle_distances["left_inner"] > obstacle_distances["right_inner"]:
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            elif obstacle_distances["right_inner"] > obstacle_distances["left_inner"]:
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            else:
                twist.linear.x = -1.5
                twist.angular.z = 0.0

        # Reaktion i nærzonen
        elif (obstacle_distances["mid"] < self.safety_distance or
                obstacle_distances["left_inner"] < self.safety_distance or
                obstacle_distances["right_inner"] < self.safety_distance):
            if obstacle_distances["left_inner"] > obstacle_distances["right_inner"]:
                twist.linear.x = 0.1
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.1
                twist.angular.z = -0.5

        # Let drej baseret på åben sti
        elif obstacle_distances["left_outer"] > obstacle_distances["right_outer"]:
            twist.linear.x = 0.18
            twist.angular.z = 0.3
        elif obstacle_distances["right_outer"] > obstacle_distances["left_outer"]:
            twist.linear.x = 0.18
            twist.angular.z = -0.3
        else:
            twist.linear.x = self.tele_twist.linear.x
            twist.angular.z = self.tele_twist.angular.z

        return twist



    # ********************************************************************
    #                           Shutdown
    # ********************************************************************
    def shutdown_node(self):
        # Stop robotten
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

        total_time = self.speed_updates * self.dt      
        avg_speed  = (self.speed_accumulation / total_time) if total_time > 0 else 0.0

        print("\n--- SIMULATIONSRESULTAT ---")
        print(f"Gennemsnitlig lineær hastighed: {avg_speed:.3f} m/s")
        print(f"Antal registrerede kollisioner: {self.collision_counter}")
        print(f"Antal registrerede ofre (røde farver): {self.victim_counter}")

        # Frigør GPIO-ressourcer
        GPIO.cleanup()

        self.destroy_node()


##############################################################################
#                                   MAIN
##############################################################################
def main(args=None):
    rclpy.init(args=args)

    obstacle_node = Turtlebot3ObstacleDetection()

    executor = MultiThreadedExecutor()
    executor.add_node(obstacle_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


