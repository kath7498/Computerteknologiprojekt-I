#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from turtlebot3_example.turtlebot3_obstacle_detection.turtlebot3_obstacle_detection import (
    Turtlebot3ObstacleDetection,
)

def main(args=None):
    rclpy.init(args=args)

    obs = Turtlebot3ObstacleDetection()

    executor = MultiThreadedExecutor()
    executor.add_node(obs)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        obs.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()