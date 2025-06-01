# TurtleBot3 Obstacle Detection & Victim Rescue

**Authors:** Amalie Bjørnholt & Kathrine Madsen (Group 13)  
**Supervisors:** Jalil Boudjadar & Mirgita Frasheri  

---

## Overview

This repository contains the ROS 2-based implementation for a TurtleBot3-Burger robot that autonomously:

1. **Navigates** an obstacle course using a 360° LiDAR sensor.
2. **Detects “victims”** (red dots on the floor) with an ISL29125 RGB sensor.
3. **Counts collisions** and computes average linear speed.
4. **Signals detected victims** via an LED blink and logs results to the terminal.

The final output (printed after a 120 s run) includes:
- Total victims detected
- Average linear speed (m/s)
- Number of collisions

---

## Features

- **Cone-based Navigation**  
  • Segments the front 180° LiDAR scan into five “cones” (Left Outer, Left Inner, Mid, Right Inner, Right Outer).  
  • Applies three distance thresholds (Early-React 0.8 m, Safety 0.5 m, Stop 0.25 m) to select smooth, collision-avoiding velocity commands.

- **Collision Counter**  
  • Defines `FRONT_LIMIT = 0.12 m` and `SIDE_LIMIT = 0.10 m`.  
  • Uses a boolean flag (`in_collision`) to increment a counter only once per obstacle encounter.

- **Victim Detection & LED Signaling**  
  • Reads raw R, G, B values from an ISL29125 over I²C.  
  • Classifies a “victim” when `Red > Green` and `Red > Blue`.  
  • Uses a Boolean algorithm to avoid double-counting the same red marker.  
  • Blinks a GPIO‐driven LED for 2 s

- **Average Speed Calculation**  
  • In each 50 ms timer callback, accumulates `|linear_velocity| × Δt` and counts the number of updates.  
  • On shutdown (after 120 s), computes average linear speed as `(total_distance / total_time)`.

- **ROS 2 Publisher–Subscriber Architecture**  
  • **Subscribers:**  
    - `/scan` (LaserScan) → `scan_callback()`  
    - `/cmd_vel_raw` (Twist) → `cmd_vel_raw_callback()`  
  • **Publisher:** `/cmd_vel` (Twist) → publishes computed drive commands at 20 Hz (every 0.05 s).

**Hardware**:  
   - TurtleBot3 Burger base  
   - Raspberry Pi 3  
   - ISL29125 RGB sensor (I²C)  
   - 360° LDS-01 LiDAR (TurtleBot3’s standard LIDAR)  
   - LED wired to a free GPIO pin (configured as a digital output)  


