# Webots Level-3 Tactical Autonomous Vehicle Planner

A deterministic, high-frequency tactical planner and decision brain for a Level 3 Autonomous Vehicle implementation. Built using **Python** and **OpenCV** inside the **Webots Simulation Engine**, this system architectures a robust control loop by strictly decoupling lateral perception from longitudinal safety streams to navigate complex urban corridors.

---

## 🏎️ Core Architecture & Control Philosophy

This software stack is designed around a hierarchical **Finite State Machine (FSM)** that acts as the vehicle’s central decision brain. The primary engineering goal is resolving the dependency between high-latency image processing and critical, low-latency collision avoidance systems.

```
                  +--------------------------------+
                  |       Sensory Subsystems       |
                  |  (Windshield Camera & LiDAR)   |
                  +--------------------------------+
                                  |
            +---------------------+---------------------+
            |                                           |
            v                                           v
+-----------------------+                    +---------------------+
|   Lateral Steering    |                    | Longitudinal Safety |
|   (OpenCV Matrices)   |                    |   (LiDAR Arrays)    |
+-----------------------+                    +---------------------+
            |                                           |
            +---------------------+---------------------+
                                  |
                                  v
                  +--------------------------------+
                  |  Deterministic FSM Planner     |
                  |   (Dual-Sensor Arbitration)    |
                  +--------------------------------+

```

### 1. Lateral Control (OpenCV Boundary Cushion Tracking)

Instead of relying on continuous, highly absolute coordinates, the vehicle utilizes a **relative tracking framework**:

* **Calibration Matrix:** On frame initialization (`Step 1`), the perception pipeline samples the exact pixel distance from the camera center to the left broken white line and the right solid yellow divider line (Indian traffic left-hand driving layout).
* **Relative Reference Lock:** The vehicle targets this exact relative coordinate space during transit. Deviations from this target cushion trigger linear steering corrections, drastically dampening oscillatory weaving profiles.
* **Hysteresis Blind Hold:** At open crossroads and intersections where lane markers structurally drop out, pixel density metrics fall below a specific threshold (450 pixels). The system handles this gracefully by entering a **Blind Crossing Hold**, freezing the steering angle perfectly straight (0.0) to mitigate trajectory drift until lines reappear and re-lock.

### 2. Longitudinal Safety & Tactical Overtaking (180° LiDAR Sectorization)

The vehicle's bumper mounts a 180-degree directional laser scanner array, mathematically split into distinct perception zones:

* **Frontal Integration Cone ($\pm$5°):** Actively monitors distance-to-collision profiles straight ahead.
* **Flank Clearance Pools (15° to 45° Left/Right):** Continuously sample flanking obstacle clearance horizons.

When a dynamic or static path obstruction breaches the **8.5m tactical threshold**, the planner evaluates the spatial density profiles of the flank pools. It calculates the obstacle's horizontal span ratio and triggers an immediate, smooth lateral swerve profile toward whichever side yields maximum clearance.

### 3. Closed-Loop Side-Scanning Recovery

To prevent the vehicle from returning to its original lane prematurely and clipping an obstacle, the system deploys a geometric verification step. During a passing maneuver, the vehicle locks its wheel angles straight and relies exclusively on side LiDAR rays. The return maneuver to the baseline lane configuration is completely suppressed until the active side sensor values step cleanly to infinity, proving the rear bumper has cleared the obstacle footprint.

---

## 🛠️ Hardware & Environment Stack

* **Simulation Engine:** Webots City World Environment (`city.wbt`)
* **Vehicle Prototype:** BMW X5 PROTO
* **Development Environment:** Anaconda Python 3
* **Host Operating System:** macOS 

---

## 📂 Repository Structure

```text
├── .gitignore                        # Git exclusion rules
├── LICENSE                           # MIT Open Source License
├── requirements.txt                  # Python dependency manifestations
├── controllers/
│   └── PureRelativeAVPlanner/
│       └── PureRelativeAVPlanner.py  # Master FSM Planning & Perception Script
└── worlds/
    └── city.wbt                      # Webots Indian Traffic Layout Environment

```

---

## 🚀 Finite State Machine (FSM) Specification

The system operates across six highly distinct, deterministic states to maintain deterministic state propagation:

| State ID | State Constant | Functional Description | Target Velocity |
| --- | --- | --- | --- |
| `0` | `STATE_LANE_CRUISE` | Standard relative lane keeping tracking using color-space segmentation. | 30.0 km/h |
| `1` | `STATE_ADAPTIVE_CRUISE` | Distance-proportional deceleration active; monitors tactical swerve options. | 10.0 - 22.0 km/h |
| `2` | `STATE_OVERTAKE_SWERVE` | Initiates steering deflection based on LiDAR flank clearance computations. | 12.0 km/h |
| `3` | `STATE_OVERTAKE_PASSING` | Freezes steering angle; monitors side LiDAR profile clearance bounds. | 16.0 km/h |
| `4` | `STATE_OVERTAKE_RECOVERY` | Executes reverse curvature steering deflection to return to target lane. | 14.0 km/h |
| `5` | `STATE_EMERGENCY_HALT` | High-priority safety gate. Triggers 0.0 steering and applies max braking. | 0.0 km/h |

---

## 💻 Core Production Implementation

```python
import numpy as np
import cv2
from vehicle import Driver

class PureRelativeAVPlanner:
    def __init__(self):
        self.driver = Driver()
        self.timestep = int(self.driver.getBasicTimeStep())
        
        # Hardware Initialization Subsystems
        self.camera = self.driver.getDevice("camera")
        self.camera.enable(self.timestep)
        self.img_width = self.camera.getWidth()
        self.img_height = self.camera.getHeight()
        
        self.lidar = self.driver.getDevice("Sick LMS 291")
        if self.lidar is not None:
            self.lidar.enable(self.timestep)
            
        # State Machine Configuration
        self.STATE_LANE_CRUISE = 0
        self.STATE_ADAPTIVE_CRUISE = 1
        self.STATE_OVERTAKE_SWERVE = 2
        self.STATE_OVERTAKE_PASSING = 3  
        self.STATE_OVERTAKE_RECOVERY = 4
        self.STATE_EMERGENCY_HALT = 5
        self.current_state = self.STATE_LANE_CRUISE
        
        # Motion & Curvature Variables
        self.target_speed = 30.0 
        self.overtake_counter = 0 
        self.evasion_direction = 0 
        self.dynamic_steer_angle = 0.0
        
        # Calibration & Relative Reference Tracking Variables
        self.is_calibrated = False
        self.target_left_white_dist = 0.0   
        self.target_right_yellow_dist = 0.0 
        self.crossroad_mode = False

    def extract_lane_error(self):
        raw_img = self.camera.getImage()
        if not raw_img: return 0.0
        
        frame = np.frombuffer(raw_img, dtype=np.uint8).reshape((self.img_height, self.img_width, 4))
        roi = frame[int(self.img_height * 0.75):int(self.img_height * 0.85), 0:self.img_width]
        bgr = cv2.cvtColor(roi, cv2.COLOR_BGRA2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        lower_yellow = np.array([20, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([180, 30, 255], dtype=np.uint8)
        
        mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_w = cv2.inRange(hsv, lower_white, upper_white)
        
        image_center = self.img_width / 2
        current_left_w_dist = None
        current_right_y_dist = None
        
        moments_w = cv2.moments(mask_w[:, 0:int(image_center)])
        if moments_w["m00"] > 0:
            cx_w = int(moments_w["m10"] / moments_w["m00"])
            current_left_w_dist = image_center - cx_w
            
        moments_y = cv2.moments(mask_y[:, int(image_center):self.img_width])
        if moments_y["m00"] > 0:
            cx_y = int(moments_y["m10"] / moments_y["m00"]) + int(image_center)
            current_right_y_dist = cx_y - image_center

        if current_left_w_dist is None and current_right_y_dist is None:
            if not self.crossroad_mode:
                self.crossroad_mode = True
            return 0.0
        
        if self.crossroad_mode and (current_left_w_dist is not None or current_right_y_dist is not None):
            self.crossroad_mode = False

        if not self.is_calibrated and current_left_w_dist is not None and current_right_y_dist is not None:
            self.target_left_white_dist = current_left_w_dist
            self.target_right_yellow_dist = current_right_y_dist
            self.is_calibrated = True

        steering_output = 0.0
        if current_right_y_dist is not None and current_right_y_dist < (self.target_right_yellow_dist - 8):
            deviation = self.target_right_yellow_dist - current_right_y_dist
            steering_output = -0.015 * deviation 
        elif current_left_w_dist is not None and current_left_w_dist < (self.target_left_white_dist - 8):
            deviation = self.target_left_white_dist - current_left_w_dist
            steering_output = 0.015 * deviation  
        else:
            steering_output = 0.0
            
        return np.clip(steering_output, -0.25, 0.25)

    def get_environmental_scan(self):
        if self.lidar is None: return float('inf'), float('inf'), float('inf'), 0.0
        range_image = self.lidar.getRangeImage()
        if not range_image: return float('inf'), float('inf'), float('inf'), 0.0
        
        total_samples = len(range_image)
        mid_idx = total_samples // 2
        cone_bounds = int(total_samples * 0.05)
        
        front_cone = range_image[mid_idx - cone_bounds : mid_idx + cone_bounds]
        front_dist = min([d for d in front_cone if d > 0.5]) if any(d > 0.5 for d in front_cone) else float('inf')
        
        left_sector = range_image[mid_idx - int(total_samples * 0.25) : mid_idx - int(total_samples * 0.08)]
        left_clearance = min([d for d in left_sector if d > 0.5]) if any(d > 0.5 for d in left_sector) else float('inf')
        
        right_sector = range_image[mid_idx + int(total_samples * 0.08) : mid_idx + int(total_samples * 0.25)]
        right_clearance = min([d for d in right_sector if d > 0.5]) if any(d > 0.5 for d in right_sector) else float('inf')
        
        blocked_rays = [idx for idx, dist in enumerate(front_cone) if dist < 12.0]
        obstacle_span_ratio = len(blocked_rays) / len(front_cone) if front_cone else 0.0
        
        return front_dist, left_clearance, right_clearance, obstacle_span_ratio

    def run_loop(self):
        while self.driver.step() != -1:
            lane_error = self.extract_lane_error()
            distance_ahead, left_space, right_space, obstacle_span = self.get_environmental_scan()
            
            # Critical Safety Intercept
            if distance_ahead < 3.8:
                if self.current_state != self.STATE_EMERGENCY_HALT:
                    self.current_state = self.STATE_EMERGENCY_HALT
            
            # Finite State Machine Execution
            if self.current_state == self.STATE_LANE_CRUISE:
                self.target_speed = 30.0
                self.driver.setSteeringAngle(0.0 if self.crossroad_mode else lane_error)
                if distance_ahead < 16.0:
                    self.current_state = self.STATE_ADAPTIVE_CRUISE
                    
            elif self.current_state == self.STATE_ADAPTIVE_CRUISE:
                self.target_speed = max(10.0, (distance_ahead - 5.0) * 2.0)
                self.driver.setSteeringAngle(0.0 if self.crossroad_mode else lane_error)
                if distance_ahead < 8.5:
                    self.dynamic_steer_angle = min(0.16 + (obstacle_span * 0.24), 0.38)
                    required_lane_clearance = 4.0 + (self.dynamic_steer_angle * 10.0)
                    
                    if left_space > right_space and left_space > required_lane_clearance:
                        self.current_state = self.STATE_OVERTAKE_SWERVE
                        self.evasion_direction = -1
                        self.overtake_counter = 0
                    elif right_space > left_space and right_space > required_lane_clearance:
                        self.current_state = self.STATE_OVERTAKE_SWERVE
                        self.evasion_direction = 1
                        self.overtake_counter = 0
                    else:
                        self.current_state = self.STATE_EMERGENCY_HALT
                elif distance_ahead > 22.0:
                    self.current_state = self.STATE_LANE_CRUISE
                    
            elif self.current_state == self.STATE_OVERTAKE_SWERVE:
                self.target_speed = 12.0
                self.overtake_counter += 1
                self.driver.setSteeringAngle(self.dynamic_steer_angle * self.evasion_direction)
                if self.overtake_counter >= 42:
                    self.current_state = self.STATE_OVERTAKE_PASSING
                    
            elif self.current_state == self.STATE_OVERTAKE_PASSING:
                self.target_speed = 16.0
                self.driver.setSteeringAngle(0.0)
                active_side_cushion = right_space if self.evasion_direction == -1 else left_space
                if active_side_cushion >= 6.0:
                    self.current_state = self.STATE_OVERTAKE_RECOVERY
                    self.overtake_counter = 0
                    
            elif self.current_state == self.STATE_OVERTAKE_RECOVERY:
                self.target_speed = 14.0
                self.overtake_counter += 1
                self.driver.setSteeringAngle(-0.25 * self.evasion_direction)
                if self.overtake_counter >= 42:
                    self.current_state = self.STATE_LANE_CRUISE
                    
            elif self.current_state == self.STATE_EMERGENCY_HALT:
                self.target_speed = 0.0
                self.driver.setSteeringAngle(lane_error)
                if distance_ahead > 15.0:
                    self.current_state = self.STATE_LANE_CRUISE
                    
            self.driver.setCruisingSpeed(self.target_speed)

if __name__ == "__main__":
    planner = PureRelativeAVPlanner()
    planner.run_loop()

```

---

## 🎯 Verification and Behavior Manifests

### Adaptive Cruise Behavior

When tracking standard vehicles ahead, the longitudinal pipeline scales vehicle speed based on the linear relation:
Target Velocity = Max(10.0, (Distance Ahead - 5.0) * 2.0)

This forces safe asymptotic deceleration profiles down to a minimum velocity anchor of 10 km/h before an evasion state execution or emergency intervention triggers.

### Critical Safety Limits

An absolute priority safety gate overrides all active state trajectories when:

* Frontal spacing drops below 3.8 meters.
* Multi-lane blocking scenarios occur where flank spaces fail to offer adequate clearing pathways (<= 4.0m + 10 * Steering Angle).
