## System Architecture

The self-balancing robot is built around a **Rock Pi 4b** and an **ESP32 Dev Module**, which communicate to ensure stable operation and motor control.  

- **Battery & Buck Converter**  
  The LiPo battery supplies power to the system, regulated by a buck converter to safe levels.  

- **Rock Pi 4b**  
  Runs higher-level tasks such as ROS, sensor fusion, and control algorithms.  

- **ESP32 Dev Module**  
  Handles real-time sensor readings from the IMU and encoders, and sends motor commands.  

- **Motor Drivers & Motors**  
  Receive commands from the ESP32 and drive the robot’s wheels.  

- **Sensors**  
  - **9-Axis IMU** provides orientation (yaw, pitch, roll), acceleration, and magnetic heading data.  
  - **Encoders** track wheel rotations for velocity and position feedback.   

Below is the system diagram showing how all the components connect:

![System Diagram](self_balancing_system_diagram.jpg)
