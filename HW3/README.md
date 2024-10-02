
# Humanoid Robot Following and Maze Navigation in ROS

## Project Overview
This project involves simulating the behavior of a mobile robot using the Robot Operating System (ROS), camera sensors, control algorithms, and image processing techniques. The project is divided into two main tasks:

1. **Humanoid Robot Following**: A TurtleBot3 Burger robot must follow a humanoid robot controlled by a user via keyboard.
2. **Maze Navigation**: The Burger robot must autonomously navigate through a maze by following walls to find an exit.

---

## Task 1: Humanoid Robot Following

### Description
In this simulation, the environment consists of two robots:
- A **humanoid robot**, controlled by the user through the keyboard using the `turtlebot3_teleop` package.
- A **Burger robot**, which must follow the humanoid robot autonomously using image processing and a control algorithm.

### Components:
1. **ImageProcessor Node**:
   - Subscribes to the robot's camera topic and processes the images to detect the target humanoid.
   - Responsibilities include:
     - Subscribing to the image topic.
     - Setting up the detection service for object recognition.
     - Displaying the robot’s camera view.

2. **Controller Node**:
   - Controls the movement of the Burger robot using a simple P-Controller.
   - Responsibilities include:
     - Using data from the ImageProcessor node to adjust the robot’s linear and angular speeds.
     - Moving the robot towards the humanoid target.
     - Adjusting the robot’s behavior if the target is lost (e.g., stop, rotate, or continue movement).

### Detection Service:
- This service takes a label (e.g., "person") as input and returns detection data such as bounding box coordinates and object dimensions.
- The **ImageProcessor** node acts as the server for this service, while the **Controller** node utilizes the detection data to guide the robot.

---

## Task 2: Maze Navigation

### Description
The robot is placed inside a maze and must navigate through it by following the walls until it finds an exit.

### Components:
1. **Wall-Following Algorithm**:
   - The robot follows the wall using a right-hand rule to navigate through the maze.
   - The algorithm adjusts the robot’s position and orientation to ensure it remains close to the wall.

2. **Maze Environment**:
   - The simulation environment (`world.maze`) is a predefined maze with the robot starting at coordinates (-0.5, 0).
   - The robot explores the maze, finds the exit, and navigates out.

### Outcome:
- The robot successfully exits the maze by following the walls.
- A path visualized in **Rviz** showing the robot’s navigation through the maze, with an image included in the report.

---

## Video Demonstration
[Watch the video](https://github.com/user-attachments/assets/78ab2047-2c76-4a35-97f5-f00f4b9e5453)


