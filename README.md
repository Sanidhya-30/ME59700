# ME597 sim_ws: Autonomous Mobile Robot for Mapping and Navigation

This repository contains the source code for an autonomous mobile robot project as part of course(**ME59700**) Autonomous Systems during my master's at Purdue University. This project demonstrates a complete autonomy pipeline, including **perception**, **planning**, and **control**. Applying robotics stack like ROS2, Gazebo, RViz, and exposing to concepts of A*, RRT, localisation, SLAM, mapping and PID controllers


### Media
A full video demonstration of the robot's capabilities is available on YouTube:  
[Video](https://youtu.be/xAcPW1oGTrY?si=4IjDck7RzmC39ZkU)

See the full presentation as part of submission for the course of Atonomous Systems:  
[Presentation](https://docs.google.com/presentation/d/1pr9O_q3-NSk569cJgl1d-uYNKOifBShK5hFOpCh8j8A/edit?usp=sharing) |
[Google Drive link](https://drive.google.com/drive/folders/1JCaMRaihruAj6XTVthfVRJGM5DfYzZwt?usp=sharing)


##
## Project Overview

The primary goal of this project was to design and validate a **complete robotics stack** for autonomous operation. This includes:

- **Mapping:** Autonomously exploring and mapping an unknown indoor environment using a wall-following algorithm.  
- **Path Planning:** Integrating **A\*** and **RRT** algorithms to find optimal paths to a given goal within the generated map.  
- **Visual Servoing:** Developing a visual tracking and following system for dynamic targets using color masking and **PID control**.  

The entire pipeline was successfully validated, showcasing **robust perception**, **fast planning**, and **responsive control**.



## Core Features & Performance

### Autonomous Mapping
- The robot is capable of autonomously mapping indoor environments using **LiDAR** and **IMU** data.  
- It employs a wall-following strategy to explore and complete maps in **under 4 minutes**.

### Goal-Directed Navigation
- Integrated **A\*** and **RRT** path planning algorithms enable goal-directed navigation.  
- Path computation and execution are highly efficient, with **execution times between 3–4 seconds**.

### Dynamic Target Tracking
- A visual tracking and following system uses **HSV color masking (OpenCV)** to identify and follow a dynamic target.  
- A low-latency **PID controller** ensures smooth and responsive following behavior.


## Instructions:
1. Simply save this workspace e.g., 
    ```
    cd ~/ros2 # cd into the dir you want to keep this workspace in
    git clone https://github.com/naslab-projects/sim_ws.git
    ```

2. In a new terminal build the sim_ws workspace: 
    ```
    cd sim_ws
    colcon build --symlink-install
    ```

3. Add turtlebot3 environment variable to .bashrc file
    ```
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
    ```
4. Run these to install useful packages you will use in the simulator.
    ```
    sudo apt install ros-humble-turtlebot3-teleop
    sudo apt install ros-humble-slam-toolbox
    sudo apt install ros-humble-navigation2
    ```
    
    ```
    pip install pynput
    ```

5. Don't forget to source the workspace whenever you use it
    ```
    cd sim_ws
    source install/local_setup.bash
    ```
