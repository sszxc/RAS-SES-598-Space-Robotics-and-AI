# Cart-Pole Optimal Control Assignment

[Watch the demo video](https://drive.google.com/file/d/1UEo88tqG-vV_pkRSoBF_-FWAlsZOLoIb/view?usp=sharing)
![image](https://github.com/user-attachments/assets/c8591475-3676-4cdf-8b4a-6539e5a2325f)

## Overview
This assignment challenges students to tune and analyze an LQR controller for a cart-pole system subject to earthquake disturbances. The goal is to maintain the pole's stability while keeping the cart within its physical constraints under external perturbations. The earthquake force generator in this assignment introduces students to simulating and controlling systems under seismic disturbances, which connects to the Virtual Shake Robot covered later in the course. The skills developed here in handling dynamic disturbances and maintaining system stability will be useful for optimal control of space robots, such as Lunar landers or orbital debris removal robots.

## System Description
The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

## Assignment Objectives

### Core Requirements
1. Analyze and tune the provided LQR controller to:
   - Maintain the pendulum in an upright position
   - Keep the cart within its ±2.5m physical limits
   - Achieve stable operation under earthquake disturbances
2. Document your LQR tuning approach:
   - Analysis of the existing Q and R matrices
   - Justification for any tuning changes made
   - Analysis of performance trade-offs
   - Experimental results and observations
3. Analyze system performance:
   - Duration of stable operation
   - Maximum cart displacement
   - Pendulum angle deviation
   - Control effort analysis

### Learning Outcomes
- Understanding of LQR control parameters and their effects
- Experience with competing control objectives
- Analysis of system behavior under disturbances
- Practical experience with ROS2 and Gazebo simulation

### Extra Credit Options
Students can implement reinforcement learning for extra credit (up to 30 points):

1. Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR

## Implementation

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]
  ```

## Getting Started

### Prerequisites
- ROS2 Humble or Jazzy
- Gazebo Garden
- Python 3.8+
- Required Python packages: numpy, scipy

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=humble

# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# Install Python dependencies
pip3 install numpy scipy control
```

### Repository Setup

#### If you already have a fork of the course repository:
```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

#### If you don't have a fork yet:
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace
```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/cart_pole_optimal_control .
```

### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```

This will start:
- Gazebo simulation (headless mode)
- RViz visualization showing:
  * Cart-pole system
  * Force arrows (control and disturbance forces)
  * TF frames for system state
- LQR controller
- Earthquake force generator
- Force visualizer

### Visualization Features
The RViz view provides a side perspective of the cart-pole system with:

#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)

2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)

Arrow lengths are proportional to force magnitudes.

## Analysis Requirements

### Performance Metrics
Students should analyze:
1. Stability Metrics:
   - Maximum pole angle deviation
   - RMS cart position error
   - Peak control force used
   - Recovery time after disturbances

2. System Constraints:
   - Cart position limit: ±2.5m
   - Control rate: 50Hz
   - Pole angle stability
   - Control effort efficiency

### Analysis Guidelines
1. Baseline Performance:
   - Document system behavior with default parameters
   - Identify key performance bottlenecks
   - Analyze disturbance effects

2. Parameter Effects:
   - Analyze how Q matrix weights affect different states
   - Study R value's impact on control aggressiveness
   - Document trade-offs between objectives

3. Disturbance Response:
   - Characterize system response to different disturbance frequencies
   - Analyze recovery behavior
   - Study control effort distribution

## Evaluation Criteria
### Core Assignment (100 points)
1. Analysis Quality (40 points)
   - Depth of parameter analysis
   - Quality of performance metrics
   - Understanding of system behavior

2. Performance Results (30 points)
   - Stability under disturbances
   - Constraint satisfaction
   - Control efficiency

3. Documentation (30 points)
   - Clear analysis presentation
   - Quality of data and plots
   - Thoroughness of discussion

### Extra Credit (up to 30 points)
- Reinforcement Learning Implementation (30 points)

## Tips for Success
1. Start with understanding the existing controller behavior
2. Document baseline performance thoroughly
3. Make systematic parameter adjustments
4. Keep detailed records of all tests
5. Focus on understanding trade-offs
6. Use visualizations effectively

## Submission Requirements
1. Technical report including:
   - Analysis of controller behavior
   - Performance data and plots
   - Discussion of findings
2. Video demonstration of system performance
3. Any additional analysis tools or visualizations created

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 


# Assignment Submission

Xuechao Zhang, Feb. 12th

## Parameter Analysis and Tuning Process

Analysis of the Q and R matrices for the LQR controller:

The Q matrix represents the final optimization objective, so adjusting the weight of each component affects the system's sensitivity to the corresponding values. Below is an analysis of how each component of the Q matrix affects system performance.

### Q0

Q0 corresponds to cart position. Increasing Q0 keeps the cart closer to the origin, while decreasing it allows the cart to more easily reach the boundaries, potentially causing the pole to fall (as shown in the figure, Q0 = 0.01).

<img src="resource/Q0-0.01.png" width="400"/>

### Q1

Q1 corresponds to cart velocity. Increasing Q1 makes the cart tend to remain stationary (as shown in the figure, Q1 = 100), while decreasing it leads to more active cart movement.

<img src="resource/Q1-100.png" width="400"/>

### Q2

Q2 corresponds to pole angle. Increasing Q2 maintains the pole angle more stable at small angles (as shown in the figure, Q2 = 500), while decreasing it may cause the pole to fall.

<img src="resource/Q2-500.png" width="400"/>

### Q3

Q3 corresponds to pole angular velocity. Decreasing Q3 makes the pole more likely to fall (as shown in the figure, Q3 = 0.1).

<img src="resource/Q3-0.1.png" width="400"/>

### R

The R matrix represents the weight of control input. Increasing R makes control input more costly, resulting in smaller control inputs that may lead to pole falling; decreasing R leads to more aggressive control inputs. The figures below show cases where R is 0.001 and 10 respectively.

<img src="resource/R-0.001.png" width="400"/>
<img src="resource/R-10.png" width="400"/>


### Final Parameter

It's challenging to determine a final Q and R matrix for this task, as different task objectives have different requirements for performance metrics. However, it's clear that LQR is a very simple and effective controller for this task.

## Challenges and solutions
1. Default install command is not working, the Gazebo version is not compatible with the current code. The situation is that I cannot get anything in '/world/empty/model/cart_pole/joint_state' topic from the Gazebo /bridge. Finally I found that I have to change the Gazebo version to Garden to make it work.

2. The code framework is a bit complex, I need to spend some time to understand the structure. Thanks to the `rqt_graph` tool, I can have a clear view of the node structure and topic communication.
   ![rqt_graph](resource/ros_graph.jpg)

3. The control performance is influenced by the random seed of the disturbance generator. The simulation has to be run multiple times for analysis.

4. Currently, the control force is not limited by the physical limit of the cart-pole system(including the amplitude and the frequency). Simply increase the Q matrix will make the cart stable for quite a long time. But when considering the real system, the parameter tuning will be more complex.
