# Robot Football Simulation using Webots and ROS2

This project implements a basic robot football simulation using Webots and ROS2. It includes a robot capable of detecting and moving a ball, with goal detection and obstacle avoidance mechanisms.

## Requirements
- **ROS2 Jazzy Jalisco** (or another compatible ROS2 version)
- **Webots** simulation software
- **Ubuntu** machine or virtual machine (VM) running Linux

## Directory Overview

```The project structure is as follows:
.
├── launch
│   └── robot_launch.py
├── LICENSE
├── package.xml
├── README.md
├── resource
│   ├── my_robot.urdf
│   └── robot_football
├── robot_football
│   ├── ballSupervising.py
│   ├── __init__.py
│   ├── my_robot_driver.py
│   └── obstacle_avoider.py
├── setup.cfg
├── setup.py
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── worlds
    └── my_world.wbt
```


### Key Files and Folders

- **`launch/robot_launch.py`**: Contains the ROS2 launch file to initialize the simulation. It starts Webots and connects it to ROS2 nodes.

- **`resource/`**: Includes configuration files:
  - `my_robot.urdf`: Defines the robot's physical parameters (e.g., links, joints).
  - `robot_football`: Additional resources for robot configuration.

- **`robot_football/`**: Main Python package containing the core logic for:
  - `ballSupervising.py`: Supervises the ball's position and spawns it in the environment.
  - `my_robot_driver.py`: Handles robot motion and interactions with the environment.
  - `obstacle_avoider.py`: Implements basic obstacle avoidance behavior.
  - `__init__.py`: Marks this as a Python module.

- **`worlds/my_world.wbt`**: Defines the Webots world for the robot football simulation. This includes the robot, ball, and environmental objects.

- **`test/`**: Contains automated testing scripts to ensure coding standards and functionality:
  - `test_copyright.py`: Checks for proper copyright headers.
  - `test_flake8.py`: Validates PEP8 compliance.
  - `test_pep257.py`: Ensures proper documentation in code.

- **`setup.py`** and **`setup.cfg`**: Define the Python package setup for easy installation and ROS2 compatibility.

- **`package.xml`**: ROS2 package metadata (dependencies, author, license, etc.).

- **`LICENSE`**: Specifies the licensing terms for the project.

---

## How to Run the Simulation

Follow these steps to build and launch the simulation:

1. **Navigate to the workspace**  
   Make sure you are in the `ros2_ws` folder where the workspace is set up.

2. **Build the package**  
   Run the following command to build the `robot_football` package:
   ```bash
   colcon build --packages-select robot_football

3. **Source the environment**
   Source the setup file to ensure the environment variables are set:
   ```source install/local_setup.bash```

4. **Launch the simulation**
   Execute the launch file to start the simulation:
   ```ros2 launch robot_football robot_launch.py```

---

## Simulation Details

- **Ball Supervision**:  
  The ball is dynamically spawned in the simulation environment. `ballSupervising.py` ensures the ball is placed in appropriate positions for gameplay.

- **Robot Behavior**:  
  - `my_robot_driver.py`: Directs the robot to interact with the ball and navigate the field.  
  - `obstacle_avoider.py`: Prevents collisions with obstacles during gameplay.

- **Goal Detection**:  
  The simulation includes logic to detect when the ball is scored into a goal, implemented in the `ballSupervising.py`.

---

## Testing

To validate the package's compliance and functionality, run the automated tests:

```bash
colcon test```

