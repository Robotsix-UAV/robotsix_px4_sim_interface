# robotsix_px4_sim_interface

ROS2 interface package for PX4 simulation with Gazebo. This package provides action interfaces for 
starting and stopping PX4-based simulations, along with convenient command-line tools.

**Important**: This package defines action interfaces that require an action server to be running, 
like the one provided in the `robotsix_px4_simulation` package. You can interact with these actions 
in three ways:
1. Through a custom ROS2 client node you create
2. Using the ROS2 CLI (`ros2 action send_goal`)
3. Using the convenience scripts provided in this package

For the first two options, please refer to the ROS2 documentation on action clients and servers.


## Installation

```bash
# Clone the repository into your ROS workspace src folder
cd ~/ros2_ws/src
git clone <repository_url>

# Build the package
cd ~/ros2_ws
colcon build --packages-select robotsix_px4_sim_interface

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Available Actions

This package defines the following ROS2 action interfaces:

#### StartSimulation

Used to start a PX4 simulation with a specified world and optional models.

**Goal Fields:**
- `string world`: Path to the world file (absolute or relative to GZ_SIM_RESOURCE_PATH)
- `ModelInfo[] models`: Array of models to spawn in the simulation

**Result Fields:**
- `string message`: Status message indicating success or failure

**Example usage with CLI:**
```bash
ros2 action send_goal /start_simulation robotsix_px4_sim_interface/action/StartSimulation \
  "{world: 'robotsix_px4_simulation/worlds/default.sdf', \
  models: [{model_dir: 'robotsix_px4_simulation/models/quad_gps', \
  model_name: 'uav0', px4_parameters: 6661, x: 0.0, y: 0.0, z: 0.0}]}"
```

#### StopSimulation

Used to stop a running PX4 simulation.

**Goal Fields:**
- (empty)

**Result Fields:**
- `string message`: Status message indicating success or failure

**Example usage with CLI:**
```bash
ros2 action send_goal /stop_simulation robotsix_px4_sim_interface/action/StopSimulation "{}"
```

#### ModelInfo Message

The `ModelInfo` message is used to define models to be loaded into the simulation.

**Fields:**
- `string model_dir`: Path to model directory (relative to GZ_SIM_RESOURCE_PATH)
- `string model_name`: Unique name for the model in gazebo
- `int32 px4_parameters`: SYS_AUTOSTART parameter number for PX4 (defines vehicle type)
- `float64 x`: X position in meters
- `float64 y`: Y position in meters
- `float64 z`: Z position in meters

### Using Convenience Scripts

After installation, you can use the provided tools with `ros2 run`:

#### Starting a Simulation

```bash
# Start with default settings
ros2 run robotsix_px4_sim_interface start_simulation.py

# Start with custom world
ros2 run robotsix_px4_sim_interface start_simulation.py --world my_world.sdf

# Start with custom model
ros2 run robotsix_px4_sim_interface start_simulation.py --model "model_dir" "model_name" \
  "px4_param" "x_pos" "y_pos"
```

#### Stopping a Simulation

```bash
ros2 run robotsix_px4_sim_interface stop_simulation.py
```

