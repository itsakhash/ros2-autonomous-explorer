# autonomous_explorer

A ROS 2 package that makes a simulated TurtleBot3 map an unknown room by itself. It runs SLAM Toolbox and Nav2 in Gazebo and adds a custom **frontier-exploration node** that decides where to drive next. One launch command starts everything.

**Stack:** ROS 2 Jazzy, Gazebo Harmonic, SLAM Toolbox, Nav2 (MPPI controller), TurtleBot3 Burger, Python.

## Results

Measured on the TurtleBot3 hexagon world (`turtlebot3_world`), starting from an empty map each time, with the final version of the code:

| Run | Time (simulation) | Goals | Failed | Mapped area |
|---|---|---|---|---|
| 1 | 278 s | 10 | 0 | 21.8 m² |
| 2 | 181 s | 10 | 0 | 21.8 m² |
| 3 | 282 s | 9 | 0 | 21.7 m² |

Three runs, no human input after launch. The explorer finished each time by itself, once no frontiers were left. For reference, a map driven by hand with Nav2 click-goals covered 21.4 m² (`maps/arena.pgm`); the explorer's map from an earlier run is in `maps/explored.pgm`. Times are simulation time, not wall-clock time.

## Run it

Requirements: Ubuntu 24.04 with ROS 2 Jazzy, plus the Nav2, SLAM Toolbox and TurtleBot3 Gazebo packages.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repo's URL> autonomous_explorer
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select autonomous_explorer --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch autonomous_explorer explore.launch.py
```

This opens Gazebo and RViz, starts SLAM and Nav2, and starts the explorer after a 25 s warm-up. Watch the map grow in RViz; the explorer prints `EXPLORATION COMPLETE` with a summary and exits when it is done.

Launch arguments: `explore:=false` (bring up the stack without the explorer, for driving it yourself with Nav2 goals), `rviz:=false`, `x_pose` / `y_pose` (spawn point, default `-2.0 -0.5`), `explore_delay` (seconds before the explorer starts).

## How it works

```
Gazebo (TurtleBot3) --/scan,/odom--> SLAM Toolbox --/map--> frontier_explorer
        ^                                                        |
        |                                                  NavigateToPose
   /cmd_vel (TwistStamped)                                       v
        ^                                                       Nav2
   twist_relay <------------ /cmd_vel_nav2 (Twist) --------------+
```

**Frontier explorer** (`frontier_explorer.py`, with the pure-Python geometry in `frontier.py`). Once a second it reads the SLAM map and:

1. Finds **frontiers**: free cells with an unknown neighbour, grouped into 8-connected clusters. Clusters below a minimum size are ignored.
2. Picks the nearest usable cluster (not too close to the robot, not near a previously failed goal) and sends it to Nav2 as a `NavigateToPose` goal.
3. Blacklists goals that are rejected, fail, or time out.
4. Waits a few seconds after each goal so SLAM can publish its update. If the next frontier is still next to a place the robot has already reached, it is treated as stale and skipped rather than revisited.
5. Stops when no usable frontier remains for several checks in a row, then logs time, goals and mapped area.
6. If the map is tiny at startup with no usable frontier (robot boxed in), it drives to a nearby known-free cell to widen its view. A 360° lidar learns nothing by turning in place, so it has to move.

**Twist relay** (`twist_relay.py`). Nav2 in Jazzy publishes `geometry_msgs/Twist`, but the TurtleBot3 Gazebo bridge expects `TwistStamped` on `/cmd_vel`. `config/nav2_params.yaml` sends Nav2's output to `cmd_vel_nav2`, and the relay republishes it as `TwistStamped`.

**SLAM Toolbox tuning** (`config/slam_params.yaml`). By default SLAM Toolbox only accepts a new scan after the robot has moved 0.5 m or turned 0.5 rad. Exploration goals can be closer than that, so the map would never grow. The file lowers `minimum_travel_distance` and `minimum_travel_heading` to 0.1 and `map_update_interval` to 2 s.

## Tests

```bash
cd ~/ros2_ws && colcon test --packages-select autonomous_explorer && colcon test-result
```

Unit tests cover frontier detection and clustering, goal selection (distance filter, blacklist) and startup seed-cell selection. They use small synthetic grids, so they run without a simulator. The frontier tests were checked by deliberately breaking the code and confirming they fail.

## Limitations

- **Simulation only, one world.** Nothing here has run on a real robot, and results come from a single small hexagon arena with static pillars. There are no moving obstacles.
- **Three runs, one machine.** The results table is not a statistical benchmark. Timing varies by about 100 s between runs.
- **The startup seed move is unit-tested but has not triggered in a simulation run.** An attempted boxed-in start test spawned the robot inside a pillar and was invalid.
- **Nearest-frontier greedy strategy.** It does not plan a tour or weigh information gain, so paths in larger or more cluttered maps would not be efficient.
- **Coverage is compared against one hand-driven map**, not against ground truth from the Gazebo world.
- Developed on Windows 11 with WSL2 and WSLg. On that setup Gazebo's window can fail to start after a previous run ends, which `wsl --shutdown` fixes.

## Layout

```
autonomous_explorer/   frontier.py, frontier_explorer.py, twist_relay.py
launch/                explore.launch.py
config/                nav2_params.yaml, slam_params.yaml
maps/                  arena (hand-driven reference), explored (explorer output)
test/                  unit tests
```

## License

MIT