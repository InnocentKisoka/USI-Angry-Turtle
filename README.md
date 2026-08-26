# USI Angry Turtle

ROS 2 **turtlesim** project: a turtle writes **“USI”** while getting angry at (and hunting) other turtles that get in the way.

States: **writing → angry → returning → final**.

## Behaviour

- Draws the letters **USI** with the pen on/off along a scripted path
- Spawns distractor turtles
- When disturbed, switches to an angry pursuit mode, then returns to writing
- Uses turtlesim services: `spawn`, `kill`, `set_pen`, `clear`

## Package layout

```
usi_angry_turtle/     controller_node.py
package.xml / setup.py
test/
```

## Node

| Entry point | Module |
|---|---|
| `controller_node` | `usi_angry_turtle.controller_node:main` |

## Requirements

- ROS 2 (rclpy)
- `turtlesim`
- `geometry_msgs`

## Run

```bash
# Terminal 1
ros2 run turtlesim turtlesim_node

# Terminal 2 (from a workspace that contains this package)
colcon build --packages-select usi_angry_turtle
source install/setup.zsh
ros2 run usi_angry_turtle controller_node
```


