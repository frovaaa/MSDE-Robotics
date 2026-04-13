# How to Build and Source the Environment

- Enter `frank` folder (pixi environment)
- Run `pixi run build-clean` to build the project
    - We made the custom run command to correctly source the python versions and the custom interface `MoveToGoal` which needed a different `buildtool_depend`
- Run `source install/setup.zsh` or different shell script to source the ros2 environment

# To Run the Environment

We made a launch file to run the turtlesim and the `usi_angry_turtle` controller
This will automatically spawn a turtle that will start to write "USI"

- Run `ros2 launch usi_angry_turtle turtle_writer_launch.py`

This starts the following nodes:
- `turtlesim`
- `usi_angry_turtle`
    - `move2goal_node` which is the ActionServer that receives the goals and moves the turtle accordingly
    - `writer_node` which is the ActionClient that sends the goals to the `move2goal_node` to write "USI"


To generate a new turtle
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"`

Then to control it via teleop in the terminal

`ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel`
