# How to Build and Source the Environment

- Enter the workspace directory (the one containing .pixi, pixi.toml and pixi.lock)
- In this specific case, it is needed to use the pixi.toml that we provided in the zip, as it contains the following commands to build correctly the two packages. There is a second package `action_usi_angry_turtle_interfaces` that contains the custom action interface for the `MoveToGoal` action server.

After copying the pixi.toml, run the following commands:

- Run `pixi run build-clean` to build the project
  - We made the custom run command to correctly source the python versions and the custom interface `MoveToGoal` which needed a different `buildtool_depend`
- Run `source install/setup.zsh` or different shell script to source the ros2 environment

# To Run the Application

We made a launch file to run the turtlesim and the `usi_angry_turtle` controller
This will automatically spawn a turtle that will start to write "USI"

- Run `ros2 launch usi_angry_turtle turtle_writer_launch.py`

This starts the following nodes:

- `turtlesim`
- `usi_angry_turtle`
  - `move2goal_node` which is the ActionServer that receives the goals and moves the turtle accordingly
  - `writer_node` which is the ActionClient that sends the goals to the `move2goal_node` to write "USI", and handles the state machine to follow, intercept and kill the second turtle

To generate a new turtle in a new terminal (after sourcing again the ros2 environment) run the following command:
`ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"`

It is important to spawn the turtle with the name **turtle2** as the `usi_angry_turtle` controller is coded to follow and kill the turtle with that name.

Then to control it via teleop in the terminal, run the following command:
`ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel`

If for any reason the zip is not working, there is the correct/complete environment available in the following git repository:
[https://github.com/frovaaa/MSDE-Robotics](https://github.com/frovaaa/MSDE-Robotics)
