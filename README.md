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