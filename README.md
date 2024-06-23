# stage_challenge

Assignment for Robotics class.

The node can be found inside the stage_challenge folder.

To run the code you'll need to:

Access the [stage_ros2](https://github.com/tuw-robotics/stage_ros2) github and install it.
Run:

```bash
ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
```

And in another terminal:

```bash
ros2 run stage_challenge challenge_node.py
```
