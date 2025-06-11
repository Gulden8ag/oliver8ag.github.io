common cmds:



```bash
ros2 launch robot_mov rsp.launch.py
```

```bash
rviz2
```

```bash
source install/setup.bash
```

```bash
colcon build --symlink-install
```

```bash
ros2 launch my_bot rsp.launch.py use_sim_time:=true
```

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```
spawn bot
```bash
ros2 run ros_gz_sim create -name robot_name -topic robot_description -entity bot_name
```