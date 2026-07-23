# launch — ADMM 三狗編隊（ROS2）

```bash
export FLEET_ARENA=plum     # 梅花樁
export FLEET_ARENA=door     # 門
export FLEET_ARENA=empty    # 空世界
```

**1. Gazebo**

```bash
ros2 launch legged_admm_fleet sim.launch.py
```

**2. 控制器**

```bash
ros2 launch legged_admm_fleet ctrl.launch.py
```

**3. ADMM**

```bash
ros2 launch legged_admm_fleet admm_fleet.launch.py
```

**4. 步態**

```bash
ros2 run legged_admm_fleet start_gaits.sh
```

**5. 下點**

```bash
ros2 topic pub -1 /formation/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 8.0, y: 0.0, z: 0.5}}}"
```

**6. RViz**

```bash
ros2 launch legged_admm_fleet viz.launch.py
```

記 log：`ros2 bag record -a -o <路徑>`

某狗沒站起來 → re-activate 那一隻：

```bash
ros2 service call /robotN/controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: [legged_robot_controller], strictness: 1, timeout: {sec: 10}}"
```
