# Indoor-mobile-robot-with-arms

## Usage
```bash
git clone https://github.com/GetOverMassif/faboRobotBehavior.git
cd faboRobotBehavior
bash build.sh
source devel/setup.bash
```


To control robot arms to do designed actions:
```bash
roslaunch arm_control arm_control.launch
```

To test behavior_module only:
```bash
rosrun behavior_module behavior_node
```


To test behavior_module and perform_module together, controling fabo by bluetooth:
```bash
roslaunch behavior_module bluetooth_demo.launch
```

Use the following to **publish different topics more easily**:
```bash
rosrun behavior_module talker.py
```
