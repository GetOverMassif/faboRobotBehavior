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

To test BehaviorModule only:
```bash
rosrun BehaviorModule behavior_node
```

To test BehaviorModule and PerformModule together, controling fabo by bluetooth:
```bash
roslaunch BehaviorModule bluetooth_demo.launch
```

Use the following to **publish different topics more easily**:
```bash
rosrun BehaviorModule talker.py
```
