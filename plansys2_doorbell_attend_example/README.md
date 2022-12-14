# PlanSys2 Doorbell Example

## Description

This example shows how to use sound classification (doorbell detection) using the YAMNet pretrained deep net and nav2 to answer the door.

## How to run

In terminal 1:

```
ros2 launch audio_detection audio_detection.launch.py
```

In terminal 2:

```
ros2 launch plansys2_doorbell_attend_example plansys2_simple_example_launch.py 
```

In terminal 3:

```
ros2 run plansys2_doorbell_attend_example doorcheck_controller_node  
```