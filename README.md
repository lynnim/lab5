## Usage
### Run

To launch turtlebot and map for part 1
```
roslaunch followbot launch.launch
```
```
python part1.py
```

To launch turtlebot and map for part 2
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To launch turtlebot and map for part 3
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```

```
python main.py [-s for single planner, -m for multi planner, -b for bonus] [step_size]

```
### Method
Part I:
- `image_callback()`: displays the map 

Part II:

Part III:



### Video
[Watch Part I, II, III Demo Here](https://youtu.be/H-SYxDvzKh4)

[



