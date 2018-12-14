## Usage
### Run

To launch turtlebot and map for part 1
```
roslaunch followbot launch.launch
```

To run the python script for part 1

```
python part1.py
```

To launch turtlebot and map for part 2
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=color.world
```

To run the python script for part 2

```
python part2.py
```

To launch turtlebot and map for part 3
```
ROBOT_INITIAL_POSE="-x -2.85 -y -0.27 -Y 1.53" roslaunch followbot launch.launch world_file:=shape.world
```
To run the python script for part 3

```
python part3.py

```

### Method
Part III:
-`image_callback()`: performs template matching to detech each shape. The robot follows the yellow track when not at an intersection. The robot turns left when the left arrow image is detected. The robot turns right when the right arrow image is detected. The robot stops when the star image is detected. 
### Video
[Watch Part I, II, III Demo Here](https://www.youtube.com/watch?v=cQQ3Jp61-Gk&feature=youtu.be)
