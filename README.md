# AccioAssignment
## INFO
- Lidar package contain the LIDIAR bot with scripts to get the distance
- Here turtlebots were used as ROSbag was not provided 
    - So to give dynamic behavior to obstacles turtlebot3 were used
## clone this in src folder of your workspace and build package

## To launch LIDAR and dynamic obstacles
```
roslaunch lidar spawn.launch
```
## To view distace, start and end angle
```
rostopic echo /PublishObstaclesTopic 
```

<img src="./img.png" width=800/>