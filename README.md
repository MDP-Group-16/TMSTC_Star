
# TMSTC-Star
## A Turn-minimizing Algorithm For Multi-robot Coverage Path Planning  

## Fork changes

Reworked version of the original TMSTC_Star package implementing it as a service that takes a map, inital robot positions and the cleaning tool width and returns a list of paths. Mostly identical to original results, however, the grid resizing is now done using the grid_map package and uses opencv. There also still seems to be some misalignement between the generated grid and the path for some reason.

I have also completely removes the turtlebot simulation since this was not neede here and should also be decoupled form TMSTC* in a demo package.

## Description
It partitions the map with a minimum number of bricks as spanning tree’s branches, greedily connects bricks to form a complete spanning tree and then applys the greedy strategy of MSTC* to find the optimal equilibrium division on the topological loops around the spanning tree, thus averaging the weights of each robot coverage path. In addition, it incorporates turn cost into the weights of the paths.


## 1. Build package on ROS
```
    cd ~/catkin_ws/src
    git clone https://github.com/MDP-Group-16/TMSTC-Star.git
    cd ../
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
(maybe you alsoe have to ```sudo apt-get install libopencv-dev``` before, not sure)

## 2. Run test visualisation
```
    roslaunch TMSTC_Star test.launch
```

## 4. Parameters Explanation (main.launch)
`/allocate_method`: "DARP" or "MSTC"

`/MST_shape`: "DFS_HORIZONTAL", "DFS_VERTICAL", "KRUSKAL", "ACO", "HEURISTIC", "RECT_DIV" or "DINIC"

`/coverAndReturn`: let robots return to its depot or not, "true" or "false" (currently only false works)

`/free_threshold`: at what point is a cell considered occpuied (ie will not be cleaned) after interpolation (0-100, 100 is occupied)
    
If sets "MSTC" and "DINIC" at the same time, the planner will perform TMSTC*. Change main.launch to add more robots. If you want to use anotheor map, remember to modify image path in `map/map.yaml` as well.

## 5. Original Demos
mCPP on various environments:

<img src="map/indoor_real.png" width = "350" height = "300" alt="indoor" /> 
<img src="map/random_20_10.png" width = "300" height = "300" alt="random" /> 
<img src="map/Real_world/Denver_2_1024.png" width = "300" height = "300" alt="Denver_2" /> 
<img src="map/Real_world/NewYork_0_1024.png" width = "300" height = "300" alt="NewYork_0" /> 

<img src="map/results/Indoor_real.png" width = "350" height = "300" alt="indoor_res" /> 
<img src="map/results/random_20_10_new.png" width = "300" height = "300" alt="random_res" /> 
<img src="map/results/Denver_2.png" width = "300" height = "300" alt="Denver_2_res" />     
<img src="map/results/NewYork_0.png" width = "300" height = "300" alt="NewYork_0_res" />


