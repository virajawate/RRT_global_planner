# Robot_workspace


## Setting up the workspace

```
$ mkdir robo_ws // create workspace
$ cd robo_ws
$ mkdir src
$ cd src
$ git clone *Repo_Link*
$ cd ..
$ catkin_make 
$ source ./devel/setup.bash
```
## Running the environment

```
// Running default global planner
$ roslaunch robot_simulation sim_master.launch
// Running default global planner with dynamic window local planner
$ roslanch robot_simulation dwa_sim.launch
// Running RRT global planner
$ roslaunch robot_simulation rrt_sim.launch 
```

### Implementing different types of RRT planner

> You can change the planner by changing the planner.
> Change the parameter ***'planner_name'*** in ***'robot_simulation/launch/blocks/base_global_planner_rrt.yaml'*** file.
> The choices are already available in the file.