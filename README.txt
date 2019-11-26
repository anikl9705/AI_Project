# Heirarchial Planning for Vehicle Routing Problem (VRP)

Cluster-First, Route Second approach to VRP
(Refer: http://neo.lcc.uma.es/vrp/solution-methods/heuristics/cluster-first-route-second-method/)

## Setup
1. Copy house_1 folder to gazebo models folder (`~/.gazebo/models`).
2. Run `./env_setup.sh`.

## Running the simulation

1. First start ros.
```
$ roscore
```

2. Start the server.
```
$ rosrun group_4 server.py -h
usage: server.py [-h] [-t 5] [-p 5] [-d 5] [-g 5] [-s 32]

optional arguments:
  -h, --help  show this help message and exit
  -t 5        for specifying number of trucks
  -p 5        for specifying number of packages
  -d 5        for specifying number of depots
  -g 5        for specifying grid scale
  -s 32       for providing random seed
```
  Although the packages and trucks are randomly assigned to depots, these can be changed by changing `temp_env.json`.
  
3. Run the solver.
```
$ rosrun group_4 solver1.py
  uses: k-means clustering and DP for solving TSP.
```
(or)
```
$ rosrun group_4 solver2.py
  uses: sweep line clustering and greedy solver for TSP.
```

4. Start Gazebo.
```
$ roslaunch group_4 maze.launch
```

5. Start movetbot service.
```
$ rosrun group_4 move_tbot3.py
```

6. Run the execute service.
```
$ rosrun group_4 execute.py -h
usage: execute.py [-h] [-p 5] [-f 5]

optional arguments:
  -h, --help  show this help message and exit
  -p 5        for specifying plan file
  -f 5        for specifying environment file
```

## Possible Issues/ Troubleshooting

1. Sometimes, if the execute service is started too fast, the first move action may get ignored.

2. Robot can randomly spin (similar to assignment 2).

3. Solver 2 Issues with clustering if there are too few packages.
