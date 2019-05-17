# Drone meshnetwork simulation

The Drone meshnetwork simulation is a ros package used for testing with drone distributions and networking protocols.

The current implementation supports:
  - Hybride Lightweight Mobile Routing
  - Drone movement
  - Drone movement requests through a virtual gateway
  - Automatic network repairs using drone movement

### Tech

Drone meshnetwork simulation uses a number of free to use software distributions to work properly:

* [ROS Melodic] - ros version
* [Gazebo] - Simulation enviroment
* [Ubuntu] - Version: 18.04.2 LTS


### Building the software

Drone meshnetwork simulation requires [ROS Melodic] and [Gazebo] ( *Comes with ros-melodic-desktop-full* ) to be installed and the user has followed the [ROS Tutorials].
By following the ROS tutorials the user will have basic knowledge of use the Catkin Workspace the comes with ROS.

Start with placing the package in the catkin_workspace.

```sh
$ cp -R drone_meshnetwork_simulation <your_catkin_workspace>/src
```

navigate towards your catkin_workspace and source the workspace with
```sh
$ source devel/setup.bash
```
Use catkin_make to build the software

```sh
$ catkin_make drone_meshnetwork_simulation
```

To build the software with google unit testing enabled use 
```sh
$ catkin_make drone_meshnetwork_simulation
```

### Configuration
To configure the amount of routers and gateways drones edit the provided drone_meshnetwork_simulation/world/factory.world file.

```xml
<?xml version="1.0"?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
      <size> 10000 10000 </size>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <plugin name="DroneFactory" filename="libDroneFactory.so">
      <amountOfGatewayDrones>1</amountOfGatewayDrones><!--Edit this for the amount of gateways to use -->
      <amountOfRouterDrones>5</amountOfRouterDrones> <!--Edit this for the amount of routers to use -->
      <Debug>1</Debug><!--Use this if you want the drones to publish debuginformation to a ros topic -->
    </plugin>
  </world>
</sdf>
```

### Using the software

There is a launchfile provided that starts a gazebo server filled with drones in the role of routers or gateways.
To run the launchfile enter the following command. 

```sh
roslaunch drone_meshnetwork_simulation factory.launch 
```
The id of the drone start with 1 that goes up with 1 for eacht drones.
It starts with creating gateways and after that the routers.


To visually see what is happening in the simulation boot the gazebo client by using.

```sh
gzlient 
```

The easiest way to control the drones is by using the rqt service caller and topic monitor, run rqt by using.

```sh
rqt
```


### Todos

 - Write more Tests
 - Implement improved abstract drone
 - Setup buildserver for ROS package
 - Use improved code checker
 - Come up with a better drone distribution system


**Free Software**

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)


   [ROS Melodic]: <http://wiki.ros.org/melodic>
   [Gazebo]: <http://gazebosim.org/>
   [Ubuntu]: <http://releases.ubuntu.com/18.04/>
   [ROS Tutorials]:<http://wiki.ros.org/ROS/Tutorials>

