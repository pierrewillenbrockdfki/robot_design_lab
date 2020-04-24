# rdl_sim_world

Simulated environments in Gazebo 9 for robot design lab lecture.

Contains turtlebot3_world (see http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#2-turtlebot3-world)

however with special settings to reduce simulation complexity and ease to be run on systems with no dedicated GPU.

Contains simple_competition environment which can be used for developments purposes, however the

final competition environment will not be disclosed until x hours before the event, this is to

avoid an over tuning of the parameters to a particular setup but rather to encourage your code to

be flexible and adaptable to multiple scenarios.

## install dependencies

sudo apt install ros-melodic-gazebo-ros ros-melodic-turtlebot3-gazebo

## usage

run

    roslaunch rdl_sim_worlds turtlebot3_world.launch
    roslaunch rdl_sim_worlds simple_competition_world.launch

or you can include from other launch files using this snippet:

    <arg name="gazebo_gui" default="true"/>

    <!-- launch turtlebot3_world simulated environment in Gazebo -->
    <include file="$(find rdl_sim_worlds)/launch/turtlebot3_world.launch">
        <arg name="gazebo_gui" value="$(arg gazebo_gui)"/>
    </include>

or a more generic one:

    <arg name="gazebo_gui" default="true"/>
    <arg name="world_name" default="simple_competition"/> <!-- options: empty, turtlebot3_world, simple_competition -->

    <!-- launch turtlebot3_world simulated environment in Gazebo -->
    <include file="$(find rdl_sim_worlds)/launch/rdl_sim_worlds_example.launch">
        <arg name="gazebo_gui" value="$(arg gazebo_gui)"/>
        <arg name="world_name" value="$(arg world_name)"/>
    </include>

## FAQ

### Open gazebo gui during runtime after having passed false to not open it before

If for some reason you passed gazebo_gui:=false, or the launch file does it
by default for you, but you want to open it during runtime you can use:

    rosrun gazebo_ros gzclient __name:=my_awesome_gui

the `__name:=my_awesome_gui` part is to rename the node and avoid the warning:

    [ WARN] [/gazebo]: Shutdown request received.
    [ WARN] [/gazebo]: Reason given for shutdown: [new node registered with same name]
