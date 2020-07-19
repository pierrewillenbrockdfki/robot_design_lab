1) Configure move base to load this plugin.

For example, to add to global costmap upload to param server:

    global_costmap:
    global_frame: map

    update_frequency: 10.0
    publish_frequency: 10.0

    plugins:
        - name: static_layer
        type: 'costmap_2d::StaticLayer'
        - 
        name: inflation_layer
        type: 'costmap_2d::InflationLayer'
        - 
        name: custom_rdl_layer                             <------------ THIS PLUGIN!
        type: 'rdl_navigation_layers::RGBObstaclesLayer'   <------------ THIS PLUGIN!

2) run roscore, bringup, nav stack and publish a pose array msg to the topic offered by this plugin:

3) rostopic pub /move_base/global_costmap/custom_rdl_layer/rdl_rgb_obstacles geometry_msgs/PoseArray "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'map'
poses:
- position:
    x: 1.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
