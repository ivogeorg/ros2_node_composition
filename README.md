### ros2_node_composition

ROS2 node components.

#### Cloning

`git clone https://github.com/ivogeorg/ros2_node_composition my_components`

#### `ros2 component types`

Note `my_components::MoveRobot` in the my_components` namespace (type).  

```
user:~/ros2_ws$ ros2 component types
my_components
  my_components::MoveRobot
robot_state_publisher
  robot_state_publisher::RobotStatePublisher
tf2_ros
  tf2_ros::StaticTransformBroadcasterNode
moveit_servo
  moveit_servo::ServoNode
  moveit_servo::JoyToServoPub
moveit_hybrid_planning
  moveit::hybrid_planning::HybridPlanningManager
  moveit::hybrid_planning::GlobalPlannerComponent
  moveit::hybrid_planning::LocalPlannerComponent
image_proc
  image_proc::RectifyNode
  image_proc::DebayerNode
  image_proc::ResizeNode
  image_proc::CropDecimateNode
  image_proc::CropNonZeroNode
stereo_image_proc
  stereo_image_proc::DisparityNode
  stereo_image_proc::PointCloudNode
image_view
  image_view::DisparityViewNode
  image_view::ExtractImagesNode
  image_view::ImageViewNode
  image_view::ImageSaverNode
  image_view::StereoViewNode
  image_view::VideoRecorderNode
image_rotate
  image_rotate::ImageRotateNode
depth_image_proc
  depth_image_proc::ConvertMetricNode
  depth_image_proc::CropForemostNode
  depth_image_proc::DisparityNode
  depth_image_proc::PointCloudXyzNode
  depth_image_proc::PointCloudXyzRadialNode
  depth_image_proc::PointCloudXyziNode
  depth_image_proc::PointCloudXyziRadialNode
  depth_image_proc::PointCloudXyzrgbNode
  depth_image_proc::PointCloudXyzrgbRadialNode
  depth_image_proc::RegisterNode
image_publisher
  image_publisher::ImagePublisher
composition
  composition::Talker
  composition::Listener
  composition::NodeLikeListener
  composition::Server
  composition::Client
depthimage_to_laserscan
  depthimage_to_laserscan::DepthImageToLaserScanROS
teleop_twist_joy
  teleop_twist_joy::TeleopTwistJoy
image_tools
  image_tools::Cam2Image
  image_tools::ShowImage
logging_demo
  logging_demo::LoggerConfig
  logging_demo::LoggerUsage
examples_rclcpp_minimal_subscriber
  WaitSetSubscriber
  StaticWaitSetSubscriber
  TimeTriggeredWaitSetSubscriber
action_tutorials_cpp
  action_tutorials_cpp::FibonacciActionClient
  action_tutorials_cpp::FibonacciActionServer
quality_of_service_demo_cpp
  quality_of_service_demo::MessageLostListener
  quality_of_service_demo::MessageLostTalker
  quality_of_service_demo::QosOverridesListener
  quality_of_service_demo::QosOverridesTalker
demo_nodes_cpp_native
  demo_nodes_cpp_native::Talker
demo_nodes_cpp
  demo_nodes_cpp::OneOffTimerNode
  demo_nodes_cpp::ReuseTimerNode
  demo_nodes_cpp::ServerNode
  demo_nodes_cpp::ClientNode
  demo_nodes_cpp::ListParameters
  demo_nodes_cpp::ParameterBlackboard
  demo_nodes_cpp::SetAndGetParameters
  demo_nodes_cpp::ParameterEventsAsyncNode
  demo_nodes_cpp::EvenParameterNode
  demo_nodes_cpp::ContentFilteringPublisher
  demo_nodes_cpp::ContentFilteringSubscriber
  demo_nodes_cpp::Talker
  demo_nodes_cpp::LoanedMessageTalker
  demo_nodes_cpp::SerializedMessageTalker
  demo_nodes_cpp::Listener
  demo_nodes_cpp::SerializedMessageListener
  demo_nodes_cpp::ListenerBestEffort
joy
  joy::Joy
```  

#### Runtime composition

Start the component container:
```
ros2 run rclcpp_components component_container
```

Check if it's running:
```
user:~/ros2_ws$ ros2 component list
/ComponentManager
```

#### Load (run) component

Structure of the command is `ros2 component load /ComponentManager <pkg_name> <component_name>`.

Load component where `/ComponentManager` was verified to be running:  
```
user:~/ros2_ws$ ros2 component load /ComponentManager my_components my_components::MoveRobot
Loaded component 1 into '/ComponentManager' container node as '/moverobot'
```  

The component container has loaded a node:  
```
user:~/ros2_ws$ ros2 component list
/ComponentManager
  1  /moverobot
```

Where `component_container` was run:  
```
user:~/ros2_ws$ ros2 run rclcpp_components component_container
[INFO] [1723842246.722373140] [ComponentManager]: Load Library: /home/user/ros2_ws/install/my_components/lib/libmoverobot_component.so
[INFO] [1723842246.724174203] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<my_components::MoveRobot>
[INFO] [1723842246.725208312] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<my_components::MoveRobot>
```

`Ctrl-C` to stop.  

#### Unload component

Structure `ros2 component unload /ComponentManager <component_id>`. <component_id> is _unique_.

```
user:~/ros2_ws$ ros2 component unload /ComponentManager 1
Unloaded component 1 from '/ComponentManager' container node
```

**Note** that the robot started by the component continues to move! So, stop it with:
```
user:~/ros2_ws$ ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```
and reset it with:
```
user:~/ros2_ws$ ros2 service call /reset_world std_srvs/srv/Empty
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()
```

#### Loading components with a launch file

See the [launch file](launch/movebot_component.launch.py)

```
user:~/ros2_ws$ ros2 launch my_components movebot_component.launch.py
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2024-08-16-21-38-33-622663-1_xterm-8010
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container-1]: process started with pid [8023]
[component_container-1] [INFO] [1723844314.098622060] [my_container]: Load Library: /home/user/ros2_ws/install/my_components/lib/libmoverobot_component.so
[component_container-1] [INFO] [1723844314.099674057] [my_container]: Found class: rclcpp_components::NodeFactoryTemplate<my_components::MoveRobot>
[component_container-1] [INFO] [1723844314.099725269] [my_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<my_components::MoveRobot>
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/moverobot' in container '/my_container'
```

#### Node composition

See the [launch file](launch/combine_components.launch.py)

#### Compile-time composition

Aka **manual composition**.  

Components are libraries!!! 

1. Source code:
   ```
   rclcpp::executors::SingleThreadedExecutor exec;
   rclcpp::NodeOptions options;

   auto moverobot = std::make_shared<my_components::MoveRobot>(options);
   exec.add_node(moverobot);
   auto readodom = std::make_shared<my_components::OdomSubscriber>(options);
   exec.add_node(readodom);

   exec.spin();
   ```
2. CMakeLists.txt:
   ```
   add_executable(manual_composition src/manual_composition.cpp)
   target_link_libraries(manual_composition moverobot_component readodom_component)
   ament_target_dependencies(manual_composition "rclcpp")
   ```

#### Service composition

Server and client are components.

1. Start container (`Ctrl-C` to stop):
```
ros2 run rclcpp_components component_container
[INFO] [1723848148.572690571] [ComponentManager]: Load Library: /home/user/ros2_ws/install/my_components/lib/libserver_component.so
[INFO] [1723848148.573569691] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<my_components::Server>
[INFO] [1723848148.573616998] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<my_components::Server>
[INFO] [1723848177.266208730] [ComponentManager]: Load Library: /home/user/ros2_ws/install/my_components/lib/libclient_component.so
[INFO] [1723848177.266929616] [ComponentManager]: Found class: rclcpp_components::NodeFactoryTemplate<my_components::Client>
[INFO] [1723848177.266968383] [ComponentManager]: Instantiate class: rclcpp_components::NodeFactoryTemplate<my_components::Client>
[INFO] [1723848179.276733139] [move_circle_client]: Result: success
[INFO] [1723848181.275520217] [move_circle_client]: Result: success
[INFO] [1723848183.275524208] [move_circle_client]: Result: success
```
2. Load components:
```
user:~/ros2_ws$ ros2 component load /ComponentManager my_components my_components::Server
Loaded component 1 into '/ComponentManager' container node as '/move_circle_service'
user:~/ros2_ws$ ros2 component list
/ComponentManager
  1  /move_circle_service
user:~/ros2_ws$ ros2 component load /ComponentManager my_components my_components::Client
Loaded component 2 into '/ComponentManager' container node as '/move_circle_client'
user:~/ros2_ws$ ros2 component list
/ComponentManager
  1  /move_circle_service
  2  /move_circle_client
user:~/ros2_ws$
```
3. Unload and stop robot:
```
user:~/ros2_ws$ ros2 component unload /ComponentManager 2
Unloaded component 2 from '/ComponentManager' container node
user:~/ros2_ws$ ros2 component unload /ComponentManager 1
Unloaded component 1 from '/ComponentManager' container node
user:~/ros2_ws$ ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0))
```

