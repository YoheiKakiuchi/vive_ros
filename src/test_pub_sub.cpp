#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <iostream>

using std::placeholders::_1;

class HOGE
{
public:
    HOGE(const std::string _name) : ros_clock(RCL_ROS_TIME) {
        g_node = rclcpp::Node::make_shared(_name);
        tf_broadcaster_ = new tf2_ros::TransformBroadcaster(g_node);

        sub = g_node->create_subscription<std_msgs::msg::String>("subtopic", 10,
              std::bind(&HOGE::topic_callback, this, _1));
        pub = g_node->create_publisher<std_msgs::msg::String>("pubtopic", 10);
    }
    rclcpp::Node::SharedPtr g_node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    //pub;
    void topic_callback(const std_msgs::msg::String & msg)
    {
        RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg.data.c_str());
        //std_msgs::msg::String pmsg;
        //pmsg.data = "replay: " + msg.data;
        //pub->publish(pmsg);
#if 0
        tf2::Transform tf;
        tf.setOrigin(1, 2, 3);
        tf2::Quaternion quat;
        tf2::Matrix3x3 rot_matrix(1, 0, 0,
                                  0, 1, 0,
                                  0, 0, 1);
        rot_matrix.getRotation(quat);
        tf.setRotation(quat);
        geometry_msgs::msg::TransformStamped(tf, ros::Time::now(), "world_vive", "hmd")
#endif
        rclcpp::Time now = ros_clock.now();
        Eigen::Isometry3d eigenPos = Eigen::Isometry3d::Identity();
        geometry_msgs::msg::TransformStamped tf = eigenToTransform(eigenPos);
        tf.header.stamp = now;
        tf.header.freame_id = "world_vive";
        tf.child_frame_id = "hmd";
        tf_broadcaster_->sendTransform(tf);
    }
private:
    rclcpp::Clock ros_clock;

    tf2_ros::TransformBroadcaster *tf_broadcaster_;
    //tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  HOGE hoge("hoge");
  //rclcpp::spin(hoge.g_node);
  rclcpp::Rate r_(1.0);
  rclcpp::executors::SingleThreadedExecutor exec;
  //rclcpp::executors::StaticSingleThreadedExecutor exec;
  exec.add_node(hoge.g_node);
  while(1) {
      std::cerr << "spin!!" << std::endl;
      //rclcpp::spin_some(hoge.g_node);
      //exec.spin_once();
      exec.spin_all(std::chrono::nanoseconds(10000000000));
      r_.sleep();
  }
  exec.remove_node(hoge.g_node);
  rclcpp::shutdown();
  return 0;
}
