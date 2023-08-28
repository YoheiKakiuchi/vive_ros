#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using std::placeholders::_1;

class HOGE
{
public:
    HOGE(const std::string _name) {
        g_node = rclcpp::Node::make_shared(_name);
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
    }
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
