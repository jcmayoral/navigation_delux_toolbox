#include <mode_monitor/mode_monitor.h>
#include <ros/ros.h>

namespace mode_monitor{
  ModeMonitor::ModeMonitor(): costmap_(), tf_(ros::Duration(10)){
    ROS_INFO("Constructor");
    costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    std::cout << "constructor";
    costmap_ros_->start();
    costmap_ = costmap_ros_->getCostmap();
    costmap_ros_->updateMap();

    ros::spin();

  }

  ModeMonitor::~ModeMonitor(){
    std::cout << "desconstructor";

  }
}
