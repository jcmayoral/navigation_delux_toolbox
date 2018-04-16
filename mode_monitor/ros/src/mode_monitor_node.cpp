#include <ros/ros.h>
#include <mode_monitor/mode_monitor.h>

int main(int argc,char** argv){
  ros::init(argc, argv, "mode_monitor_node");
  mode_monitor::ModeMonitor monitor;
  std::cout << "EXIT";
  return 0;
}
