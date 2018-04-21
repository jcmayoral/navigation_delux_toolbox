#include <ros/ros.h>
#include <mode_monitor/mode_monitor.h>

int main(int argc,char** argv){
  ros::init(argc, argv, "mode_monitor_node");
  mode_monitor::ModeMonitor monitor;
  ros::Rate r= 1; // 1 Hz

  while(ros::ok){
    monitor.run();
    r.sleep();
    ros::spinOnce();
  }
  std::cout << "hpayhfa";
  std::cout << "EXIT";
  return 0;
}
