  #include <mode_monitor/mode_monitor.h>
#include <ros/ros.h>

namespace mode_monitor{
  ModeMonitor::ModeMonitor(): costmap_(), tf_(ros::Duration(10)){
    ROS_INFO("Constructor");
    //costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    std::cout << "constructor";
    //costmap_ros_->start();
    //costmap_ = costmap_ros_->getCostmap();
    //costmap_ros_->updateMap();
    ros::NodeHandle nh("");
    //"/move_base/global_costmap/costmap"
    costmap_sub_ = nh.subscribe("/map", 1, &ModeMonitor::costmapCB,this);
    point_debug_ = nh.advertise<nav_msgs::OccupancyGrid>("point_debug", 1);
    ros::spin();
  }

  void ModeMonitor::costmapCB(const nav_msgs::OccupancyGridConstPtr& input_costmap_){
    ROS_INFO("Inside CB");
    //tf::Stamped< tf::Pose > global_pose;
    //geometry_msgs::PoseStamped global_pose_stamped;
    //costmap_ros_->getRobotPose (global_pose);
    //tf::poseStampedTFToMsg(global_pose, global_pose_stamped);
    ROS_INFO("here1");
    nav_msgs::OccupancyGrid point;
    costmap_ = new costmap_2d::Costmap2D(input_costmap_->info.width,
                                        input_costmap_->info.height,
                                        input_costmap_->info.resolution,
                                        input_costmap_->info.origin.position.x,
                                        input_costmap_->info.origin.position.y,
                                        0);
    //costmap_->costmap_ = input_costmap_->data;
    int array[input_costmap_->info.width * input_costmap_->info.height];

    for (int i =0; i< input_costmap_->info.width; ++i){
        for (int j =0; j< input_costmap_->info.height; ++j){
          costmap_->setCost(i, j, input_costmap_->data[i*j+i]);
          point.data.push_back(0);//array[i*j+1] = 0;//input_costmap_->data[i*j+i]);
          ROS_DEBUG_STREAM((int)costmap_->getCost(i,j));
        }
    }
    ROS_INFO("here1");
    double wx = 0;//7.077;
    double wy = 0;///1.661;
    unsigned int mx, my = 0;

    costmap_->worldToMap(wx, wy,mx, my);
    ROS_INFO_STREAM("here1" << "here" << (int)costmap_->getCost(mx,my));
    point.data[costmap_->getIndex(mx,my)] = 100;
    costmap_->mapToWorld(mx, my,wx, wy);

    point.info = input_costmap_->info;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";
    //point.data = &array;
    point_debug_.publish(point);

  }

  ModeMonitor::~ModeMonitor(){
    std::cout << "desconstructor";

  }
}
