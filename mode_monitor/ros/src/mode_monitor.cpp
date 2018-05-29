  #include <mode_monitor/mode_monitor.h>
#include <ros/ros.h>

namespace mode_monitor{
  ModeMonitor::ModeMonitor(): costmap_(), unknown_value_(99), tf_(ros::Duration(10)), is_costmap_received_(false){
    ROS_INFO("Constructor");
    ros::NodeHandle nh("");
    //"/move_base/global_costmap/costmap"
    costmap_sub_ = nh.subscribe("/move_base/global_costmap/costmap", 1, &ModeMonitor::costmapCB,this);
    point_debug_ = nh.advertise<nav_msgs::OccupancyGrid>("point_debug", 1);
    ros::spinOnce();
    //ros::spin();
  }

  void ModeMonitor::run(){
    if (is_costmap_received_){
      double x,y = 0.0;
      if(getTransform(x,y)){
        check(x,y);
      }
    }
  }

  bool ModeMonitor::getTransform(double& x, double& y){
    tf::TransformListener listener;

    ros::Rate rate(10.0);
    tf::StampedTransform transform;

    listener.waitForTransform("/base_link", "/map",
                              ros::Time(0), ros::Duration(1.0));

    try{
      listener.lookupTransform("/base_link", "/map",
                               ros::Time(0), transform);
      x = transform.getOrigin().x();
      y = transform.getOrigin().x();
      return true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  }


  void ModeMonitor::check(double wx, double wy){
    unsigned int mx, my = 0;
    costmap_->worldToMap(wx, wy,mx, my);

    grid_msg_.data[costmap_->getIndex(mx,my)] = 100; //TODO

    double cost = (double)costmap_->getCost(mx,my);

    if (cost == unknown_value_){
      ROS_WARN_STREAM("COST " << cost << " coord " << mx << ", "<<my);
    }

    grid_msg_.header.stamp = ros::Time::now();
    //point.data = &array;
    point_debug_.publish(grid_msg_);
  }

  void ModeMonitor::costmapCB(const nav_msgs::OccupancyGridConstPtr& input_costmap_){
    costmap_ = new costmap_2d::Costmap2D(input_costmap_->info.width,
                                        input_costmap_->info.height,
                                        input_costmap_->info.resolution,
                                        input_costmap_->info.origin.position.x,
                                        input_costmap_->info.origin.position.y,
                                        0);
    grid_msg_.data.clear();

    uint x, y;

    for (int i =0; i< input_costmap_->info.width * input_costmap_->info.height; ++i){
      costmap_->indexToCells(i,x,y);
      costmap_->setCost(x, y, input_costmap_->data[i]);
      grid_msg_.data.push_back(input_costmap_->data[i]);//array[i*j+1] = 0;//input_costmap_->data[i*j+i]);
      ROS_DEBUG_STREAM((int)costmap_->getCost(x,y));
    }

    ROS_INFO("Map has been copied successfully");

    //Filling Msg
     grid_msg_.header.frame_id = "map";
    grid_msg_.info.resolution = costmap_->getResolution();
    grid_msg_.info.width = costmap_->getSizeInCellsX();
    grid_msg_.info.height = costmap_->getSizeInCellsY();
    grid_msg_.info.origin.position.x = costmap_->getOriginX();
    grid_msg_.info.origin.position.y = costmap_->getOriginY();

    is_costmap_received_ = true;
    costmap_sub_.shutdown();
  }

  ModeMonitor::~ModeMonitor(){
    std::cout << "desconstructor";

  }
}
