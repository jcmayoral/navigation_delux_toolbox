 #include <costmap_2d/costmap_2d.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <nav_msgs/OccupancyGrid.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <geometry_msgs/PointStamped.h>
 #include <tf/tf.h>

namespace mode_monitor{

  class ModeMonitor{
    public:
      ModeMonitor();
      ~ModeMonitor();
      void costmapCB(const nav_msgs::OccupancyGridConstPtr& input_costmap_);

    private:
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      tf::TransformListener tf_;
      ros::Subscriber costmap_sub_;
      ros::Publisher point_debug_;

  };
};
