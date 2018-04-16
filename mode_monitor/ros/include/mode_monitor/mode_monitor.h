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
      void run();
      void check(double wx, double wy);

    private:
      costmap_2d::Costmap2D* costmap_;
      tf::TransformListener tf_;
      ros::Subscriber costmap_sub_;
      ros::Publisher point_debug_;
      bool is_costmap_received_;
      nav_msgs::OccupancyGrid grid_msg_;

  };
};
