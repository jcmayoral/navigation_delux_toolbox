 #include <costmap_2d/costmap_2d.h>
 #include <costmap_2d/costmap_2d_ros.h>

 #include <tf/tf.h>

namespace mode_monitor{

  class ModeMonitor{
    public:
      ModeMonitor();
      ~ModeMonitor();

    private:
      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      tf::TransformListener tf_;
  };
};
