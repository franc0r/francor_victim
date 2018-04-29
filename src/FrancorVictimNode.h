#ifndef FRANCORVICTIMNODE_H_
#define FRANCORVICTIMNODE_H_



#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <francor_msgs/SensorHeadCmd.h>
#include <rona_lib/Map/GridMap.h>
#include <rona_lib/Map/Operations.h>
#include <rona_lib/Utility.h>
#include <tf/transform_listener.h>
//dyn reconfig

class FrancorVictimNode
{

public:
  FrancorVictimNode();
  virtual ~FrancorVictimNode();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void start(double duration = 0.01);

private: //functions
  /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
  void run();

  void loop_callback(const ros::TimerEvent& e);

  //void subCallback(const ROS_PACK::MESSAGE& msg);
  void sub_map_callback(const nav_msgs::OccupancyGrid& msg);
  void sub_sh_pos_callback(const francor_msgs::SensorHeadCmd& msg);
  void sub_add_victim_callback(const std_msgs::Bool& msg);

  void draw_victims(std::shared_ptr<rona::map::GridMap> map)
  {
    for(auto& e : _victims)
    {
      rona::map::Operations::drawFilledCircle(*(map->getGrid()), e, 0.2, 150);
    }
  }


  //void dynreconfig_callback(FrancorVictimNode::FrancorVictimNodeConfig &config, uint32_t level);
private: //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubMap;

  ros::Subscriber _subMap;
  ros::Subscriber _subShPos;
  ros::Subscriber _subAddVictim;

  tf::TransformListener _tf;

  double _pan_angle;
  double _dist;

  std::shared_ptr<rona::map::GridMap> _map_small;
  std::shared_ptr<rona::map::GridMap> _map;

  ros::Timer _loopTimer;

  std::vector<rona::map::Point2D> _victims;

};

#endif  //FRANCORVICTIMNODE_H_
