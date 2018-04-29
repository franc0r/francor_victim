
#include "FrancorVictimNode.h"

FrancorVictimNode::FrancorVictimNode()
{
  //rosParam
  ros::NodeHandle privNh("~");
  // std::string string_val;
  double dist;
  // int int_val;
  // bool bool_val;


  // privNh.param(         "string_val" ,    string_val,   std::string("string"));
  privNh.param<double>( "dist" ,    dist,   1.0);
  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);
  
  _dist = dist;
  
  //init publisher
  // _pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);
  _pubMap = _nh.advertise<nav_msgs::OccupancyGrid>("francor/victim/map", 1);

  //inti subscriber
  //_sub = _nh.subscribe("subname", 1, &FrancorVictimNode::subCallback, this);
  _subMap       = _nh.subscribe("map", 1, &FrancorVictimNode::sub_map_callback, this);
  _subShPos     = _nh.subscribe("/sensor_head/pos", 1, &FrancorVictimNode::sub_sh_pos_callback, this);
  _subAddVictim = _nh.subscribe("francor/add_victim", 1, &FrancorVictimNode::sub_add_victim_callback, this);

}

FrancorVictimNode::~FrancorVictimNode()
{
}

void FrancorVictimNode::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &FrancorVictimNode::loop_callback, this);
  this->run();
}

void FrancorVictimNode::run()
{
  ros::spin();
}

void FrancorVictimNode::loop_callback(const ros::TimerEvent& e)
{
  //do loop stuff here!!!
}

  void FrancorVictimNode::sub_map_callback(const nav_msgs::OccupancyGrid& msg)
  {
    _map = std::make_shared<rona::map::GridMap>(msg);
    this->draw_victims();
    _pubMap.publish(_map->getGrid()->toRosOccGrid());
  }
  void FrancorVictimNode::sub_sh_pos_callback(const francor_msgs::SensorHeadCmd& msg)
  {
    //use pan left 90 right -90
    _pan_angle = (msg.pan * M_PI) / 180.0;
  }
  void FrancorVictimNode::sub_add_victim_callback(const std_msgs::Bool& msg)
  {
    auto tf = rona::Utility::getTransform(_tf, "map", "base_footprint");
    
    //todo add some magic
    tf::Transform tf_sh;
    tf_sh.setIdentity();
    tf::Quaternion sh_rot;
    sh_rot.setEuler(0.0, 0.0, _pan_angle);
    tf_sh.setRotation(sh_rot);

    tf::Transform tf_dist;
    tf_dist.setIdentity();
    tf_dist.setOrigin(tf::Vector3(_dist, 0.0, 0.0));
   
    const auto tf_final = (tf * tf_sh) * tf_dist;

    // ROS_INFO_STREAM("TF_final: " << tf_final.getRotation().angle());

    rona::map::Point2D p;
    p.x = tf_final.getOrigin().getX();
    p.y = tf_final.getOrigin().getY();


    _victims.push_back(p);


    //pub map
    this->draw_victims();
    _pubMap.publish(_map->getGrid()->toRosOccGrid());

    ROS_INFO("Save Victim Map");
    //save as png
    cv::Mat save_map = _map->getGrid()->toCvMat();
    std::string name = "/tmp/victim_map" + std::to_string(_victims.size()) + ".png";
    
    for(unsigned int i = 0; i < _victims.size(); i++)
    {
      std::string name_v = "v_" + std::to_string(i);
      ROS_INFO_STREAM("write name: " << name_v);
      
      auto vp = _map->getGrid()->toPixel(_victims[i]);
      cv::Point cvvp(vp.x, vp.y - 10);

      cv::putText(save_map, name_v, cvvp, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(180), 2.0);
      // cv::putText(save_map, name_v, cv::Point(300,300), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(180));
    }

    cv::imwrite(name, save_map);
  }

// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "francor_victim_node");
  ros::NodeHandle nh("~");

  FrancorVictimNode node;
  node.start();
}
