/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>

ros::Publisher ros_laserscan_pub;
std::string ros_laserscan_frame;
bool use_sim_time;

/////////////////////////////////////////////////
void gz_laserscan_Callback(ConstLaserScanStampedPtr &msg_in)
{ 
  std::cout << "callback" << std::endl;
  //std::cout << ">> " << msg_in->DebugString() << std::endl;
  sensor_msgs::LaserScan msg_out;
  msg_out.angle_min         = msg_in->scan().angle_min();
  msg_out.angle_max         = msg_in->scan().angle_min();
  msg_out.angle_increment   = msg_in->scan().angle_step();
  msg_out.time_increment    = 0;
  msg_out.scan_time         = 0;
  msg_out.range_min         = msg_in->scan().range_min();
  msg_out.range_max         = msg_in->scan().range_max();
  msg_out.header.frame_id   = ros_laserscan_frame;
  
  if(use_sim_time)
  {
    std::cout << "sim time" << std::endl;
    uint32_t sec = msg_in->time().sec();
    uint32_t nsec = msg_in->time().nsec();
    msg_out.header.stamp    = ros::Time( sec, nsec );
  }
  else
  {
    std::cout << "no sim time" << std::endl;
    msg_out.header.stamp    = ros::Time::now();
  }
  
  for(int i = 0; i < msg_in->scan().ranges_size(); i++){
    msg_out.ranges.push_back( msg_in->scan().ranges(i) );
  }
  for(int i = 0; i < msg_in->scan().intensities_size(); i++){
    msg_out.intensities.push_back( msg_in->scan().intensities(i) );
  }
  std::cout << "publish" << std::endl;
  ros_laserscan_pub.publish(msg_out);
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS
  std::cout << "main" << std::endl;
  ros::init(argc, argv, "Gazebo_laserscan");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  //ros_laserscan_frame = "";
  n.param<bool>("/use_sim_time", use_sim_time, 0);
  n_.param<std::string>("ros_laserscan_frame", ros_laserscan_frame,   "/scan");
  //n_.getParam("ros_laserscan_frame", ros_laserscan_frame);
  ros_laserscan_pub = n.advertise<sensor_msgs::LaserScan>(ros_laserscan_frame, 100);
    
  // When launched from a launch file we need to give Gazebo time to load
  std::cout << "ros sleep" << std::endl;
  ros::Duration(5.0).sleep();
  std::cout << "gazebo client" << std::endl;
  // Initialize Gazebo
  gazebo::setupClient(argc, argv);
  // gazebo::transport::init();
  std::cout << "gazebo node" << std::endl;
  gazebo::transport::NodePtr gz_node(new gazebo::transport::Node());
  gz_node->Init();
  //gazebo::transport::run();

  std::cout << "gazebo node ok" << std::endl;
  
  std::string gz_laserscan_topic = "";
  n_.getParam("gz_laserscan_topic", gz_laserscan_topic);
  std::cout << "subscribing to gazebo: " << gz_laserscan_topic << std::endl;
  gazebo::transport::SubscriberPtr gz_laserscan_sub = gz_node->Subscribe( gz_laserscan_topic,
                                                                          gz_laserscan_Callback);

  std::cout << "spin" << std::endl;
  // Spin
  ros::spin();
  std::cout << "fini" << std::endl;
  // Shutdown
  //gazebo::transport::fini();
  gazebo::shutdown();
}




