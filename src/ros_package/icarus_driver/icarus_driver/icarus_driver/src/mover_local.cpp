// Copyright 2020 Robotech URJC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Fernando González fergonzaramos@yahoo.es */

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include "icarus_driver_msgs/TargetPose.h"
#include "icarus_driver/IcarusDriver.h"
#include "boca_negra/Bocanegra.h"

#define HZ 10
#define POSITION_MARGIN_ERROR 0.15
#define ORIENTATION_MARGIN_ERROR 0.035

class MoverLocal : public icarus_driver::IcarusDriver, boca_negra::Bocanegra
{
public:
  MoverLocal()
  :
  IcarusDriver(), Bocanegra(), nh_("~"), tf_listener_(tf_buffer_),
  finished_published_(false)

  {
    local_pose_sub_ = nh_.subscribe(local_pose_topic_, 1, &MoverLocal::localPoseCb, this);
    target_srv_ = nh_.advertiseService(
      "/icarus_driver/mover_local_srv", &MoverLocal::moveToLocalPose, this);
    finish_trigger_pub_ = nh_.advertise<std_msgs::Empty>(
      "/icarus_driver/mover_local/finished", 1);

    mavros_local_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      local_pose_setter_topic_, 1);
  }

  void
  update()
  {
    if (!isActive())
      return;

    std_msgs::Empty msg;
    if (localTargetReached())
    {
      if (!finished_published_)
        finish_trigger_pub_.publish(msg);
      finished_published_ = true;
    }
    else
    {
      target_pos_.header.stamp = ros::Time::now();
      mavros_local_pose_pub_.publish(target_pos_);
    }
  }

private:
  bool
  moveToLocalPose(icarus_driver_msgs::TargetPose::Request & req,
    icarus_driver_msgs::TargetPose::Response & res)
  {
    finished_published_ = false;

    std::string error;
    geometry_msgs::TransformStamped src_frame2map;

    target_pos_ = req.target_pose;
    if (!tf_buffer_.canTransform(
      "map", target_pos_.header.frame_id, ros::Time(0), ros::Duration(0.1), &error))
    {
      ROS_ERROR("%s", error.c_str());
      res.success = false;
      return false;
    }
    src_frame2map = tf_buffer_.lookupTransform("map", target_pos_.header.frame_id,
    ros::Time(0), ros::Duration(0.2));
    tf2::doTransform(target_pos_, target_pos_, src_frame2map);
    setMode("OFFBOARD");
    res.success = true;
    return true;
  }

  bool
  positionReached()
  {
    return abs(current_pos_.pose.position.x - target_pos_.pose.position.x) <= POSITION_MARGIN_ERROR
      && abs(current_pos_.pose.position.y - target_pos_.pose.position.y) <= POSITION_MARGIN_ERROR
      && abs(current_pos_.pose.position.z - target_pos_.pose.position.z) <= POSITION_MARGIN_ERROR;
  }

  bool
  orientationReached()
  {
    double t_roll, t_pitch, t_yaw;
    double c_roll, c_pitch, c_yaw;

    // Compose quaternions:

    tf::Quaternion t_q(target_pos_.pose.orientation.x, target_pos_.pose.orientation.y,
      target_pos_.pose.orientation.z, target_pos_.pose.orientation.w);

    tf::Quaternion c_q(current_pos_.pose.orientation.x, current_pos_.pose.orientation.y,
      current_pos_.pose.orientation.z, current_pos_.pose.orientation.w);

    tf::Matrix3x3(t_q).getRPY(t_roll, t_pitch, t_yaw);
    tf::Matrix3x3(c_q).getRPY(c_roll, c_pitch, c_yaw);

    return abs(t_roll - c_roll) <= ORIENTATION_MARGIN_ERROR
      && abs(t_pitch - c_pitch) <= ORIENTATION_MARGIN_ERROR
      && abs(t_yaw - c_yaw) <= ORIENTATION_MARGIN_ERROR;
  }

  bool
  localTargetReached()
  {
    return positionReached() && orientationReached();
  }

  void
  localPoseCb(const geometry_msgs::PoseStamped::ConstPtr & msg)
  {
    // Save drone global position

    current_pos_ = *msg;
  }

  ros::NodeHandle nh_;
  ros::Subscriber local_pose_sub_;
  ros::Publisher finish_trigger_pub_, mavros_local_pose_pub_;
  ros::ServiceServer target_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::PoseStamped current_pos_, target_pos_;

  bool finished_published_;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "mover_local_node");

  MoverLocal ml;

  ros::Rate loop_rate(HZ);
  while (ros::ok())
  {
    ml.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
