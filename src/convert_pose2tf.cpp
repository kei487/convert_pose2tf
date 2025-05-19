// Copyright 2025 Keitaro Nakamura
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


#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    RCLCPP_INFO(this->get_logger(), "start pose2tf_converter" );
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mcl_pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "set subscriver" );
  }

  void init_tf()
  {
    tfb_.reset();
//    tfl_.reset();
    tf_.reset();
    
    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    RCLCPP_INFO(this->get_logger(), "set tf_buffer" );
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface(),
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
    tf_->setCreateTimerInterface(timer_interface);
    RCLCPP_INFO(this->get_logger(), "set tf_timer" );
//    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    RCLCPP_INFO(this->get_logger(), "set tf_broadcaster" );
    latest_tf_ = tf2::Transform::getIdentity();
    RCLCPP_INFO(this->get_logger(), "set tf" );
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped & msg)
  {
    geometry_msgs::msg::PoseStamped odom_to_map;
	try {
                double x = msg.pose.position.x;
                double y = msg.pose.position.y;
		tf2::Quaternion q;
		tf2::fromMsg(msg.pose.orientation, q);

                RCLCPP_INFO(this->get_logger(), "I heard: '%f %f %f'", x, y, tf2::getYaw(q) );
		tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));

		geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = footprint_frame_id_;
		tmp_tf_stamped.header.stamp = rclcpp::Time(); //scan_time_stamp_;
                //tf2::Transform inverse_tf = tmp_tf.inverse();
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
		//tf2::toMsg(inverse_tf, tmp_tf_stamped.pose);

		tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
	} catch (tf2::TransformException & e) {
		RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
		return;
	}
	tf2::convert(odom_to_map.pose, latest_tf_);
	auto stamp = tf2_ros::fromMsg(rclcpp::Time());//scan_time_stamp_);
	tf2::TimePoint transform_expiration = stamp + tf2::durationFromSec(transform_tolerance_);

	geometry_msgs::msg::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = global_frame_id_;
	tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
	tmp_tf_stamped.child_frame_id = odom_frame_id_;
	tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

	tfb_->sendTransform(tmp_tf_stamped);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string footprint_frame_id_ = std::string("bese_footprint");
  std::string global_frame_id_ = std::string("map");
  std::string odom_frame_id_ = std::string("odom");
  double transform_tolerance_ = 0.2;
  tf2::Transform latest_tf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  node->init_tf();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
