/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <toggle_radar_msgs/msg/toggle_radar.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <stdint.h>
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>
#include <vector>

// using namespace std::chrono;
// using namespace std::chrono_literals;

class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_zone_visualizer_node") {

			_zone_subscription_ = this->create_subscription<toggle_radar_msgs::msg::ToggleRadar>(
			"/radar_toggle", 10,
			std::bind(&RadarPCLFilter::zone_marker, this, std::placeholders::_1));

			_marker_pub = 
			this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar_zones", 10);


			// Zones init
			_zone_front.header = std_msgs::msg::Header();
			_zone_front.header.frame_id = "drone";
			_zone_front.action = visualization_msgs::msg::Marker::ADD;
			_zone_front.color.r = 0.9f;
			_zone_front.color.g = 0.9f;
			_zone_front.color.b = 0.9f;
			_zone_front.color.a = 0.0f;
			_zone_front.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
			_zone_front.lifetime = rclcpp::Duration::from_seconds(0);
			// _zone_front.pose.orientation.x = drone_arrow_rotation(0);
			// _zone_front.pose.orientation.y = drone_arrow_rotation(1);
			// _zone_front.pose.orientation.z = drone_arrow_rotation(2);
			// _zone_front.pose.orientation.w = drone_arrow_rotation(3);
			// _zone_front.pose.position.x = _drone_pose.position(0); //0; 
			// _zone_front.pose.position.y = _drone_pose.position(1); //0;
			// _zone_front.pose.position.z = _drone_pose.position(2); //0; 
			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			_zone_front.scale.x = 0.0045;
			_zone_front.scale.y = 0.0045;
			_zone_front.scale.z = 0.0045;
			std::string mesh_path;

			_zone_rear = _zone_front;
			_zone_top = _zone_front;
			_zone_bot = _zone_front;
			_zone_right = _zone_front;
			_zone_left = _zone_front;
			
			_zone_front.ns = "zone_front";
			_zone_front.id = 81;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/front_zone.stl";
			_zone_front.mesh_resource = mesh_path;

			_zone_rear.ns = "zone_rear";
			_zone_rear.id = 82;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/rear_zone.stl";
			_zone_rear.mesh_resource = mesh_path;

			_zone_top.ns = "zone_top";
			_zone_top.id = 83;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/top_zone.stl";
			_zone_top.mesh_resource = mesh_path;

			_zone_bot.ns = "zone_bot";
			_zone_bot.id = 84;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/bot_zone.stl";
			_zone_bot.mesh_resource = mesh_path;

			_zone_right.ns = "zone_right";
			_zone_right.id = 85;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/right_zone.stl";
			_zone_right.mesh_resource = mesh_path;

			_zone_left.ns = "zone_left";
			_zone_left.id = 86;
			mesh_path = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone-visualizer") + "/mesh/left_zone.stl";
			_zone_left.mesh_resource = mesh_path;


		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_zone_visualizer_node...");
		}

	private:

		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;

		rclcpp::Subscription<toggle_radar_msgs::msg::ToggleRadar>::SharedPtr _zone_subscription_;

		void zone_marker(const std::shared_ptr<toggle_radar_msgs::msg::ToggleRadar> msg);

		visualization_msgs::msg::Marker _zone_front;
		visualization_msgs::msg::Marker _zone_rear;
		visualization_msgs::msg::Marker _zone_top;
		visualization_msgs::msg::Marker _zone_bot;
		visualization_msgs::msg::Marker _zone_right;
		visualization_msgs::msg::Marker _zone_left;

};




void RadarPCLFilter::zone_marker(const std::shared_ptr<toggle_radar_msgs::msg::ToggleRadar> msg) {

	visualization_msgs::msg::MarkerArray marker_array;

	_zone_front.header.stamp = this->now();
	_zone_rear.header.stamp = _zone_front.header.stamp;
	_zone_top.header.stamp = _zone_front.header.stamp;
	_zone_bot.header.stamp = _zone_front.header.stamp;
	_zone_right.header.stamp =_zone_front.header.stamp;
	_zone_left.header.stamp = _zone_front.header.stamp;

	_zone_front.color.a = msg->radar_toggle_array[0]*0.25f; 
	_zone_rear.color.a = msg->radar_toggle_array[1]*0.25f; 
	_zone_top.color.a = msg->radar_toggle_array[2]*0.25f; 
	_zone_bot.color.a = msg->radar_toggle_array[3]*0.25f; 
	_zone_right.color.a = msg->radar_toggle_array[4]*0.25f; 
	_zone_left.color.a = msg->radar_toggle_array[5]*0.25f; 


	// if (msg->radar_toggle_array[0] == true)
	// {
	// 	_zone_front.color.a = 0.75f; // make zone visible
	// }

	// if (msg->radar_toggle_array[1] == true)
	// {
	// 	_zone_rear.color.a = 0.75f; // make zone visible
	// }

	// if (msg->radar_toggle_array[2] == true)
	// {
	// 	_zone_top.color.a = 0.75f; // make zone visible
	// }

	// if (msg->radar_toggle_array[3] == true)
	// {
	// 	_zone_bot.color.a = 0.75f; // make zone visible
	// }
	
	// if (msg->radar_toggle_array[4] == true)
	// {
	// 	_zone_right.color.a = 0.75f; // make zone visible
	// }

	// if (msg->radar_toggle_array[5] == true)
	// {
	// 	_zone_left.color.a = 0.75f; // make zone visible
	// }

	marker_array.markers.push_back(_zone_front);
	marker_array.markers.push_back(_zone_rear);
	marker_array.markers.push_back(_zone_top);
	marker_array.markers.push_back(_zone_bot);
	marker_array.markers.push_back(_zone_right);
	marker_array.markers.push_back(_zone_left);

	this->_marker_pub->publish(marker_array);

}


int main(int argc, char* argv[]) {
	std::cout << "Starting radar zone visualizer node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}