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


// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// #include <std_msgs/msg/int32.hpp>

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
		RadarPCLFilter() : Node("marker_republisher_node") {

			_marker_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
			"/markers", 10,
			std::bind(&RadarPCLFilter::repub_marker, this, std::placeholders::_1));

			_marker_pub = 
			this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers_repub", 10);

		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down marker_republisher_node node..");
		}

	private:

		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_pub;

		rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr _marker_subscription_;

		void repub_marker(const visualization_msgs::msg::MarkerArray::SharedPtr msg);

};




void RadarPCLFilter::repub_marker(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {

	visualization_msgs::msg::MarkerArray marker_array;
	
	///////////////////////////////////////////////////////////
	// look-ahead volume republish

	for (long int i = 0; i < (end(msg->markers)-begin(msg->markers)); i++)
	{
		// RCLCPP_WARN(this->get_logger(),  "ID: %d", msg->markers[i].id);

   		if (msg->markers[i].id == 20)
		{
			visualization_msgs::msg::Marker look_ahead_marker = msg->markers[i];

			std::string mesh_path = look_ahead_marker.mesh_resource = "file://" + ament_index_cpp::get_package_share_directory("spherical-radar-drone") + "/mesh/look_ahead_cone.stl";
			look_ahead_marker.mesh_resource = mesh_path;

			marker_array.markers.push_back(look_ahead_marker);
			
			this->_marker_pub->publish(marker_array);

		}
	} 
}


int main(int argc, char* argv[]) {
	std::cout << "Starting marker republisher node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}