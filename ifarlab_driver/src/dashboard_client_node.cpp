// Copyright 2019, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#include "ifarlab_driver/dashboard_client_ros.hpp"

#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include "ifarlab_driver/urcl_log_handler.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ur_dashboard_client");

  // The IP address under which the robot is reachable.
  std::string robot_ip = node->declare_parameter<std::string>("robot_ip", "192.168.4.5");
  node->get_parameter<std::string>("robot_ip", robot_ip);

  ifarlab_driver::registerUrclLogHandler("");  // Set empty tf_prefix at the moment

  std::shared_ptr<ifarlab_driver::DashboardClientROS> client;
  try {
    client = std::make_shared<ifarlab_driver::DashboardClientROS>(node, robot_ip);
  } catch (const urcl::UrException& e) {
    RCLCPP_WARN(rclcpp::get_logger("Dashboard_Client"),
                "%s This warning is expected on a PolyScopeX robot. If you don't want to see this warning, "
                "please don't start the dashboard client. Exiting dashboard client now.",
                e.what());
    return 0;
  }

  rclcpp::spin(node);

  return 0;
}
