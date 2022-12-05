// Copyright 2019 Intelligent Robotics Lab
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

#include <memory>
#include <algorithm>
#include <iostream>
#include <regex>
#include <string>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "text_to_speech_interfaces/action/tts.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ListenAction : public rclcpp::Node
{
public:
  ListenAction()
  : rclcpp::Node("listen_node")
  {
  }

  void init(){
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
      
    sound_sub_ = create_subscription<std_msgs::msg::String>(
      "/audio_detection/audio_detected",
      10,
      std::bind(&ListenAction::current_sound_callback, this, _1));
    
  }

  void current_sound_callback(const std_msgs::msg::String msg)
  {
    //Add new sound instance
    if(!problem_expert_->getInstance(msg.data).has_value()){
      //RCLCPP_INFO(get_logger(), "Adding the new instance: %s", msg.data.c_str());
      problem_expert_->addInstance(plansys2::Instance{msg.data, "sound"});
    }

    //Remove old predicate sound_listened if exists
    std::string old_predicate = "(sound_listened "+current_sound_+")";
    if(problem_expert_->getPredicate(old_predicate).has_value()){
      //RCLCPP_INFO(get_logger(), "Remove predicate: %s", old_predicate.c_str());
      problem_expert_->removePredicate(plansys2::Predicate(old_predicate));
    }
    
    
    //Add new predicate sound_listened
    std::string new_predicate = "(sound_listened "+msg.data+")";
    problem_expert_->addPredicate(plansys2::Predicate(new_predicate));
    RCLCPP_INFO(get_logger(), "%s", new_predicate.c_str());
    current_sound_ = msg.data;

  }

private:

  //std_msgs::msg::String current_sound_;
  std::string current_sound_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sound_sub_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ListenAction>();
  node->init();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
