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

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "text_to_speech_interfaces/action/tts.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class CheckDoorAction : public plansys2::ActionExecutorClient
{
public:
  CheckDoorAction()
  : plansys2::ActionExecutorClient("checkdoor", 250ms)
  {
    progress_ = 0.0;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    send_feedback(0.0, "Check starting");

    tts_action_client_ =
      rclcpp_action::create_client<text_to_speech_interfaces::action::TTS>(
      shared_from_this(),
      "text_to_speech/tts");

    bool is_action_server_ready = false;
    do {
      RCLCPP_INFO(get_logger(), "Waiting for text_to_speech action server...");

      is_action_server_ready =
        tts_action_client_->wait_for_action_server(std::chrono::seconds(5));
    } while (!is_action_server_ready);

    RCLCPP_INFO(get_logger(), "text_to_speech action server ready");

    tts_goal_.text = "You can come in";

    RCLCPP_INFO(get_logger(), "Checking the door");

    auto send_goal_options =
      rclcpp_action::Client<text_to_speech_interfaces::action::TTS>::SendGoalOptions();

    send_goal_options.result_callback = [this](auto) {
        finish(true, 1.0, "Door attended");
      };

    future_tts_goal_handle_ =
      tts_action_client_->async_send_goal(tts_goal_, send_goal_options);

    return ActionExecutorClient::on_activate(previous_state);
  }

private:

  using TTSGoalHandle =
    rclcpp_action::ClientGoalHandle<text_to_speech_interfaces::action::TTS>;
  using TTSFeedback =
    const std::shared_ptr<const text_to_speech_interfaces::action::TTS::Feedback>;
  rclcpp_action::Client<text_to_speech_interfaces::action::TTS>::SharedPtr tts_action_client_;
  std::shared_future<TTSGoalHandle::SharedPtr> future_tts_goal_handle_;
  text_to_speech_interfaces::action::TTS::Goal tts_goal_;

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CheckDoorAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "checkdoor"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
