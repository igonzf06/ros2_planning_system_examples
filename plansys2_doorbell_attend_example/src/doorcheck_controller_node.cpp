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

#include <plansys2_pddl_parser/Utils.h>

#include <memory>
#include <algorithm>

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DoorCheckController : public rclcpp::Node
{
public:
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;

  DoorCheckController()
  : rclcpp::Node("doorcheck_controller"), state_(STARTING)
  {
  }

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    init_knowledge();
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"tiago", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"entrance", "room"});
    problem_expert_->addInstance(plansys2::Instance{"livingroom", "room"});
    problem_expert_->addInstance(plansys2::Instance{"door", "door"});

    problem_expert_->addPredicate(plansys2::Predicate("(connected entrance livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(connected livingroom entrance)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tiago livingroom)"));
    problem_expert_->addPredicate(plansys2::Predicate("(door_at door entrance)"));
  }

  bool step()
  {

    // Set the goal for next state
    problem_expert_->setGoal(plansys2::Goal("(and(door_checked door))"));

    // Compute the plan
    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      std::cout << "Could not find plan to reach goal " <<
        parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
      RCLCPP_ERROR(get_logger(), "Could not find plan to reach goal.");
      return false;
    }

    // Execute the plan
    if (executor_client_->start_plan_execution(plan.value())) {
      std::cout << "Execute the following plan: " << std::endl;
      RCLCPP_INFO(get_logger(), "Execute the following plan: ");
      for (auto const &i: plan.value().items){
        std::cout << i.action << std::endl;
        //RCLCPP_INFO(get_logger(), i.action);
      }
    } 
    return true;
  }

private:
  typedef enum {STARTING, DOORBELL_LISTENED} StateType;
  StateType state_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoorCheckController>();
  std::list<std::string> doorbell_sounds = {"doorbell", "bell", "ding-dong", "tubular_bells", "reversing_beeps", "beepbleep", "chime"};

  node->init();

  bool sound_listened = false;
  rclcpp::Rate rate(5);
  while (!sound_listened) {
    auto predicates = node->problem_expert_->getPredicates();
    
    auto it = std::find_if(predicates.begin(), predicates.end(), [&doorbell_sounds](const plansys2::Predicate& pred) {
      return (std::find(doorbell_sounds.begin(), doorbell_sounds.end(), pred.parameters[0].name) != doorbell_sounds.end());
    });

    if(it != predicates.end()){
      RCLCPP_INFO(node->get_logger(), "doorbell sound listened.");
      sound_listened = true;
      node->step();
    }
    

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
