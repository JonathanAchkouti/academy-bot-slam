#ifndef ACADBOT_COURIER__BT__NAVIGATE_TO_LOCATION_HPP_
#define ACADBOT_COURIER__BT__NAVIGATE_TO_LOCATION_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "acadbot_courier/location_book.hpp"
#include "acadbot_courier/nav_leg_client.hpp"
#include "acadbot_courier/types.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace acadbot_courier::bt
{

struct MissionContext
{
  rclcpp::Node * node;
  LocationBook * location_book;
  NavLegClient * nav_client;
  std::atomic<bool> * cancel_requested;
  std::function<void(const Leg &, std::uint16_t, const std::string &)> set_snapshot;

  double retry_backoff_sec;
  bool clear_costmap_before_retry;
  double leg_timeout_sec;

  std::uint16_t attempts_used{0};
  std::uint16_t current_attempt{0};
  std::string expected_pickup;
  std::string expected_dropoff;
  std::string current_leg{kLegPickup};
  std::string current_location;
  std::string failure_message;
  NavResult last_result{NavResult::ABORTED};
  bool cancel_confirmed{true};
  bool pickup_reached{false};
  bool dropoff_reached_after_pickup{false};
};

class NavigateToLocation : public BT::StatefulActionNode
{
public:
  NavigateToLocation(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionContext> context);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    RETRY_BACKOFF,
    NAVIGATING
  };

  BT::NodeStatus start_navigation();

  std::shared_ptr<MissionContext> context_;
  std::string location_;
  std::string leg_;
  std::uint16_t attempt_{0};
  Phase phase_{Phase::IDLE};
  std::chrono::steady_clock::time_point deadline_;
};

class Dwell : public BT::StatefulActionNode
{
public:
  Dwell(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<MissionContext> context);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<MissionContext> context_;
  std::chrono::steady_clock::time_point deadline_;
};

void register_courier_bt_nodes(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<MissionContext> & context);

}  // namespace acadbot_courier::bt

#endif  // ACADBOT_COURIER__BT__NAVIGATE_TO_LOCATION_HPP_
