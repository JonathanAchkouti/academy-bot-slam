#include "acadbot_courier/bt/navigate_to_location.hpp"

#include <chrono>
#include <exception>
#include <utility>

namespace acadbot_courier::bt
{

NavigateToLocation::NavigateToLocation(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionContext> context)
: BT::StatefulActionNode(name, config), context_(std::move(context))
{
}

BT::PortsList NavigateToLocation::providedPorts()
{
  return {
    BT::InputPort<std::string>("location"),
    BT::InputPort<std::string>("leg")};
}

BT::NodeStatus NavigateToLocation::onStart()
{
  if (!getInput("location", location_) || !getInput("leg", leg_)) {
    context_->failure_message = "NavigateToLocation requires location and leg inputs";
    context_->last_result = NavResult::ABORTED;
    return BT::NodeStatus::FAILURE;
  }

  if (!context_->location_book->has(location_)) {
    context_->failure_message = "behavior tree referenced unknown location '" + location_ + "'";
    context_->last_result = NavResult::ABORTED;
    return BT::NodeStatus::FAILURE;
  }

  ++attempt_;
  context_->current_attempt = attempt_;
  context_->current_leg = leg_;
  context_->current_location = location_;
  context_->failure_message.clear();

  const Leg leg{leg_, location_, 0.0};
  if (attempt_ > 1) {
    context_->set_snapshot(leg, attempt_, kStateRetrying);
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(context_->retry_backoff_sec));
    phase_ = Phase::RETRY_BACKOFF;
    return BT::NodeStatus::RUNNING;
  }

  return start_navigation();
}

BT::NodeStatus NavigateToLocation::onRunning()
{
  if (context_->cancel_requested->load()) {
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::RETRY_BACKOFF) {
    if (std::chrono::steady_clock::now() < deadline_) {
      return BT::NodeStatus::RUNNING;
    }
    if (context_->clear_costmap_before_retry) {
      const Leg leg{leg_, location_, 0.0};
      context_->set_snapshot(leg, attempt_, kStateRecovering);
      context_->nav_client->clear_costmaps();
      if (context_->cancel_requested->load()) {
        return BT::NodeStatus::RUNNING;
      }
    }
    return start_navigation();
  }

  if (phase_ != Phase::NAVIGATING) {
    context_->failure_message = "NavigateToLocation entered an invalid execution phase";
    context_->last_result = NavResult::ABORTED;
    return BT::NodeStatus::FAILURE;
  }

  if (std::chrono::steady_clock::now() >= deadline_) {
    context_->cancel_confirmed = context_->nav_client->cancel();
    context_->last_result = NavResult::TIMEOUT;
    phase_ = Phase::IDLE;
    return BT::NodeStatus::FAILURE;
  }

  const auto result = context_->nav_client->poll_result();
  if (!result.has_value()) {
    return BT::NodeStatus::RUNNING;
  }

  phase_ = Phase::IDLE;
  context_->last_result = result.value();
  if (result.value() == NavResult::SUCCEEDED) {
    if (leg_ == kLegPickup && location_ == context_->expected_pickup) {
      context_->pickup_reached = true;
      // A later pickup starts a new delivery sequence. A previous dropoff can
      // no longer justify success for the newly picked-up mission payload.
      context_->dropoff_reached_after_pickup = false;
    } else if (
      leg_ == kLegDropoff && location_ == context_->expected_dropoff &&
      context_->pickup_reached)
    {
      context_->dropoff_reached_after_pickup = true;
    }
    return BT::NodeStatus::SUCCESS;
  }

  // A Nav2-side cancellation that was not requested by the courier is a failed
  // navigation attempt, not permission to report the outer action as canceled.
  if (result.value() == NavResult::CANCELED && !context_->cancel_requested->load()) {
    context_->last_result = NavResult::ABORTED;
  }
  return BT::NodeStatus::FAILURE;
}

void NavigateToLocation::onHalted()
{
  if (phase_ == Phase::NAVIGATING) {
    context_->cancel_confirmed = context_->nav_client->cancel();
    context_->last_result =
      context_->cancel_confirmed ? NavResult::CANCELED : NavResult::TIMEOUT;
  }
  phase_ = Phase::IDLE;
}

BT::NodeStatus NavigateToLocation::start_navigation()
{
  try {
    // Every new Nav2 goal starts without an outstanding cancellation. If this
    // goal is later halted, onHalted() replaces this with the real handshake result.
    context_->cancel_confirmed = true;
    auto target_pose = context_->location_book->pose(location_);
    context_->nav_client->set_target(target_pose);
    context_->set_snapshot(Leg{leg_, location_, 0.0}, attempt_, kStateNavigating);
    ++context_->attempts_used;

    if (!context_->nav_client->send(target_pose)) {
      context_->last_result = NavResult::ABORTED;
      context_->failure_message =
        "Nav2 rejected " + leg_ + " attempt " + std::to_string(attempt_);
      phase_ = Phase::IDLE;
      RCLCPP_WARN(context_->node->get_logger(), "%s", context_->failure_message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(context_->leg_timeout_sec));
    phase_ = Phase::NAVIGATING;
    return BT::NodeStatus::RUNNING;
  } catch (const std::exception & error) {
    context_->last_result = NavResult::ABORTED;
    context_->failure_message = error.what();
    phase_ = Phase::IDLE;
    RCLCPP_ERROR(context_->node->get_logger(), "BT navigation setup failed: %s", error.what());
    return BT::NodeStatus::FAILURE;
  }
}

Dwell::Dwell(
  const std::string & name,
  const BT::NodeConfig & config,
  std::shared_ptr<MissionContext> context)
: BT::StatefulActionNode(name, config), context_(std::move(context))
{
}

BT::PortsList Dwell::providedPorts()
{
  return {BT::InputPort<double>("seconds")};
}

BT::NodeStatus Dwell::onStart()
{
  double seconds = 0.0;
  if (!getInput("seconds", seconds) || seconds < 0.0) {
    context_->failure_message = "Dwell requires a non-negative seconds input";
    return BT::NodeStatus::FAILURE;
  }

  context_->set_snapshot(
    Leg{context_->current_leg, context_->current_location, seconds},
    context_->current_attempt,
    kStateDwelling);
  if (seconds == 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(seconds));
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dwell::onRunning()
{
  return std::chrono::steady_clock::now() >= deadline_ ?
         BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void Dwell::onHalted()
{
  // Dwell owns no external operation. Returning from onHalted stops it
  // immediately; the outer executor then reports the requested cancellation.
}

void register_courier_bt_nodes(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<MissionContext> & context)
{
  factory.registerNodeType<NavigateToLocation>("NavigateToLocation", context);
  factory.registerNodeType<Dwell>("Dwell", context);
}

}  // namespace acadbot_courier::bt
