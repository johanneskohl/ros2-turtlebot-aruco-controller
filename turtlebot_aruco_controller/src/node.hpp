#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
using namespace geometry_msgs;

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>


class ArucoControllerNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  using CycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  struct InitialMarkerReference
  {
    cv::Point2f center, topLeft;
    double distanceCornerCenter;
  };

public:
  ArucoControllerNode();

  /** TODO:
   *  - get parameters?
   *    - …
   *  - create publisher
   *  - create camera input stream
   *  - create aruco detection (params, dict, etc.)
   */
  CycleCallbackReturn on_configure(
    const rclcpp_lifecycle::State &state
  );

  /** TODO:
   *  - open stream
   *  - start timer function
   *  - on_activate of publisher
   */
  CycleCallbackReturn on_activate(
    const rclcpp_lifecycle::State &state
  );

  /** TODO:
   *  - close stream
   *  - stop timer function
   *  - on_deactivate of publisher
   */
  CycleCallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &state
  );

  CycleCallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &state
  )
  {
    return this->cleanupInternal(state);
  }

  CycleCallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &state
  )
  {
    return this->cleanupInternal(state);
  }

private:
  CycleCallbackReturn cleanupInternal(const rclcpp_lifecycle::State &state);
  msg::TwistStamped makeEmptyTwist() const;
  void mainloop();

private:
  cv::Ptr<cv::aruco::DetectorParameters> mParameters;
  cv::Ptr<cv::aruco::Dictionary> mDictionary;
  cv::VideoCapture mCapture;
  rclcpp::Logger mLogger;
  rclcpp_lifecycle::LifecyclePublisher<msg::TwistStamped>::SharedPtr mPublisher;
  rclcpp::TimerBase::SharedPtr mWallTimer;
  // rclcpp_lifecycle::LifecyclePublisher
  const rclcpp::ParameterValue *mpTopicName;
  const rclcpp::ParameterValue *mpMarkerId;
  const rclcpp::ParameterValue *mpMarkerTimeout;
  const rclcpp::ParameterValue *mpLinearScalingFactor;
  const rclcpp::ParameterValue *mpMaxAngle;
  const rclcpp::ParameterValue *mpMaxLinearStep;
  const rclcpp::ParameterValue *mpMaxAngularStep;
  rclcpp::Time mLastHitTime;
  std::optional<InitialMarkerReference> mInitialMarkerReference;
};
