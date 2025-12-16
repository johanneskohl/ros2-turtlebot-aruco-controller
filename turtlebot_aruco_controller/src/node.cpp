#include "node.hpp"

#include <chrono>
namespace cr = std::chrono;
#include <cassert>
#include <algorithm>
#include <cstring>
#include <cmath>

#include <rmw/qos_profiles.h>
#include <rmw/types.h>


#define PRM_ARUCO_DICT      "aruco.dict"
#define PRM_MARKER_ID       "aruco.marker_id"
#define PRM_CAMERA_ID       "camera.id"
#define PRM_TARGET_FPS      "camera.target_fps"
#define PRM_TOPIC           "output_topic"
#define PRM_MARKER_TIMEOUT  "marker_timeout_s"
#define PRM_LIN_SCALE       "linear_scaling_factor"
#define PRM_MAX_ANGLE       "max_input_angle"
#define PRM_MAX_STEP_LIN    "max_step_per_frame.linear"
#define PRM_MAX_STEP_ANG    "max_step_per_frame.angular"

#define TURTLEBOT_MAX_VEL_LIN   0.22
#define TURTLEBOT_MAX_VEL_ANG   2.84
#define ARUCO_DEFAULT_MARKER_ID 75
#define DEFAULT_LINEAR_SCALING  1.0
#define DEFAULT_MARKER_TIMEOUT  5.0

#ifdef TAC_DEBUG
  #include <opencv2/highgui.hpp>
  #include <opencv2/imgproc.hpp>
  #include <sstream>

  #define DEBUG_WINDOW "Turtlebot3 Aruco Controller Debug Window"
  #define RETURN cv::waitKey(10); return
#else
  #define RETURN return
#endif // defined(TAC_DEBUG)

ArucoControllerNode::ArucoControllerNode():
  LifecycleNode("turtlebot_aruco_controller"),
  mLogger(this->get_logger()),
  mInitialMarkerReference({})
{
  //! TODO:
  //! - Parameter descriptions
  //! - pipe through detectMarker options
  //!   (https://docs.opencv.org/3.4/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html#details)
  this->declare_parameter(PRM_ARUCO_DICT, static_cast<int>(cv::aruco::DICT_ARUCO_ORIGINAL));
  this->declare_parameter(PRM_CAMERA_ID,  0);
  this->declare_parameter(PRM_TARGET_FPS, 0.0);

  //! NOTE: these can be converted to const ref members
  mpTopicName = &this->declare_parameter(
    PRM_TOPIC,
    rclcpp::ParameterValue("/cmd_vel")
  );
  mpMarkerId = &this->declare_parameter(
    PRM_MARKER_ID,
    rclcpp::ParameterValue(ARUCO_DEFAULT_MARKER_ID)
  );
  mpMarkerTimeout = &this->declare_parameter(
    PRM_MARKER_TIMEOUT,
    rclcpp::ParameterValue(DEFAULT_MARKER_TIMEOUT)
  );
  mpLinearScalingFactor = &this->declare_parameter(
    PRM_LIN_SCALE,
    rclcpp::ParameterValue(DEFAULT_LINEAR_SCALING)
  );
  mpMaxAngle = &this->declare_parameter(
    PRM_MAX_ANGLE,
    rclcpp::ParameterValue(M_PI_2)
  );
  mpMaxLinearStep = &this->declare_parameter(
    PRM_MAX_STEP_LIN,
    rclcpp::ParameterValue(TURTLEBOT_MAX_VEL_LIN)
  );
  mpMaxAngularStep = &this->declare_parameter(
    PRM_MAX_STEP_ANG,
    rclcpp::ParameterValue(TURTLEBOT_MAX_VEL_ANG)
  );

  mLogger.set_level(rclcpp::Logger::Level::Debug);
}

ArucoControllerNode::CycleCallbackReturn ArucoControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  int dictId = this->get_parameter(PRM_ARUCO_DICT).as_int();
  mDictionary = cv::aruco::getPredefinedDictionary(dictId);
  if (!mDictionary)
  {
    RCLCPP_ERROR_STREAM(mLogger, "Failed to acquire aruco dictionary with id: " << dictId);
    return CycleCallbackReturn::FAILURE;
  }
  mParameters = cv::aruco::DetectorParameters::create();
  if (!mParameters)
  {
    RCLCPP_ERROR(mLogger, "Failed to create DetectorParameters");
    return CycleCallbackReturn::FAILURE;
  }
  rmw_qos_profile_t qosProfile = {
    RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_DEPTH_SYSTEM_DEFAULT,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_DURATION_INFINITE,
    RMW_DURATION_INFINITE,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    RMW_DURATION_INFINITE,
    false
  };
  const std::string &topicName = mpTopicName->get<std::string>();
  mPublisher = this->create_publisher<msg::TwistStamped>(topicName, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qosProfile)));
  if (!mPublisher)
  {
    RCLCPP_ERROR_STREAM(mLogger, "Failed to create Publisher on topic \"" << topicName << "\"");
    return CycleCallbackReturn::FAILURE;
  }

  return CycleCallbackReturn::SUCCESS;
}

ArucoControllerNode::CycleCallbackReturn ArucoControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  mPublisher->on_activate();

  int cameraId = this->get_parameter(PRM_CAMERA_ID).as_int();
  //! NOTE: explicitly selecting video4linux2 backend so it is force loaded before gstreamer
  //!       for more info see: https://github.com/opencv/opencv/issues/23864
  if (!mCapture.open(cameraId, cv::CAP_V4L2))
  {
    RCLCPP_ERROR_STREAM(mLogger, "Failed to open capture device: " << cameraId);
    return CycleCallbackReturn::FAILURE;
  }
  RCLCPP_DEBUG_STREAM(mLogger, "Camera uses " << mCapture.getBackendName() << " backend");

  double fps = mCapture.get(cv::CAP_PROP_FPS);
  if (fps == 0.0)
  {
    RCLCPP_ERROR(mLogger, "Failed to get capture device framerate");
    return CycleCallbackReturn::FAILURE;
  }
  double targetFps = this->get_parameter(PRM_TARGET_FPS).as_double();
  double pollingFps;
  if (targetFps > 0.0 && targetFps < fps)
  {
    pollingFps = 2 * targetFps + 1;
  }
  else
  {
    pollingFps = 2 * fps + 1;
  }
  RCLCPP_DEBUG_STREAM(mLogger, "target fps: " << targetFps << " source fps: " << fps << " resulting polling fps: " << pollingFps);

  cr::milliseconds duration(static_cast<cr::milliseconds::rep>(1000.0 / pollingFps));
  mWallTimer = this->create_wall_timer(
    duration,
    [
      this]() -> void
    {
      this->mainloop();
    }
  );
  if (!mWallTimer)
  {
    RCLCPP_ERROR_STREAM(mLogger, "Failed to create wall timer with duration: " << duration.count() << "ms");
    return CycleCallbackReturn::FAILURE;
  }

#ifdef TAC_DEBUG
  cv::namedWindow(DEBUG_WINDOW);
#endif // defined(TAC_DEBUG)

  return CycleCallbackReturn::SUCCESS;
}

msg::TwistStamped ArucoControllerNode::makeEmptyTwist() const
{
  msg::TwistStamped stampedTwist(rosidl_runtime_cpp::MessageInitialization::ZERO);
  stampedTwist.header.set__frame_id("base_link");
  stampedTwist.header.set__stamp(this->now());

  return stampedTwist;
}

static std::ostream &operator<<(std::ostream &stream, const std::vector<int> &vector)
{
  stream << '{';
  if (!vector.empty())
  {
    auto it = vector.begin();
    auto endIt = vector.end();
    stream << *it;

    for (++it; it != endIt; ++it)
      stream << ", " << *it;
  }
  stream << '}';
  return stream;
}

static const char *cvDepthToName(int depth)
{
  switch (depth)
  {
  case 0:
    return "CV_8U";
  case 1:
    return "CV_8S";
  case 2:
    return "CV_16U";
  case 3:
    return "CV_16S";
  case 4:
    return "CV_32S";
  case 5:
    return "CV_32F";
  case 6:
    return "CV_64F";
  default:
    return "unknown";
  }
}

static double roundTo(double x, int precision)
{
  double factor = std::pow(10.0, precision);
  return std::round(x * factor) / factor;
}

void ArucoControllerNode::mainloop()
{
  cv::Mat image;
  if (!mCapture.read(image))
  {
    RCLCPP_DEBUG(mLogger, "No new image to read. This is expected behaviour for about every second call due to the polling rate.");
    RETURN;
  }
  assert(!image.empty());
  RCLCPP_DEBUG_STREAM(mLogger,
    "Data on captured image:"
    " rows = " << image.rows <<
    " cols = " << image.cols <<
    " nr channels = " << image.channels() <<
    " pixel depth = " << cvDepthToName(image.depth())
  );
# ifdef TAC_DEBUG
  cv::imshow(DEBUG_WINDOW, image);
# endif // TAC_DEBUG

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  try {
    cv::aruco::detectMarkers(image, mDictionary, markerCorners, markerIds, mParameters);
  }
  catch (cv::Exception &error)
  {
    RCLCPP_ERROR_STREAM(mLogger, "Error detecting markers: " << error.what());
    RETURN;
  }
  assert(markerIds.size() == markerCorners.size());

  auto it = std::find(markerIds.begin(), markerIds.end(), mpMarkerId->get<int>());
  if (it == markerIds.end())
  {
    RCLCPP_WARN(mLogger, "Could not find aruco marker in this frame.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(
      mLogger, !markerIds.empty(),
      "Is the marker id parameter correct? Found other markers: " << markerIds
    );

    //! NOTE: publish empty twist so previously drive instructions are not endlessly executed
    if (mInitialMarkerReference.has_value())
    {
      mPublisher->publish(this->makeEmptyTwist());

      double secondsSinceLastHit = (this->now() - mLastHitTime).seconds();
      const double &markerTimeout = mpMarkerTimeout->get<double>();
      RCLCPP_DEBUG_STREAM(mLogger, "time since last detected marker: " << secondsSinceLastHit << "s < " << markerTimeout << "s?");
      if (secondsSinceLastHit > markerTimeout)
      {
        mInitialMarkerReference.reset();
        RCLCPP_INFO_STREAM(mLogger, secondsSinceLastHit << "s since last detected marker, resetting baseline.");
      }
    }

    RETURN;
  }
  std::vector<cv::Point2f> markerCorner = std::move(markerCorners[std::distance(markerIds.begin(), it)]);
  mLastHitTime = this->now();

  cv::Point2f center = std::accumulate(markerCorner.begin(), markerCorner.end(), cv::Point2f()) / 4.0;
  const cv::Point2f &topLeft = markerCorner.front();
  double distanceCornerCenter = cv::norm(center - topLeft);

# ifdef TAC_DEBUG
  cv::Scalar green(0.0, 255.0, 0.0);
  cv::drawMarker(image, center, green);
  cv::drawMarker(image, topLeft, green);
  cv::line(image, center, topLeft, green);
  cv::putText(image, std::to_string(distanceCornerCenter) + "px", (center + topLeft) / 2 + cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, green);
  cv::putText(image, (std::stringstream{} << topLeft).str(), topLeft + cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, green);
  cv::putText(image, (std::stringstream{} << center).str(), center + cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, green);
# endif // TAC_DEBUG

  if (!mInitialMarkerReference.has_value())
  {
    RCLCPP_INFO(mLogger, "Found initial marker position");
    RCLCPP_DEBUG_STREAM(mLogger,
      "Initial marker position data: "
      "center = " << center << ", "
      "top left = " << topLeft << ", "
      "distance = " << distanceCornerCenter
    );
    mInitialMarkerReference = InitialMarkerReference{
      .center = center,
      .topLeft = topLeft,
      .distanceCornerCenter = distanceCornerCenter
    };

#   ifdef TAC_DEBUG
    cv::imshow(DEBUG_WINDOW, image);
#   endif // TAC_DEBUG
    RETURN;
  }
  const InitialMarkerReference &initialMarkerRef = mInitialMarkerReference.value();

# ifdef TAC_DEBUG
  cv::Scalar blue(255.0, 0.0, 0.0);
  cv::drawMarker(image, initialMarkerRef.center, blue);
  cv::drawMarker(image, initialMarkerRef.topLeft, blue);
  cv::line(image, initialMarkerRef.center, initialMarkerRef.topLeft, blue);
  cv::putText(image, std::to_string(initialMarkerRef.distanceCornerCenter) + "px", (initialMarkerRef.center + initialMarkerRef.topLeft) / 2 - cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, blue);
  cv::putText(image, (std::stringstream{} << initialMarkerRef.topLeft).str(), initialMarkerRef.topLeft - cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, blue);
  cv::putText(image, (std::stringstream{} << initialMarkerRef.center).str(), initialMarkerRef.center - cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, blue);
# endif // TAC_DEBUG

  // calculate zoom and rotation
  double zoomScale = distanceCornerCenter / initialMarkerRef.distanceCornerCenter;
  double relativeSpeed = (zoomScale - 1.0) * mpLinearScalingFactor->get<double>(); // convert from value around 1 to value around 0 to get positive and negative values
  relativeSpeed = -1.0 * std::max(std::min(relativeSpeed, 1.0), -1.0); // cap between -1.0 and 1.0 and invert
  RCLCPP_DEBUG_STREAM(mLogger, "zoomScale = " << zoomScale << " relativeSpeed = " << relativeSpeed);

  cv::Point2f offsetVector = initialMarkerRef.center - center;
  cv::Point2f correctedTopLeft = initialMarkerRef.topLeft - offsetVector; // correct initial top left as if it had the current center
  double \
    currentAngle = std::atan2(topLeft.y - center.y, topLeft.x - center.x),
    correctedRefAngle = std::atan2(correctedTopLeft.y - center.y, correctedTopLeft.x - center.x);
  double angle = roundTo(currentAngle - correctedRefAngle, 3);
  double relativeAngle = angle / mpMaxAngle->get<double>();
  relativeAngle = std::max(std::min(relativeAngle, 1.0), -1.0); // cap between -1.0 and 1.0
  RCLCPP_DEBUG_STREAM(mLogger, "initialTopLeft = " << initialMarkerRef.topLeft << "correctedTopLeft = " << correctedTopLeft);
  RCLCPP_DEBUG_STREAM(mLogger, "current angle = " << currentAngle << " corrected reference angle: " << correctedRefAngle);
  RCLCPP_DEBUG_STREAM(mLogger, "angle = " << angle << " relativeAngle = " << relativeAngle);

# ifdef TAC_DEBUG
  std::vector<std::vector<cv::Point>> polygons = {{cv::Point(0, 0), cv::Point(100, 0), cv::Point(100, 20), cv::Point(0, 20)}};
  cv::fillPoly(image, polygons, cv::Scalar(255, 255, 255));
  cv::putText(image, std::to_string(angle), cv::Point(0, 10), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar{});

  cv::Scalar red(0.0, 0.0, 255.0);
  cv::drawMarker(image, correctedTopLeft, red);
  cv::line(image, center, correctedTopLeft, red);
  cv::putText(image, (std::stringstream{} << correctedTopLeft).str(), correctedTopLeft + cv::Point2f(5, -5), cv::FONT_HERSHEY_PLAIN, 1.0, red);

  cv::imshow(DEBUG_WINDOW, image);
# endif //TAC_DEBUG

  msg::TwistStamped stampedTwist = this->makeEmptyTwist();
  stampedTwist.twist.linear.set__x(relativeSpeed * mpMaxLinearStep->get<double>());
  stampedTwist.twist.angular.set__z(relativeAngle * mpMaxAngularStep->get<double>());
  mPublisher->publish(stampedTwist);
  RCLCPP_DEBUG_STREAM(mLogger, "Published Twist msg with linear.x = " << stampedTwist.twist.linear.x << " and angular.z = " << stampedTwist.twist.angular.z);

  RETURN;
}

ArucoControllerNode::CycleCallbackReturn ArucoControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
# ifdef TAC_DEBUG
  if (cv::getWindowProperty(DEBUG_WINDOW, cv::WND_PROP_AUTOSIZE) != -1.0)
    cv::destroyWindow(DEBUG_WINDOW);
# endif // TAC_DEBUG

  mCapture.release();
  if (mWallTimer)
    mWallTimer->cancel();
  mPublisher->publish(this->makeEmptyTwist());
  mPublisher->on_deactivate();

  return CycleCallbackReturn::SUCCESS;
}

ArucoControllerNode::CycleCallbackReturn ArucoControllerNode::cleanupInternal(const rclcpp_lifecycle::State &state)
{
  this->on_deactivate(state);
  mWallTimer.reset();
  mPublisher.reset();
  mParameters.release();
  mDictionary.release();

  return CycleCallbackReturn::SUCCESS;
}
