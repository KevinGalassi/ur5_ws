// this header incorporates all the necessary #include files and defines the
// class "ExampleRosClass"
#include <move_rt/emergencystop.hpp>

static bool setsEqual(const std::vector<std::string> &a,
                      const std::vector<std::string> &b) {
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i) {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }
  for (size_t i = 0; i < b.size(); ++i) {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

EmergencyStop::EmergencyStop(ros::NodeHandle &n,
                             Eigen::Matrix<long double, Eigen::Dynamic, 1> *_q)
    : PriorityLevel(n), q(_q) {

  ROS_DEBUG_STREAM("EmergencyStop::EmergencyStop: enter contructor ");

  PriorityLevel::update_task();

  JK.setIdentity();

  A.setIdentity();
  enabled = true;

  position_initialized = false;

  ROS_DEBUG_STREAM("EmergencyStop::EmergencyStop: contructor finished");
}

EmergencyStop::~EmergencyStop() {}

void EmergencyStop::updateConstraints() {
  for (int i = 0; i < m; ++i)
    x(i, 0) = q->operator()(i, 0);

  if (!position_initialized) {
    for (int i = 0; i < m; ++i) {
      xd(i, 0) = q->operator()(i, 0);
    }
    if (initialized)
      position_initialized = true;
  }

  if (!enabled)
    for (int i = 0; i < m; ++i)
      xd(i, 0) = q->operator()(i, 0);
}
