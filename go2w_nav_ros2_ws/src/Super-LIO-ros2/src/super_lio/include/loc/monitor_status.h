#ifndef LOC_MONITOR_STATUS_H_
#define LOC_MONITOR_STATUS_H_

#include <optional>
#include <string>
#include <vector>

#include "loc/loc_state.h"

namespace LI2Sup {
namespace loc {

struct MonitorReport {
  std::string state;
  std::vector<std::string> msgs;
  std::string json() const;
};

/// Monitor contract: ready once, succeed on recovery and at 1 Hz. LOST reports
/// error to stop the robot, then terminal failed after a grace period. Frontend
/// failure is terminal too. Initialization exhaustion is initialized_failed;
/// it remains visible during retries. Temporary input outages use error. Times
/// are supplied from a steady clock; sensor timestamps never drive heartbeats.
/// Called only on T0. The backend supplies ordered LocEvents through its queue.
class MonitorStatus {
public:
  explicit MonitorStatus(double input_timeout_s = 2.0, double lost_fail_after_s = 5.0)
      : input_timeout_s_(input_timeout_s), lost_fail_after_s_(lost_fail_after_s) {}

  /// A nonempty synchronized lidar/IMU group was received.
  void inputReceived(double now);
  /// Consume every backend event, even when several arrived between polls.
  std::optional<MonitorReport> update(const LocEvent& event, double now);
  /// Called independently of frontend frames. Only succeed repeats at 1 Hz.
  std::optional<MonitorReport> tick(double now);

private:
  bool ready_ = false;
  bool have_fix_ = false;
  bool failed_ = false;  ///< one-way latch, reset only by recreating the node
  LocState backend_state_ = LocState::kWaitOdom;
  double lost_since_s_ = 0.0;
  double last_input_s_ = 0.0;
  double last_publish_s_ = 0.0;
  double input_timeout_s_;
  double lost_fail_after_s_;
  MonitorReport backend_{"ready", {}};
  std::optional<MonitorReport> last_report_;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_MONITOR_STATUS_H_
