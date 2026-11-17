#ifndef LOC_STATE_H_
#define LOC_STATE_H_

#include <cstdint>
#include <string>
#include <vector>

namespace LI2Sup {
namespace loc {

enum class LocState : std::uint8_t {
  kWaitOdom = 0,
  kWaitInit = 1,
  kInitializing = 2,
  kTracking = 3,
  /// Map matching lost; frontend odometry still supports dead reckoning.
  kLost = 4,
  /// Terminal frontend failure or exhausted LOST grace period. Localization
  /// pose/TF outputs stop; only restarting the node can leave this state.
  kFailed = 5,
  /// Search budget exhausted. Wait for a new guess or sufficient movement
  /// before trying another bounded initialization round.
  kInitFailed = 6
};

const char* toString(LocState s);

inline constexpr const char* kInitFailedHint =
    "初始化失败，请重新提供初始定位，或移动机器人到结构丰富的场景重新初始化";

/// Ordered state transitions, independent of ROS. Keeping events, rather than
/// polling only the latest snapshot, preserves a short-lived FAILED or LOST.
struct LocEvent {
  LocState state;
  std::vector<std::string> reasons;
};

}  // namespace loc
}  // namespace LI2Sup

#endif  // LOC_STATE_H_
