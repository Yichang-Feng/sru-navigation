#include "loc/monitor_status.h"

namespace LI2Sup {
namespace loc {
namespace {

std::string quoted(const std::string& value) {
  std::string out = "\"";
  constexpr char hex[] = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += static_cast<char>(c);
    } else if (c < 0x20) {
      out += "\\u00";
      out += hex[c >> 4];
      out += hex[c & 15];
    } else {
      out += static_cast<char>(c);
    }
  }
  return out + '"';
}

}  // namespace

std::string MonitorReport::json() const {
  std::string out = "{\"state\":" + quoted(state) + ",\"msgs\":[";
  for (std::size_t i = 0; i < msgs.size(); ++i) {
    if (i) out += ',';
    out += quoted(msgs[i]);
  }
  return out + "]}";
}

void MonitorStatus::inputReceived(double now) {
  ready_ = true;
  last_input_s_ = now;
}

std::optional<MonitorReport> MonitorStatus::update(const LocEvent& event, double now) {
  if (failed_) return std::nullopt;
  const LocState previous = backend_state_;
  backend_state_ = event.state;
  switch (event.state) {
    case LocState::kTracking:
      have_fix_ = true;
      backend_ = {"succeed", {}};
      break;
    case LocState::kLost:
      if (previous != LocState::kLost) lost_since_s_ = now;
      backend_ = {"error", event.reasons};
      break;
    case LocState::kFailed:
      failed_ = true;
      backend_ = {"failed", event.reasons};
      break;
    case LocState::kInitFailed:
      backend_ = {"initialized_failed", {"localization_init_failed", kInitFailedHint}};
      break;
    case LocState::kInitializing:
      // Initialization failure stays visible throughout retries; only a
      // confirmed fix clears it. No intermediate error event is emitted.
      if (backend_.state == "initialized_failed") break;
      if (have_fix_ || backend_.state == "error") {
        backend_ = {"error", {"localization_reinitializing"}};
      }
      break;
    case LocState::kWaitInit:
      if (!event.reasons.empty()) {
        backend_ = {"error", event.reasons};
      } else if (previous == LocState::kFailed || have_fix_) {
        backend_ = {"error", {"localization_waiting_init"}};
      }
      break;
    case LocState::kWaitOdom:
      break;
  }
  return tick(now);
}

std::optional<MonitorReport> MonitorStatus::tick(double now) {
  if (!ready_ && !failed_ && backend_.state != "initialized_failed") return std::nullopt;
  MonitorReport report = backend_;
  if (!failed_ && backend_state_ == LocState::kLost && now - lost_since_s_ >= lost_fail_after_s_) {
    failed_ = true;
    report.state = "failed";
    report.msgs.emplace_back("lost_duration_exceeded");
    backend_ = report;
  }
  // An input outage is recoverable. Preserve LOST or initialization failure
  // reasons when it overlaps another failure, and never repeat stale succeed.
  if (!failed_ && ready_ && now - last_input_s_ > input_timeout_s_) {
    if (report.state == "succeed" || report.state == "ready") report = {"error", {}};
    report.msgs.emplace_back("sensor_data_timeout");
  }
  const bool changed = !last_report_ || report.state != last_report_->state ||
                       report.msgs != last_report_->msgs;
  if (!changed && (report.state != "succeed" || now - last_publish_s_ < 1.0)) {
    return std::nullopt;
  }
  last_report_ = report;
  last_publish_s_ = now;
  return report;
}

}  // namespace loc
}  // namespace LI2Sup
