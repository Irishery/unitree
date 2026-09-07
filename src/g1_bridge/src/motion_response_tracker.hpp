#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

// Single-executor, bounded-by-deadline bookkeeping. Never sends commands.
class MotionResponseTracker {
 public:
  using Time = std::chrono::steady_clock::time_point;
  struct Pending {
    int64_t id;
    bool nonzero;
    Time sent;
  };
  explicit MotionResponseTracker(double timeout_s) : timeout_(timeout_s) {}

  void sent(int64_t id, bool nonzero, Time now) {
    pending_.emplace(id, Pending{id, nonzero, now});
    ++requests;
    nonzero_requests += nonzero;
  }

  std::vector<Pending> expire(Time now) {
    std::vector<Pending> expired;
    for (auto it = pending_.begin(); it != pending_.end();) {
      if (now - it->second.sent >= timeout_) {
        expired.push_back(it->second);
        ++timeouts;
        nonzero_timeouts += it->second.nonzero;
        last_timeout_id = it->first;
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    return expired;
  }

  std::optional<Pending> response(int64_t id, int32_t api_id, int32_t code) {
    const auto it = pending_.find(id);
    if (api_id != 7105 || it == pending_.end()) {
      return std::nullopt;  // Foreign, duplicate or expired response.
    }
    const auto request = it->second;
    pending_.erase(it);
    ++responses;
    if (code == 0) {
      ++accepted;
      nonzero_accepted += request.nonzero;
    } else {
      ++rejected;
      nonzero_rejected += request.nonzero;
      last_rejection_id = id;
      last_rejection_code = code;
    }
    last_response_id = id;
    last_response_code = code;
    return request;
  }

  std::size_t pending() const { return pending_.size(); }
  uint64_t requests{0}, nonzero_requests{0}, responses{0};
  uint64_t accepted{0}, rejected{0}, timeouts{0};
  uint64_t nonzero_accepted{0}, nonzero_rejected{0}, nonzero_timeouts{0};
  int64_t last_response_id{0}, last_timeout_id{0}, last_rejection_id{0};
  int32_t last_response_code{0}, last_rejection_code{0};

 private:
  std::chrono::duration<double> timeout_;
  std::map<int64_t, Pending> pending_;
};
