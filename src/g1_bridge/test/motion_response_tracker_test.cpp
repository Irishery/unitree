#include <cassert>
#include <chrono>
#include "../src/motion_response_tracker.hpp"

int main() {
  using namespace std::chrono_literals;
  MotionResponseTracker tracker(5.0);
  const auto start = MotionResponseTracker::Time{};
  tracker.sent(10, false, start);
  tracker.sent(11, true, start);
  assert(!tracker.response(99, 7105, 0));  // Foreign ID.
  assert(!tracker.response(11, 7001, 0));  // Wrong API, same ID.
  assert(tracker.pending() == 2);
  assert(tracker.response(10, 7105, 0)->nonzero == false);
  assert(!tracker.response(10, 7105, 0));  // Duplicate cannot count twice.
  assert(tracker.response(11, 7105, 7302)->nonzero);
  assert(tracker.accepted == 1 && tracker.nonzero_accepted == 0);
  assert(tracker.rejected == 1 && tracker.nonzero_rejected == 1);
  assert(tracker.last_response_code == 7302);
  tracker.sent(12, true, start);
  assert(tracker.expire(start + 4999ms).empty());
  assert(tracker.expire(start + 5s).size() == 1);
  assert(!tracker.response(12, 7105, 0));  // Late response is not success.
  assert(tracker.timeouts == 1 && tracker.nonzero_timeouts == 1);
  assert(tracker.last_timeout_id == 12);
  assert(tracker.expire(start + 6s).empty());
  tracker.sent(13, true, start + 6s);
  assert(tracker.response(13, 7105, 0));
  assert(tracker.nonzero_accepted == 1 && tracker.responses == 3);
  assert(tracker.last_rejection_id == 11 && tracker.last_rejection_code == 7302);
  assert(tracker.requests == 4 && tracker.nonzero_requests == 3);
  assert(tracker.pending() == 0);
  // At 100 Hz, expiration keeps storage bounded; no resets on disarm.
  for (int i = 0; i < 10000; ++i) {
    const auto now = start + 10s + i * 10ms;
    tracker.expire(now);
    tracker.sent(100 + i, true, now);
    assert(tracker.pending() <= 500);
  }
}
