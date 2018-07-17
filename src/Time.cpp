#include <Arduino.h>
#include "Time.h"


namespace dropbot {
namespace time {

dropbot::time::TimeSync time_sync_ = {0, 0};

uint32_t sync_time(double wall_time) {
  time_sync_.wall_time = wall_time;
  time_sync_.local_ms = millis();
  return time_sync_.local_ms;
}

double wall_time() {
  return time_sync_.wall_time + 1e-3 * (millis() - time_sync_.local_ms);
}

}  // namespace time {
}  // namespace dropbot {
