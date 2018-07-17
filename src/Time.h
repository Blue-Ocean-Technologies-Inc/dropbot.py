#ifndef ___DROPBOT__TIME__H___
#define ___DROPBOT__TIME__H___

#include <stdint.h>


namespace dropbot {
namespace time {

struct TimeSync {
  double wall_time;
  uint32_t local_ms;
};

extern TimeSync time_sync_;

/**
* @brief Synchronize specified timestamp to current milliseconds counter.
*
* @param wall_time  Unix timestamp, i.e., seconds since the epoch; January 1,
* 1970 00:00:00.
*
* @return  Milliseconds count synced against.
*
* \See wall_time()
*/
uint32_t sync_time(double wall_time);

/**
* @brief Return last synchronized time plus the number of seconds since time
* was synced.
*
* @return  Number of seconds since time was last synchronized plus the
* synchronized time.
*
* \See sync_time()
*/
double wall_time();

}  // namespace time {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__TIME__H___
