#ifndef ___DROPBOT__SWITCHING_MATRIX__H___
#define ___DROPBOT__SWITCHING_MATRIX__H___

#include <math.h>


namespace dropbot {
namespace switching_matrix {


inline int a(float d, int p) {
  /*
   * Returns
   * -------
   * int
   *     Number of active windows in period `p` given duty cycle `d`.
   */
  int a_ = round(d * p);
  if (a_ >= p) {
    return p - 1;
  } else if (a_ < 1) {
    return 1;
  }
  return a_;
}


template <typename T>
T c_(T i, T j, T a_j) {
  /*
   * Returns
   * -------
   * T
   *     Actuation state of the $j^{th}$ sensitive channel during window $i$,
   *     given the number of active windows $a_j$.
   */
  if (i == j) {
    return (a_j < j) ? 1 : 0;
  } else if ((j <= a_j) && (i <= a_j)) {
    return 1;
  } else if (i >= a_j - 1) {
    return 0;
  } else {
    return 1;
  }
  return 0;
}

}  // namespace switching_matrix {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SWITCHING_MATRIX__H___
