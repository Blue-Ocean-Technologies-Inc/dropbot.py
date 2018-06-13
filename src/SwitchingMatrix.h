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


// http://www.plantuml.com/plantuml/png/fPB1JuCm5CRl_1Nl26h3k9gnpEpaOJ61tSGj3NHRar8cVRWPql_T5c4G7S742osFz_q--lKj5T8IgrmupklT6n2oWETCCCS-chUO8N9vYCySqwFPh2cM_18LPtY4s6hzkQ38BWiUpXu0TxCGv_vtfOVOJTDVd7pYZ28d3Lih16AT815erbePXJ2TkZEUS5WpgghIb5VqH5EELAQiXm-1ne6zGsjtfIiqwmGb-M1fdMkNUn12OYfOvus09Rkq5sZNwx29l0Rfyll0QqI8DUH51ZhwrMO32A4F--R__35GtaGP0O6Wu_dadyJm1syOpASsdzruku7OszOaq63tRMRJ9zfKfJJbb_t-IOZLaKh9X6hdQ_pk9wNjnOwr30-Wx07wYRWf6Ku-CN4wIoQpAXTV
class SwitchingMatrixRowContoller {
public:
    enum Event {
        START,
        TICK,
        STOP,
    };
private:
    enum State {
        IDLE,
        SETTING_SWITCHING_MATRIX_ROW,
        WAITING_FOR_SETTLING,
        CALLING_STOPPED_CALLBACK
    };

    typedef uint32_t SettlingTime;

    State state_;
    uint32_t row_count_;
    SettlingTime t_settling_;
    SettlingTime t_0_;
    uint8_t row_i_;

public:
    SwitchingMatrixRowContoller() : state_(IDLE), row_count_(0) {}
    SwitchingMatrixRowContoller(uint32_t row_count, SettlingTime t_settling)
        : state_(IDLE), row_count_(row_count), t_settling_(t_settling) {}

    template <typename Node>
    void stop(Node &node) {
      node.channels_.turn_off_all_channels();
      state_ = IDLE;
    }

    template <typename Node>
    void update(Node &node, Event event, SettlingTime t) {
      switch (state_) {
        case IDLE:
          if (event == START && row_count_ > 0) {
            row_i_ = 0;
            state_ = SETTING_SWITCHING_MATRIX_ROW;
          } else {
            break;
          }
        case SETTING_SWITCHING_MATRIX_ROW:
          if (event == STOP) {
            stop(node);
          } else if(event == TICK) {
            node.set_switching_matrix_row(row_i_, row_count_);
            t_0_ = t;
            state_ = WAITING_FOR_SETTLING;
          }
          break;
        case WAITING_FOR_SETTLING:
          if (event == STOP) {
            stop(node);
          } else if (event == TICK && ((t - t_0_) >= t_settling_)) {
            // Settling time has passed.
            // TODO: measure capacitance and store result in $M$ vector.
            row_i_ += 1;
            if (row_i_ >= row_count_) {
              // TODO compute $\vec{y} = (S^T S)^{-1} S^T \vec{m}$
              row_i_ = 0;
            }
            state_ = SETTING_SWITCHING_MATRIX_ROW;
          }
          break;
        default:
          stop(node);
          break;
      }
    }
};

}  // namespace switching_matrix {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SWITCHING_MATRIX__H___
