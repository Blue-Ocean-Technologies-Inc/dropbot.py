#ifndef ___DROPBOT__SWITCHING_MATRIX__H___
#define ___DROPBOT__SWITCHING_MATRIX__H___

#include <Eigen/Dense>
#include <math.h>
#include "analog.h"
#include "channels.h"
#include "Signal.h"


namespace dropbot {
namespace switching_matrix {

using namespace Eigen;

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
    typedef Matrix<uint8_t, Dynamic, Dynamic> MatrixU8;
    typedef Matrix<float, Dynamic, Dynamic> MatrixF;

    State state_;
    uint32_t row_count_;
    SettlingTime t_settling_;
    SettlingTime t_0_;
    uint8_t row_i_;

    std::array<float, MAX_NUMBER_OF_CHANNELS> sensitive_duty_cycles_;
    std::vector<uint8_t> sensitive_channels_;
    MatrixF m_;
    MatrixF y_;
    MatrixF OMEGA_;

public:
    Signal<std::function<void(// sensitive channels
                              std::vector<uint8_t> const &,
                              // capacitances, i.e., $\vec{y}$
                              MatrixF const &)> > sensitive_capacitances_;

    SwitchingMatrixRowContoller() : state_(IDLE), row_count_(0) {
      sensitive_duty_cycles_.fill(0);
    }
    SwitchingMatrixRowContoller(uint32_t row_count, SettlingTime t_settling)
        : state_(IDLE), row_count_(row_count), t_settling_(t_settling) {
      sensitive_duty_cycles_.fill(0);
    }

    void reset() {
      state_ = IDLE;
    }

    void reset(uint32_t row_count, SettlingTime t_settling=0.005 * 1e6) {
      state_ = IDLE;
      row_count_ = row_count;
      t_settling_ = t_settling;
    }

    template <typename Node>
    void stop(Node &node) {
      node.channels_.turn_off_all_channels();
      state_ = IDLE;
    }

    auto const row_count() const { return row_count_; }
    auto const t_settling() const { return t_settling_; }
    auto const &m() const { return m_; }
    auto S() const {
      auto const channels_count = sensitive_channels_.size();

      // XXX Allocate matrices based on number of sensitive channels.
      auto S_ = MatrixU8(row_count_, channels_count);

      // XXX Fill switching matrix, $S$.
      for (auto j = 0; j < channels_count; j++) {
          auto const d_j = sensitive_duty_cycles_[sensitive_channels_[j]];
          auto const a_j = a(d_j, row_count_);
          for (auto i = 0; i < row_count_; i++) {
              S_(i, j) = c_(i, j, a_j);
          }
      }
      return S_;
    }

    auto OMEGA() const {
      // XXX compute $\Omega = (S^T S)^{-1} S^T$ matrix, where $\vec{y} = \Omega \vec{m}$.
      auto S_ = S();
      MatrixF OMEGA_ = ((S_.transpose().cast<float>() *
                         S_.cast<float>()).cast<float>().inverse() *
                        S_.transpose().cast<float>());
      return OMEGA_;
    }

    auto const &y() const { return y_; }

    auto const &sensitive_duty_cycles() const {
      return sensitive_duty_cycles_;
    }

    template <typename Container>
    void sensitive_duty_cycles(float duty_cycle) {
      sensitive_duty_cycles_.fill(duty_cycle);
    }

    template <typename Container>
    void sensitive_duty_cycles(float duty_cycle, Container channels) {
      sensitive_duty_cycles(duty_cycle, channels.begin(), channels.end());
    }

    template <typename Iterator>
    void sensitive_duty_cycles(float duty_cycle, Iterator channels_begin,
                               Iterator channels_end) {
      for (auto i = channels_begin; i != channels_end; i++) {
        sensitive_duty_cycles_[*i] = duty_cycle;
      }
    }

    auto const &sensitive_channels() const { return sensitive_channels_; }

    template <typename Container>
    void sensitive_channels(Container packed_sensitive_channels) {
      sensitive_channels(packed_sensitive_channels.begin(),
                         packed_sensitive_channels.end());
    }

    template <typename Iterator>
    void sensitive_channels(Iterator packed_begin, Iterator packed_end) {
      sensitive_channels_ = unpack_channels(packed_begin, packed_end);
    }

    /**
     * @brief  Update the state of the switching matrix controller.
     *
     * @tparam Node  DropBot control node class type.
     * @param node  Reference to DropBot control node object.
     * @param event  Code associated with event to trigger.
     * @param t  Settling time to wait after applying settings of a row, e.g.,
     *   before measuring capacitance.
     * @param measure_C  Measure capacitance for each row in the switching
     *   matrix.
     * @param send_signal  Send signal after completion of each cycle through
     *   the switching matrix.
     *
     * @return  Current row index in the switching matrix.
     *
     * Notes
     * -----
     *
     * The `measure_C` and `send_signal` parameters were introduced to isolate
     * the run time associated with each part of the update during performance
     * benchmarking: **a)** updating the state machine; **b)** computing
     * capacitance from ADC readings; **c)** processing callbacks connected to
     * the signal indicating a new set of sensitive capacitances are available.
     */
    template <typename Node>
    auto update(Node &node, Event event, SettlingTime t, bool measure_C=true,
                bool send_signal=true) {
      switch (state_) {
        case IDLE:
          if (event == START && row_count_ > 0) {
            row_i_ = 0;
            m_ = MatrixF(row_count_, 1);
            m_.setZero();
            OMEGA_ = OMEGA();
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
            if (row_i_ < m_.size()) {
              // XXX measure capacitance and store result in $\vec{m}$.
              if (measure_C) {
                m_(row_i_, 0) = node.capacitance(0);
              } else {
                m_(row_i_, 0) = 0;
              }
            }
            row_i_ += 1;
            if (row_i_ >= row_count_) {
              y_ = OMEGA_ * m_;
              if (send_signal) {
                sensitive_capacitances_.send(sensitive_channels_, y_);
              }
              row_i_ = 0;
            }
            state_ = SETTING_SWITCHING_MATRIX_ROW;
          }
          break;
        default:
          stop(node);
          break;
      }
      return row_i_;
    }
};

}  // namespace switching_matrix {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SWITCHING_MATRIX__H___
