#ifndef ___DROPBOT__SWITCHING_MATRIX_EIGEN__H___
#define ___DROPBOT__SWITCHING_MATRIX_EIGEN__H___

#include <Eigen/Dense>
#include "SwitchingMatrix.h"


namespace dropbot {
namespace switching_matrix {

template <typename Levels, typename T>
Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> switching_matrix(Levels const
                                                                  &levels,
                                                                  T p) {
    // Returns an Eigen switching matrix.
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> S(p, levels.size());

    for (auto j = 0; j < S.cols(); j++) {
        auto const d_j = levels[j];
        auto const a_j = a(d_j, p);
        for (auto i = 0; i < S.rows(); i++) {
            S(i, j) = c_(i, j, a_j);
        }
    }
    return S;
}

}  // namespace switching_matrix {
}  // namespace dropbot {

#endif  // #ifndef ___DROPBOT__SWITCHING_MATRIX__H___
