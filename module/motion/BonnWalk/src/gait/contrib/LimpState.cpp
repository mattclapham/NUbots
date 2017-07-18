#include "LimpState.h"

namespace gait {
namespace contrib {

    LimpState operator*(const double scalar, const LimpState& v) {
        LimpState lm;
        lm    = v;
        lm.x  = scalar * v.x;
        lm.vx = scalar * v.vx;
        lm.y  = scalar * v.y;
        lm.vy = scalar * v.vy;
        return lm;
    }

    LimpState operator*(const LimpState& v, const double scalar) {
        return scalar * v;
    }

}  // namespace contrib
}  // namespace gait
