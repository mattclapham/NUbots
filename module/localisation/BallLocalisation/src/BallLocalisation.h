#ifndef MODULE_LOCALISATION_BALLLOCALISATION_H
#define MODULE_LOCALISATION_BALLLOCALISATION_H

#include <nuclear>

#include "utility/math/filter/UKF.h"
#include "BallModel.h"

namespace module {
namespace localisation {

    class BallLocalisation : public NUClear::Reactor {
    private:
        utility::math::filter::UKF<BallModel> filter;
    public:
        /// @brief Called by the powerplant to build and setup the BallLocalisation reactor.
        explicit BallLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_BALLLOCALISATION_H
