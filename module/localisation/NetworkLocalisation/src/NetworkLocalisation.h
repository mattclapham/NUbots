#ifndef MODULE_LOCALISATION_NETWORKLOCALISATION_H
#define MODULE_LOCALISATION_NETWORKLOCALISATION_H

#include <nuclear>

namespace module {
namespace localisation {

    class NetworkLocalisation : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the NetworkLocalisation reactor.
        explicit NetworkLocalisation(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_NETWORKLOCALISATION_H
