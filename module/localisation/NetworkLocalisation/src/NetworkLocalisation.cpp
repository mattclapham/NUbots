#include "NetworkLocalisation.h"

#include "extension/Configuration.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    NetworkLocalisation::NetworkLocalisation(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("NetworkLocalisation.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file NetworkLocalisation.yaml
        });
    }
}
}
