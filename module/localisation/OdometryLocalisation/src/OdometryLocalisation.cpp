#include "OdometryLocalisation.h"

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/localisation/FieldObject.h"

#include "message/behaviour/Nod.h"
#include "message/platform/darwin/DarwinSensors.h"
#include "utility/math/matrix/Transform3D.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Self;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;

    OdometryLocalisation::OdometryLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Configuration>("OdometryLocalisation.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OdometryLocalisation.yaml
            localisationOffset = config["localisationOffset"].as<arma::vec>();
        });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLocalisation>>().then(
            [this](const Sensors& sensors) {
                NUClear::log("Localisation Orientation reset. This direction is now forward.");
                emit(std::make_unique<Nod>(true));
                Transform2D Trw    = Transform3D(sensors.world).projectTo2D();
                localisationOffset = Trw;
            });


        on<Trigger<Sensors>, Sync<OdometryLocalisation>, Single>().then("Odometry Loc", [this](const Sensors& sensors) {

            Transform2D Trw = Transform3D(sensors.world).projectTo2D();
            Transform2D Twr = Trw.i();

            if (Twr.localToWorld(Trw).norm() > 0.00001) {
                log("Twr.localToWorld(Trw).norm()",
                    Trw.transpose(),
                    Twr.transpose(),
                    Twr.localToWorld(Trw).transpose());
            }
            if (Trw.localToWorld(Twr).norm() > 0.00001) {
                log("Trw.localToWorld(Twr).norm()",
                    Trw.transpose(),
                    Twr.transpose(),
                    Trw.localToWorld(Twr).transpose());
            }

            Transform2D state = localisationOffset.localToWorld(Twr);

            auto selfs = std::make_unique<std::vector<Self>>();
            selfs->push_back(Self());
            selfs->back().locObject.position = state.xy();
            selfs->back().heading            = Eigen::Vector2d(std::cos(state.angle()), std::sin(state.angle()));
            // log("sensors world", state.t());
            // log("offset", localisationOffset.t());
            // log("world", Twr.t());
            emit(selfs);
        });
    }
}  // namespace localisation
}  // namespace module
