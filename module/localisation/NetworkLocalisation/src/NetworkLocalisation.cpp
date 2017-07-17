#include "NetworkLocalisation.h"

#include "extension/Configuration.h"
#include "message/localisation/Ball.h"
#include "message/localisation/Field.h"
#include "message/localisation/NetworkBall.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::localisation::Ball;
    using message::localisation::Field;
    using message::localisation::NetworkBall;

    NetworkLocalisation::NetworkLocalisation(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<Ball>, With<Field>>().then([this](const Ball& ball, const Field& field) {

            auto network_ball = std::make_unique<NetworkBall>();

            // Get the ball position relative to the field (in world space)
            Eigen::Vector2d rBWw = ball.position;
            Eigen::Vector2d rFWw = field.position.head<2>();
            Eigen::Vector2d rBFw = rBWw - rFWw;

            // Rotate this vector into field space
            double cs            = std::cos(field.position[2]);
            double sn            = std::sin(field.position[2]);
            Eigen::Vector2d rBFf = Eigen::Vector2d(rBFw[0] * cs - rBFw[1] * sn, rBFw[0] * sn + rBFw[1] * cs);

            network_ball->position = rBFf;
            // TODO Work out how to include the field covariance, and also how to rotate the covariance matrix
            network_ball->covariance = ball.covariance;

            // Send the packet over the network
            emit<Scope::NETWORK>(network_ball);
        });

        on<Network<NetworkBall>>().then([this](const NetworkSource& source, const NetworkBall& ball) {
            // TODO send this ball somewhere useful
            log("Received network ball from ");
        });
    }
}  // namespace localisation
}  // namespace module
