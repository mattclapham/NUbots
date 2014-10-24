#ifndef MESSAGES_ROBOTX_UNDERWATERPINGER_H
#define MESSAGES_ROBOTX_UNDERWATERPINGER_H

namespace messages {
    namespace robotx {

        struct UnderwaterPinger {

            std::string colour;
            double latitude;
            double longitude;
            double depth;

        };
    }
}

#endif // MESSAGES_ROBOTX_CONTROLREFERENCE_H
