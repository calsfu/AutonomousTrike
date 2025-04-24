#pragma once

namespace trike {
    enum Mode {
        PARK,
        NEUTRAL,
        MANUAL,
        AUTONOMOUS
    };

    enum AudioFile {
        AUTONOMOUS_ON,
        BRAKES_OFF,
        BRAKES_ON,
        DESTINATION_CONFIRMED,
        DESTINATION_SET,
        MANUAL_ON,
        NEUTRAL_ON,
        PARK_ON,
        NO_GPS_SIGNAL,
        ENTER_DESTINATION,
        SYSTEM_READY,
        TURNING_LEFT,
        TURNING_RIGHT,
        CONFIRM_DESTINATION,
    };
};