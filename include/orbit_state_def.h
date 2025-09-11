#ifndef NOVA_STATE_DEF_H
#define NOVA_STATE_DEF_H

#include <Arduino.h>

namespace nova {
    enum class state_t : uint8_t {
        STARTUP = 0,
        IDLE_SAFE,
        ARMED,
        PAD_PREOP,
        POWERED,
        COASTING,
        DROGUE_DEPLOY,
        DROGUE_DESCEND,
        MAIN_DEPLOY,
        MAIN_DESCEND,
        LANDED,
        RECOVERED_SAFE
    };

    inline const char *state_string(const state_t state) {
        switch (state) {
            case state_t::STARTUP:
                return "STARTUP";
            case state_t::IDLE_SAFE:
                return "IDLE_SAFE";
            case state_t::ARMED:
                return "ARMED";
            case state_t::PAD_PREOP:
                return "PAD_PREOP";
            case state_t::POWERED:
                return "POWERED";
            case state_t::COASTING:
                return "COASTING";
            case state_t::DROGUE_DEPLOY:
                return "DROG_DEPL";
            case state_t::DROGUE_DESCEND:
                return "DROG_DESC";
            case state_t::MAIN_DEPLOY:
                return "MAIN_DEPL";
            case state_t::MAIN_DESCEND:
                return "MAIN_DESC";
            case state_t::LANDED:
                return "LANDED";
            case state_t::RECOVERED_SAFE:
                return "REC_SAFE";
            default:
                __builtin_unreachable();
        }
    }

    enum class pyro_state_t : uint8_t {
        DISARMED = 0,
        ARMED,
        FIRING,
        FIRED
    };

    inline const char *pyro_state_string(const pyro_state_t state) {
        switch (state) {
            case pyro_state_t::DISARMED:
                return "D";
            case pyro_state_t::ARMED:
                return "A";
            case pyro_state_t::FIRING:
                return "F";
            case pyro_state_t::FIRED:
                return "X";
            default:
                __builtin_unreachable();
        }
    }
}  // namespace luna

#endif  //LUNA_STATE_DEF_H
