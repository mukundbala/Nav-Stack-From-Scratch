#ifndef HBOT__MISSION_STATES_H
#define HBOT__MISSION_STATES_H

#include <array>
#include <string_view>

namespace mission_states
{

enum class HectorState : unsigned short
{
    TAKEOFF,
    LAND,
    TURTLE,
    START,
    GOAL,
    FOLLOW, //FOLLOW, TAKEOFF , LAND , HOME apply to ONLY solo_flights
    HOME
};
            
//converting Hector State into a string
std::string_view unpack_h_state(HectorState state);

}

#endif //HBOT__MISSION_STATES_H