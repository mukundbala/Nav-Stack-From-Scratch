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
    FOLLOW //FOLLOW, TAKEOFF and LAND apply to solo missions
};
            
enum class GoalState : unsigned short
{
    PREDICTION,
    CHASE,
    GOTO
};

//converting Hector State into a string
std::string_view unpack_h_state(HectorState state);
//converting Goal State into a string
std::string_view unpack_g_state(GoalState state);

}

#endif //HBOT__MISSION_STATES_H