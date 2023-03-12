#include "mission_states.h"


std::string_view mission_states::unpack_h_state(HectorState state)
{
    std::array<std::string_view,5> states = {"TAKEOFF" , "LAND" , "TURTLE" , "START" , "GOAL"};
    return states.at(static_cast<unsigned short> (state));
}

//converting Goal State into a string
std::string_view mission_states::unpack_g_state(GoalState state)
{
    std::array<std::string_view,3> states = {"PREDICTION" , "CHASE" , "GOTO"};
    return states.at(static_cast<unsigned short> (state));
}
