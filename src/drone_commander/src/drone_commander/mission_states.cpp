#include "mission_states.h"

//converting Hector State into a string
std::string_view mission_states::unpack_h_state(HectorState state)
{
    std::array<std::string_view,7> states = {"TAKEOFF" , "LAND" , "TURTLE" , "START" , "GOAL", "FOLLOW" , "HOME"};
    return states.at(static_cast<unsigned short> (state));
}


