#ifndef HBOT__VELOCITY_CONTROLLER_PARAMS_H
#define HBOT__VELOCITY_CONTROLLER_PARAMS_H


struct ControllerParams
{
    double KP_LIN;
    double KI_LIN;
    double KD_LIN;

    double KP_Z;
    double KI_Z;
    double KD_Z;

    double YAW_RATE;

    double MAX_LIN_VEL;
    double MAX_Z_VEL;
};

#endif //HBOT__VELOCITY_CONTROLLER_PARAMS_H