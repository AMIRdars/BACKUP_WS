/*
 * 順運動学、逆運動学の計算に関する定義
 *
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

// #include <Arduino.h>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)


class DOF5Kinematic
{
private:

    struct ArmConfig_t
    {
        float L_BASE;
        float L_ARM;
        float L_FOREARM;
        float L_GRIPPER;
    };
    ArmConfig_t armConfig;

    float DH_matrix[5][4] = { { 0.0f } }; // theta,d,a,alpha

public:
    struct Joint5D_t
    {
        Joint5D_t() = default;

        Joint5D_t(float a1, float a2, float a3, float a4, float a5)
            : a{ a1, a2, a3, a4, a5 }
        {}

        float a[5];

        friend Joint5D_t operator-(const Joint5D_t& _joints1, const Joint5D_t& _joints2);
        friend Joint5D_t operator+(const Joint5D_t& _joints1, const Joint5D_t& _joints2);
    };

    struct Pos5D_t
    {
        Pos5D_t() = default;

        Pos5D_t(float x, float y, float z, float roll, float pitch, float yaw)
            : X(x), Y(y), Z(z), RotX(roll), RotY(pitch), RotZ(yaw)
        {}

        float X{}, Y{}, Z{};
        float RotX{}, RotY{}, RotZ{};
    };

    struct IKSolves_t
    {
        Joint5D_t config[4];
    };

    DOF5Kinematic(float L1, float L2, float L3, float L4);

    bool solveFK(const DOF5Kinematic::Joint5D_t& _inputJoint5D, DOF5Kinematic::Pos5D_t& _outputPos5D);
    bool solveIK(const DOF5Kinematic::Pos5D_t& _inputPos5D, DOF5Kinematic::IKSolves_t& _outputSolves);

};

const DOF5Kinematic::Joint5D_t initialConfig_MRAD = { 170.0f * DEG_TO_RAD * 1000, 
                                                      135.0f * DEG_TO_RAD * 1000, 
                                                     -160.0f * DEG_TO_RAD * 1000, 
                                                      -15.0f * DEG_TO_RAD * 1000, 
                                                     -158.0f * DEG_TO_RAD * 1000 
                                                    };
const DOF5Kinematic::Joint5D_t initialConfig_RAD = { 170.0f * DEG_TO_RAD, 
                                                     135.0f * DEG_TO_RAD, 
                                                    -160.0f * DEG_TO_RAD, 
                                                     -15.0f * DEG_TO_RAD, 
                                                    -158.0f * DEG_TO_RAD 
                                                    };

#endif //KINEMATICS_H