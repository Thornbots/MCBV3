#pragma once
#include "util/Pose2d.hpp"

namespace subsystems {
// REMEMBER THAT YOU USE NEGATIVE FOR INERTIAL TO LOCAL AND POSITIVE FOR LOCAL TO INERTIAL
//these are for 3508 
constexpr float ROOT_2 = 1.414;            // sqrt 2
constexpr float KB = 0.02361;       // V-s/rad backemf
constexpr float KT = 0.02299;       // N-m/A torque  constexprant
constexpr float RA = 0.5592;        // ohm, armature resistance

constexpr float VOLT_MAX = 24;      // V, maximum        

constexpr float MAX_POS_VEL = 1;                                                                                 

#if defined(HERO)
// START getters and setters
constexpr float TRACKWIDTH = 0.49739;      // in m. We need to measure
constexpr float M = 18.774;                  // robot mass kg
constexpr float J = 0.44;                  // measured from sys id kg-m^2
constexpr float R_WHEEL = 0.048 / ROOT_2;  // wheel radius m
constexpr float J_WHEEL = 0.0009;          // wheel moment of inertia kg-m^2
constexpr float C_MOTOR = 2.5e-4;          // motor damping kg-s/m^2`
constexpr float UK_MOTOR = 0.14;           // motor dry friction N-m
constexpr float COF_WHEEL = 0.9;           // unitless COF


constexpr float GEAR_RATIO = 13.7;  // gear ratio
constexpr float P_IDLE = 3;         // W, idle power
constexpr float P_FOS = 0.87;       // unitless, power factor of safety

// Tunable Parameters
constexpr float KP_V_XY = 1500;  // proportional gain for velocity
constexpr float KP_V_ROT = 30;   // proportional gain for rotational velocity

constexpr float KI_V = 0;  // integral gain for velocity

constexpr float IV_MAX = 120;  // maximum integral term for velocity control

constexpr float KP = 0;              // proportional gain for position control
constexpr float BEYBLADE_DELAY = .25;  // delay for beyblade mode/
constexpr float BBterm1 = 10.7717; //constant term
constexpr float BBterm2 = -2.0342; //linear term
constexpr float BBterm3 = -.5660; //quadratic term
constexpr float BBmax = 10.5; //constant term
constexpr float maxTorqueZ = 30;

#elif defined(SENTRY)
// START getters and setters
constexpr float TRACKWIDTH = 0.50038;      // in m
constexpr float M = 18.186;                  // robot mass kg
constexpr float J = 0.44;                  // measured from sys id kg-m^2
constexpr float R_WHEEL = 0.05;  // wheel radius m
constexpr float J_WHEEL = 0.0009;          // wheel moment of inertia kg-m^2
constexpr float C_MOTOR = 2.5e-4 * .02299;          // motor damping kg-s/m^2`
constexpr float UK_MOTOR = 0.14 * .02299;           // motor dry friction N-m
constexpr float COF_WHEEL = 0.9;           // unitless COF

constexpr float GEAR_RATIO = 3591/187.0f;  // gear ratio
constexpr float P_IDLE = 3;         // W, idle power
constexpr float P_FOS = 0.65;       // unitless, power factor of safety

// Tunable Parameters
constexpr float KP_V_XY = 1000;  // proportional gain for velocity
constexpr float KP_V_ROT = 30;   // proportional gain for rotational velocity

constexpr float KI_V = 0;  // integral gain for velocity

constexpr float IV_MAX = 120;  // maximum integral term for velocity control

constexpr float KP = 3;              // proportional gain for position control
constexpr float BEYBLADE_DELAY = .25 ;  // delay for beyblade mode/
constexpr float BBterm1 = 9; //constant term
constexpr float BBterm2 = -2.8647; //linear term
constexpr float BBterm3 = -.3593; //quadratic term
constexpr float BBmax = 9; //constant term
constexpr float maxTorqueZ = 30 * 5/6;

#elif defined(INFANTRY)
// START getters and setters
constexpr float TRACKWIDTH = 0.504;      // in m. We need to measure
constexpr float M = 14.8;                  // robot mass kg
constexpr float J = 0.4223;                  // measured from sys id kg-m^2
constexpr float R_WHEEL = 0.06;  // wheel radius m
constexpr float J_WHEEL = 0.000048223;          // wheel moment of inertia kg-m^2
constexpr float C_MOTOR = 3.5e-4 * .02299;          // motor damping kg-s/m^2`
constexpr float UK_MOTOR =  .25 *.02299;           // motor dry friction N-m
constexpr float COF_WHEEL = 0.9;           // unitless COF

constexpr float GEAR_RATIO = 3591.0f/187.0f;  // gear ratio
constexpr float P_IDLE = 2.6;         // W, idle power
constexpr float P_FOS = 0.65;       // unitless, power factor of safety

// Tunable Parameters
constexpr float KP_V_XY = 1150;  // proportional gain for velocity
constexpr float KP_V_ROT = 40;   // proportional gain for rotational velocity

constexpr float KI_V = 0 / .002;  // integral gain for velocity

constexpr float IV_MAX = 120;  // maximum integral term for velocity control

constexpr float KP = 0;              // proportional gain for position control
constexpr float BEYBLADE_DELAY = .25;  // delay for beyblade mode/
constexpr float BBterm1 = 10.7717; //constant term
constexpr float BBterm2 = -2.0342; //linear term
constexpr float BBterm3 = -.5660; //quadratic term
constexpr float BBmax = 10.5; //constant term
constexpr float maxTorqueZ = 30;
#else// START getters and setters
constexpr float TRACKWIDTH = 0.49739;      // in m. We need to measure
constexpr float M = 14.0;                  // robot mass kg
constexpr float J = 0.44;                  // measured from sys id kg-m^2
constexpr float R_WHEEL = 0.048 / ROOT_2;  // wheel radius m
constexpr float J_WHEEL = 0.0009;          // wheel moment of inertia kg-m^2
constexpr float C_MOTOR = 2.5e-4 * 0.02299f;          // motor damping kg-s/m^2`
constexpr float UK_MOTOR = 0.14 * 0.02299f;           // motor dry friction N-m
constexpr float COF_WHEEL = 0.9;           // unitless COF

constexpr float GEAR_RATIO = 13.7;  // gear ratio
constexpr float P_IDLE = 3;         // W, idle power
constexpr float P_FOS = 0.87;       // unitless, power factor of safety

// Tunable Parameters
constexpr float KP_V_XY = 1500;  // proportional gain for velocity
constexpr float KP_V_ROT = 30;   // proportional gain for rotational velocity

constexpr float KI_V = 0;  // integral gain for velocity

constexpr float IV_MAX = 120;  // maximum integral term for velocity control

constexpr float KP = 0;              // proportional gain for position control
constexpr float BEYBLADE_DELAY = .25;  // delay for beyblade mode/
constexpr float BBterm1 = 10.7717; //constant term
constexpr float BBterm2 = -2.0342; //linear term
constexpr float BBterm3 = -.5660; //quadratic term
constexpr float BBmax = 10.5; //constant term
constexpr float maxTorqueZ = 30;
#endif


// after the ifdefs

constexpr float LATENCY = 0.008;  // latency s
constexpr float DT = 0.002;       // DT in s

//  Feedforward gains from fundamental system  constexprants
constexpr float K_V = KB;              // Velocity feedforward gain (back EMF  constexprant)
constexpr float K_VIS = C_MOTOR / KT;  // Viscous damping feedforward gain
constexpr float K_S = UK_MOTOR / KT;   // Static friction feedforward g

constexpr float F_MAX = M * 9.81f * COF_WHEEL;    // maximum force allowed across all 4 wheels
constexpr float F_MIN_T = 5 / (2 * TRACKWIDTH);  // minimum force per wheel in the torque direction that the traction limiter is allowed to throttle to

const float M_EFFECTIVE = M + 4 * J_WHEEL * std::pow(GEAR_RATIO / R_WHEEL, 2.0f);
const float J_EFFECTIVE = J + 4 * J_WHEEL * std::pow((TRACKWIDTH / 2.0f) * (GEAR_RATIO / R_WHEEL), 2.0f);
const Pose2d KP_V{KP_V_XY, KP_V_XY, KP_V_ROT};
const Vector2d MIN_FORCE{-IV_MAX, -IV_MAX}, MAX_FORCE{IV_MAX, IV_MAX};

// queues
constexpr int Q_SIZE = (LATENCY / DT);
constexpr int BBQ_SIZE = (BEYBLADE_DELAY / DT);

// matrices (big but these only get run once so yay)
constexpr float ikr1[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
constexpr float ikr2[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
constexpr float ikr3[3] = {-GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};
constexpr float ikr4[3] = {GEAR_RATIO / (R_WHEEL * ROOT_2), -GEAR_RATIO / (R_WHEEL * ROOT_2), -(TRACKWIDTH *GEAR_RATIO) / (R_WHEEL * 2)};

constexpr float fkr1[4] = {R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
constexpr float fkr2[4] = {R_WHEEL * ROOT_2 / (4 * GEAR_RATIO), R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO), -R_WHEEL *ROOT_2 / (4 * GEAR_RATIO)};
constexpr float fkr3[4] = {-R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO), -R_WHEEL / (2 * TRACKWIDTH * GEAR_RATIO)};

constexpr float fir1[3] = {ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
constexpr float fir2[3] = {-ROOT_2 / 4, ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
constexpr float fir3[3] = {-ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};
constexpr float fir4[3] = {ROOT_2 / 4, -ROOT_2 / 4, -1 / (TRACKWIDTH * 2)};

constexpr float ffr1[4] = {1 / ROOT_2, -1 / ROOT_2, -1 / ROOT_2, 1 / ROOT_2};
constexpr float ffr2[4] = {1 / ROOT_2, 1 / ROOT_2, -1 / ROOT_2, -1 / ROOT_2};
constexpr float ffr3[4] = {-TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2, -TRACKWIDTH / 2};


const float *inverseKinematics[4] = {ikr1, ikr2, ikr3, ikr4};
const float *forwardKinematics[3] = {fkr1, fkr2, fkr3};
const float *forceInverseKinematics[4] = {fir1, fir2, fir3, fir4};
const float *forceKinematics[3] = {ffr1, ffr2, ffr3};

}  // namespace subsystems
