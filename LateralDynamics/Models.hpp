//
//  Dynamics.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//

#ifndef Models_hpp
#define Models_hpp

#include <stdio.h>
#include <vector>
#include "Vehicle.hpp"
#include "Eigen/Dense"


namespace Models{
/*!Analytic Kinematics Model of 2 wheeler.
 Sufficient for low speed models.
 \param car Pointer to Vehicle Class Instance
 \param x0 initial conditions, {X,Y,Yaw_Angle}, respectively in 3x1 Matrix
 \param inputFilePath File Path for the CSV file
\param headers Name of headers for Velocity and Steering Percentage Respectively
 */
void bicycleAnalyticKinematics(Vehicle* car,MatrixXd x0,std::string inputFilePath,std::vector<std::string> headers);
/*!Numerical Kinematics Model of 2 wheeler.
 Sufficient for low speed models
 \param car Pointer to Vehicle Class Instance
 \param x0 initial conditions, {X,Y,Yaw_Angle}, respectively in 3x1 Matrix
 \param inputFilePath File Path for the CSV file
\param headers Name of headers for Velocity and Steering Percentage Respectively
 */
void bicycleNumericalKinematics(Vehicle* car,MatrixXd x0,std::string inputFilePath,std::vector<std::string> headers);
void test();
/*!Dynamics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 The model determines vehicle kinematics first, using the bicycleKinematics Model, followed by using the final tire angle as the input to determine the dynamics of motion.
 
 */
void bicycleAnalyticSlipDynamics(Vehicle* car);
/*!Dynamics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 The model determines vehicle kinematics first, using the bicycleKinematics Model, followed by using the final tire angle as the input to determine the dynamics of motion.
 */
void bicycleNumericalSlipDynamics(Vehicle* car);
/*!Ackermann Model implementation. Input: Ackermann Steering Angle, Output: Ackermann Percentage,
 \param car Pointer to Vehicle Class Instance
 \param δf current steering angle
 \param pAck Ackermann Percentage
 \return vector containing information in the following order: δi,δo
 */
std::vector<double> ackermannModel(Vehicle* car,double δf,double pAck);

/*!
 Pacejka Tire Model for determining lateral forces on tire
 \param α slip angle
 \return Lateral Force Fy
 */
double pacejkaTireModel(double α );
/*!
    Vehicle Lateral Dynamics in terms of yaw rate and slip angle
    \param car Pointer to Vehicle Class Instance
    \param δ Steering Angle, rad
    \param β Side Slip Angle
    \param r Yaw Rate
    \param Vx current Velocity of Vehicle in x-direction, m/s
    \param φ current slope angle, radians
    \return vector in the form: {d(slip angle), d(yaw rate}}
 */
std::vector<double> sideslipModel(Vehicle* car,double δ,double β,double r,double Vx, double φ);
}
/*!
    Steering Low Pass Filter
        \param δfcmd Current steering Angle
        \param δfcmdPrev Previous Steering Angle
        \param cutoffFreq Frequency cutoff. Higher Frequency Cutoff leads to leaning of smoothing towards upperbound
        \param dt Timestep
 */
double steeringLowPassFilter(double δfcmd, double δfcmdPrev,double cutoffFreq,double dt);

#endif /* Dynamics_hpp */
