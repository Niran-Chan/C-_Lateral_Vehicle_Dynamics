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
#include "HelperFunctions.hpp"

namespace Models{
/*!Analytic Kinematics Model of 2 wheeler.
 Sufficient for low speed models.
 \param car Pointer to Vehicle Class Instance
 \param V Velocity vector
 \param ψ Current Yaw Angle
 \param steerCommandPercent Percentage of Steering done
 \return 4x1 Matrix in following format: {dx,dy,dψ,β}
 */
MatrixXd bicycleKinematicsStep(Vehicle* car,double V,double ψ,double steerCommandPercent,double dt);
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
 \param Vx Model assumes constant longitudinal velocity around turn
Determine slip dynamics of vehicle around a curve based on its current longitudinal velocity
 */
void bicycleSlipDynamicsSimulation(Vehicle* car,MatrixXd inputSequenceSteering,double Vx,double dt,std::string filePath);

/*!Dynamics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 \param sequence Relevant Matrix Column to be updated in step
 \param steerCommandPercent Percentage of front wheel steered relative to max steering angle
 \param Vx Current Longitudinal Velocity of Vehicle
 \param φ Current Bank angle of Vehicle
 \param dt timestep
 \return 4x1 Matrix,du, of the following : {dy,d2y,dψ,d2ψ}
 */
MatrixXd bicycleSlipDynamicsStep(Vehicle* car,MatrixXd sequence,double steerCommandPercent,double Vx,double φ,double dt);

/*!Ackermann Model implementation. Input: Ackermann Steering Angle, Output: Ackermann Percentage,
 \param car Pointer to Vehicle Class Instance
 \param steerCommandPercent Current percentage of steering
 \param pAck Ackermann Percentage
 \return Column Matrix returned in the following order: δi,δo
 */
MatrixXd ackermannModel(Vehicle* car,double steerCommandPercent,double curvature,double pAck);

/*!
 Pacejka Tire Model for determining lateral forces on tire
 \param car Pointer to Vehicle Class Instance
 \param steeringAngles Column Matrix of steering angles of all wheels required
 \param ψ current yaw angle
 \return Pair of Matrices in the following order: Lateral Force Fy,α. Each row correlates to each row in provided steeringAngles Matrix.
 */
std::pair<MatrixXd,MatrixXd> pacejkaTireModel(Vehicle* car, MatrixXd steeringAngles,double ψ);
/*!
    Vehicle Lateral Dynamics in terms of yaw rate and slip angle
    \param car Pointer to Vehicle Class Instance
    \param δ Steering Angle, rad
    \param β Side Slip Angle
    \param r Yaw Rate
    \param Vx current Velocity of Vehicle in x-direction, m/s
    \param φ current slope angle, radians
    \return pair in the form: {d(slip angle), d(yaw rate}}
 */
MatrixXd sideslipModel(Vehicle* car,double δ,double β,double r,double Vx, double φ);

/*!
Steering Low Pass Filter
    \param δfcmd Current steering Angle
    \param δfcmdPrev Previous Steering Angle
    \param cutoffFreq Frequency cutoff. Higher Frequency Cutoff leads to leaning of smoothing towards upperbound
    \param dt Timestep
 */
double steeringLowPassFilter(double δfcmd, double δfcmdPrev,double cutoffFreq,double dt);

/*!
 Errors obtained from tracking vehicle
 \param car Vehicle Class Instance
 \param e1Vals Distance between C.G. of vehicle and center of road
 \param e2Vals Orientation error w.r.t road
 \param dt Timestep
 */

MatrixXd laneErrorSimulation(Vehicle *car,MatrixXd e1Vals,MatrixXd e2Vals,MatrixXd inputSequence, double dt);
}
#endif /* Dynamics_hpp */
