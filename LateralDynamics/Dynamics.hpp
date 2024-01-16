//
//  Dynamics.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//

#ifndef Dynamics_hpp
#define Dynamics_hpp

#include <stdio.h>
#include <vector>
#include "Vehicle.hpp"
#include "Eigen/Dense"


namespace Dynamics{
/*!Kinematics Model of 2 wheeler.
 Sufficient for low speed models.
 \param car Pointer to Vehicle Class Instance
 */
void bicycleKinematics(Vehicle* car);

void test();
/*!Dynamics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 The model determines vehicle kinematics first, using the bicycleKinematics Model, followed by using the final tire angle as the input to determine the dynamics of motion.
 
 */
void bicycleDynamics(Vehicle* car);

/*!Ackermann Model implementation
 \param car Pointer to Vehicle Class Instance
 \return vector containing information in the following order: δi,δo,ackermann percent
 */
std::vector<double> AckermannModel(Vehicle* car);

/*!
 Pacejka Tire Model for determining lateral forces on tire
 \param α slip angle
 \return Lateral Force Fy
 */
double PacejkaTireModel(double α );
}
#endif /* Dynamics_hpp */
