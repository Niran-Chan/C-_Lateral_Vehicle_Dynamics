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
/*!Kinematics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 \param dt double val of time step
 */
void bicycleKinematics(Vehicle* car,double dt);

void test();
/*!Dynamics Model of 2 wheeler
 \param car Pointer to Vehicle Class Instance
 */
void bicycleDynamics(Vehicle* car);
}
#endif /* Dynamics_hpp */
