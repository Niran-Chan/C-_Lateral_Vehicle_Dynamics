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

namespace Dynamics{
void bicycle_kinematics(Vehicle* car,double dt);
void test();
void fourWheelDynamics(Vehicle* car);
}
#endif /* Dynamics_hpp */
