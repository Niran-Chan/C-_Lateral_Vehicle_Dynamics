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
    std::vector<double> bicycle_kinematics(double lf,double lr,double v, double ψ, double δf);
    
    void test();
}
#endif /* Dynamics_hpp */
