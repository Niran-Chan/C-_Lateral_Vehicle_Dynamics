//
//  Dynamics.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//
// Use this file for Dynamic Model Functions

#include "Dynamics.hpp"
#include "Vehicle.hpp"
#include <iostream>
//#include "odeint.hpp" //ODE Library


std::vector<double> Dynamics::bicycle_kinematics(double lf,double lr,double v, double ψ, double δf){
    //std::cout << "Bicycle Kinematics Function" << std::endl;
    double L = lf + lr;
    double β = atan(lr * tan(δf) / L); //Slip Angle
    double dX = v * cos(ψ + β); //Change in X Direction
    double dY = v * sin(ψ + β); //Change in Y Direction
    double dψ = v * cos(β) * tan(δf) / L; //Change in Steering Rate
    double dθ = v * sin(δf)/L; //Change in Heading Angle
    std::vector<double> vars {dX, dY, dψ,dθ,β};
    return vars;
}


void Dynamics::test(){
    std::cout<<"From header file" << std::endl;

}

