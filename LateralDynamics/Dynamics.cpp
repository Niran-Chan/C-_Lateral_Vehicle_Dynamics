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


std::vector<double> Dynamics::bicycle_kinematics(double lf,double lr,double v, double ψ, double δf,double δr){
    //Bicycle Kinematics for Centre of Vehicle
    //std::cout << "Bicycle Kinematics Function" << std::endl;
    double L = lf + lr;
    double β = atan((lr * tan(δf) + lf * tan(δr)) / L); //Slip Angle
    double Vx = v * cos(ψ + β); //Change in X Direction
    double Vy = v * sin(ψ + β); //Change in Y Direction
    double dψ = v * cos(β) * (tan(δf) - tan(δr))/L ; //Change in Heading Angle
    std::vector<double> vars {Vx, Vy, dψ};
    return vars;
}

void Dynamics::test(){
    std::cout<<"From header file" << std::endl;

}

