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
#include <odeint>

std::vector<double> Dynamics::bicycle_kinematics(double lf,double lr,double v, double ψ, double δf){
    std::cout << "Bicycle Kinematics Function" << std::endl;
    double L = lf + lr;
    double β = atan(lr * tan(δf) / L);
    double dX = v * cos(ψ + β);
    double dY = v * sin(ψ + β);
    double dψ = v * cos(β) * tan(δf) / L;
    std::vector<double> vars {dX, dY, dψ, β};
    return vars;
}
std::vector <double> Dynamics::bicycle_kinematics_forward(Vehicle* car){
    double L = car -> lf + car -> lr;
    double β = atan(car -> lr * tan(car -> steeringAngle) / L);
    double dX = car -> speed * cos(car ->yaw + β);
    double dY = car -> speed * sin(car ->yaw + β);
    double dψ = car -> speed * cos(β) * tan(car -> steeringAngle) / L;
    std::vector<double> con {dX, dY, dψ, β};
    return con;
}

std::vector<double> Dynamics::bicycle_kinematics_backward(Vehicle* car)
{
    double L = car -> lf + car -> lr;
double β = - atan(car -> lf * tan(car -> steeringAngle) / L);;
double dX = car -> speed * cos(car -> yaw + β);
double dY = car -> speed * sin(car ->yaw + β);
double dψ = - car -> speed * cos(β) * tan(car -> steeringAngle) / L;
std::vector<double> con {dX, dY, dψ, β};
return con;
}


void Dynamics::test(){
    std::cout<<"From header file" << std::endl;

}

