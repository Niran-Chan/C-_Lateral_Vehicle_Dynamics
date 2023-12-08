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
#include <vector>
//#include "odeint.hpp" //ODE Library

/*
std::vector<std::vector<double>> add2DVectors(std::vector<std::vector<double>> &a,std::vector<std::vector<double>> &b){
    if(a.size() != b.size())
    {
        std::cout << "[-] Error: Unable to sum vectors due to size difference";
        return {{}};
    }
    std::vector<std::vector<double>> res(a.size());
    //Populate new res
    for(int i=0; i < a.size();++i){
        for(int j =0;j < a[i].size();++j){
            res[i].push_back(b[j]);
        }
    }
    
    return res;
}
*/
void Dynamics::bicycleKinematics(Vehicle* car,double dt){
    //Bicycle Kinematics for Centre of Vehicle
    //std::cout << "Bicycle Kinematics Function" << std::endl;
    double lf = car -> lf;
    double lr = car -> lr;
    double v = car ->v;
    double ψ = car -> ψ;
    double δf = car -> δf;
    double δr = car -> δr;
    
    double L = lf + lr;
    double β = atan((lr * tan(δf) + lf * tan(δr)) / L); //Slip Angle
    double Vx = v * cos(ψ + β); //Change in X Direction
    double Vy = v * sin(ψ + β); //Change in Y Direction
    double dψ = v * cos(β) * (tan(δf) - tan(δr))/L ; //Change in Heading Angle
    
    //car -> v = sqrt(pow(Vx,2) + pow(Vy,2));
    car -> v = Vx + Vy;
    car -> ψ = std::fmod((car -> ψ + dψ * dt),(2*M_PI)); //Keep within 360 degrees
    //car -> v = car -> ψ >= M_PI/2 &&  car -> ψ <= 3*M_PI/2 ? -car -> v : car->v; //Giving Velocity its direction based on the heading angle
}
//4-Wheeler
void Dynamics::bicycleDynamics(Vehicle* car){
    double lf = car -> lf;
    double lr = car -> lr;
    double m = car -> m;
    double Iz = car -> m;
    double Cαf = car -> Cαf;
    double Cαr = car ->Cαr;
    double θVf = car -> θVf;
    double θVr = car -> θVr;
    double δf = car -> δf;
    double δr = car -> δr;
    double lw = car -> lw;
    double L = car -> L ;
    double δ = sqrt((δr - δf)*L/lw);
    
    auto du = car -> du;
    double Vx = car -> v * cos(θVf); //Assuming front steering
    //Return State Space Model with the respective inputs to predict behvaiour
    //du = Au(t) + Bi(t), where A and B are the linear combinations of current state with input respectively
    std::vector<std::vector<double>> A {{0,1,0,0},{0,-2*(Cαf + Cαr)/(m * Vx),0,-Vx -2 * (Cαf*lf - Cαr*lr)/m*Vx},{0,0,0,1},{0,-2 * (Cαf*lf - Cαr*lr)/Iz*Vx,0,-2 * (Cαf*lf*lf + Cαr*lr*lr)/Iz*Vx}}; //d/dt {y,y',ψ,ψ'}
    std::vector<std::vector<double>> B {{0},{δ * 2*Cαf*δ/m},{0},{δ * 2*lf*Cαf/Iz}};
    //Final equation is a + b
    if(du.size() == 2){
        du[0] = A;
        du[1] = B;
    }
    else{
        du.push_back(A);
        du.push_back(B);
    }
    car -> du = du;
    
}
void Dynamics::test(){
    std::cout<<"From header file" << std::endl;

}

