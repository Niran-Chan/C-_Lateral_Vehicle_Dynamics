//
//  Vehicle.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//

#ifndef Vehicle_hpp
#define Vehicle_hpp

#include <iostream>
#include <vector>
#include <math.h>
#include "Eigen/Dense"
#include "SimulateSystem.hpp"

class Vehicle {
    //Vehicle Parameters

public:
    double v;//Velocity, m/s
    double δf; //Angle of Front Tire wrt longitudinal axis of Vehicle (rad)
    double δr; //0 as vehicle is front wheel steering only
    double lf;
    double lr;
    double m; //Mass in kg
    double Iz;//Mass Moment of Inertia About Z Axis (Approximation of 6m length, 2m width and 2.5m height)
    double ψ; //heading angle: Orientation of vehicle wrt global X axis in radians
    
    //Slip Dynamics Coefficients
    double Cαf;//Proportionality Constant, in this case Cornering Stiffness of Front Wheel
    double Cαr;//Proportionality Constant, in this case Cornering Stiffness of Rear Wheel
    double θVf; //Front Tyre Velocity Angle (A portion of the δf as at high speeds, velocity does not immediately change due to inertia) (rad)
    double θVr;
    double lw; //car wheelbase width
    double L; //Length of Vehicle

    Vehicle(); 
    
    
};

//Include any further functions into Vehicle.cpp source file
#endif /* Vehicle_hpp */
