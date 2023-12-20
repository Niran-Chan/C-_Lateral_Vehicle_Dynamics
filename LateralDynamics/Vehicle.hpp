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

///
///This class is defined to only contain Vehicle Parameters
///
class Vehicle {

public:
    double v;///<Velocity of Vehicle (m/s)
    double δf; ///<Angle of Front Tire w.r.t Longitudinal Axis of Vehicle (rad)
    double δr; ///<Angle of Rear Tire w.r.t Longitudinal Axis of Vehicle (rad)
    double lf; ///<Distance of Front Wheel from Centre of Gravity of System (m)
    double lr; ///<Distance of Rear Wheel from Centre of Gravity of System (m)
    double m; ///<Mass (kg)
    double Iz;///<Mass Moment of Inertia About Z Axis (kg*m<SUP>2</SUP>)
    
    //(Approximation of 6m length, 2m width and 2.5m height)
    
    double ψ; ///<Heading angle, Orientation of Vehicle w.r.t global X axis (rad)

    //Slip Dynamics Coefficients
    double Cαf;///<Cornering Stiffness of Front Wheel, expressed as proportionality constant (dim)
    double Cαr;///<Cornering Stiffness of Rear Wheel, expressed as proportionality constant (dim)
    double θVf;///<Front Tyre Velocity Angle (A portion of the δf as at high speeds, velocity does not immediately change due to inertia) (rad)
    double θVr;///<Rear Tyre Velocity Angle (A portion of the δr as at high speeds, velocity does not immediately change due to inertia) (rad)
    double lw; ///<Vehicle Wheelbase Width (m)
    double L; ///<Length of Vehicle (m)

    Vehicle();
    ///<Default Constructor
    
    
};

//Include any further functions into Vehicle.cpp source file
#endif /* Vehicle_hpp */
