//
//  Vehicle.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//

#ifndef Vehicle_hpp
#define Vehicle_hpp

#include <stdio.h>
#include <vector>
class Vehicle {
public:
    //Set Vehicle Params
    /*
    double vx  = 0; //Velocity in X Direction
    double slipAngle  = ego.slip_angle[end]; //Slip Angle
    double yawRate   = ego.yaw_rate[end] //Yaw Rate
    double steeringAngle  = ego.steering_angle[end] //Steering Angle
    double Cαf = ego.param[:Cαf] //
    double Cαr = ego.param[:Cαr]
    double lf  = ego.param[:lf]
    double lr  = ego.param[:lr]
    double m   = ego.param[:m] //Mass
    double Iz  = ego.param[:Iz] //Mass Moment of Inertia About Z Axis
     */
    double v = 10; //Velocity, m/s
    double slipAngle  = 0.3; //Slip Angle (Difference between Heading Direction and Current Vehicle Direction, radians
    double δf = 1;
    double δr = 0; //0 as vehicle is front wheel steering only
    double Cαf = 1;//
    double Cαr = 1;
    double lf  = 1;
    double lr  = 1;
    double m   = 1000; //Mass in kg
    double Iz  = 1;//Mass Moment of Inertia About Z Axis
    double ψ = 1; //heading angle: Orientation of vehicle wrt global X axis
    //double speed = 3;
    
    
    //Constructor
    Vehicle(){
        //Use this to set initial conditions if need be
    }
};

//Include any further functions into Vehicle.cpp source file
#endif /* Vehicle_hpp */
