//
//  Vehicle.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//

#include "Vehicle.hpp"

//Functions that modify Vehicle Parameters
Vehicle::Vehicle(){
    //Constructor
    
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
    this -> v = 0.0; //Velocity, m/s
    this -> δf = M_PI_4; //Angle of Front Tire wrt longitudinal axis of Vehicle (rad)
    this -> δr = 0.0; //0 as vehicle is front wheel steering only
    this -> lf  = 1.803;
    this -> lr  = 1.517;
    this -> m   = 1000; //Mass in kg
    //this -> Iz  = (1.0/12.0) *(std::pow(2,2) + std::pow(2.5,2));//Mass Moment of Inertia About Z Axis (Approximation of 6m length, 2m width and 2.5m height)
    this -> Iz = 9620.0;
    //ENSURE FOR FLOAT MATH OPERATIONS, REPRESENT NUMBERS WITH DECIMALS
    this -> ψ = -2.66446209; //heading angle: Orientation of vehicle wrt global X axis in radians
    
    //Slip Dynamics Coefficients
    this -> Cαf = 160830.0;//Proportionality Constant, in this case Cornering Stiffness of Front Wheel
    this -> Cαr = 160320.7;//Proportionality Constant, in this case Cornering Stiffness of Rear Wheel
    this -> θVf = 1; //Front Tyre Velocity Angle (A portion of the δf as at high speeds, velocity does not immediately change due to inertia) (rad)
    this -> θVr = 1;
    this -> lw = 2.5; //car wheelbase width
    this -> L = lr+lf; //Length of Vehicle
    //Include Steering Angle (Maximum and current? ) 
    
    
    
    //State space model (excluding road bank angle)
    //u is the final output
    //std::vector<std::vector<std::vector<double>>> du; //Dynamics Model
    
    //Initialise State Space Model
    //this -> du = new SimulateSystem(); //Construct State Space Model Object Instance
    
}
