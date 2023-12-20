//
//  Dynamics.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//
// Use this file for Dynamic Model Functions

#include "Dynamics.hpp"

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
    double β = atan((lr * tan(δf) + lf * tan(δr)) / L); //Slip Angle (Difference between Heading Direction and Current Vehicle Direction, radians)
    double Vx = v * cos(ψ + β); //Change in X Direction wrt x axis
    double Vy = v * sin(ψ + β); //Change in Y Direction wrt y axis
    double dψ = v * cos(β) * (tan(δf) - tan(δr))/L ; //Change in Heading Angle
    //std::cout << car -> δf << std::endl;
    //car -> v = sqrt(pow(Vx,2) + pow(Vy,2));
    car -> v = Vx + Vy;
    //car -> ψ = std::fmod((car -> ψ + dψ * dt),(2*M_PI)); //Keep within 360 degrees
    car -> ψ = car -> ψ + dψ * dt;
    //car -> v = car -> ψ >= M_PI/2 &&  car -> ψ <= 3*M_PI/2 ? -car -> v : car->v; //Giving Velocity its direction based on the heading angle
}
//4-Wheeler
void Dynamics::bicycleDynamics(Vehicle* car){
    double lf = car -> lf;
    double lr = car -> lr;
    double lw = car -> lw;
    double m = car -> m;
    double Iz = car -> Iz;
    double Cαf = car -> Cαf;
    double Cαr = car ->Cαr;
    double θVf = car -> θVf;
    double θVr = car -> θVr;
    double δf = car -> δf;
    double δr = car -> δr;
    double L = car -> L ;
    
    //double δ = sqrt((δr - δf)*L/lw);
    
    //Assuming front angle tire is significantly larger than rear wheel angle difference,

    double Vx = car -> v * cos(θVf); //Assuming front steering
    double Vy = car -> v * sin(θVf);
    
    double ψ = car -> ψ;
    double dψ = 0; //Rate of change of heading angle (perhaps shift to Vehicle class?)
    
    /*
     Return State Space Model with the respective inputs to predict behvaiour
     du = Au(t) + Bi(t), where A and B are the linear combinations of current state with input respectively
     State space sequence model: d/dt {y,y',ψ,ψ'}
    */
    
    MatrixXd A {{0,1,0,0},{0,-2*(Cαf + Cαr)/(m * Vx),0,-Vx -2 * (Cαf*lf - Cαr*lr)/m*Vx},{0,0,0,1},{0,-2 * (Cαf*lf - Cαr*lr)/Iz*Vx,0,-2 * (Cαf*lf*lf + Cαr*lr*lr)/Iz*Vx}};
    
    //MatrixXd B {{0},{δ * 2*Cαf*δ/m},{0},{δ * 2*lf*Cαf/Iz}};
    MatrixXd B {{0},{2*Cαf/m},{0},{2*lf*Cαf/Iz}};
    
    SimulateSystem* du = new SimulateSystem();//Create State Space Model for Simulation
    auto inputSequence = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/input_sequence/ramp_input.csv"); //Our input currently is the direction of the front tire,δ, assuming that this is our current input value for our tires

    MatrixXd C;C.resize(1, 4); C.setZero();C(0,1) = 1;// Output Matrix Coefficient, Velocity in Y only
    MatrixXd x0;x0.resize(4,1);x0.setZero();
    x0(0,0) = 0;
    x0(1,0) = Vy;
    x0(2,0) = ψ;
    x0(3,0) = dψ; //Initial Conditions of state variables

    du -> setMatrices(A,B,C, x0, inputSequence);
    du -> printSimulationParams();
    du -> runSimulation();
    du -> saveData("A.csv","B.csv","C.csv","x0.csv", "inputSequenceFile.csv", "simulatedStateSequence.csv", "simulatedOutputSequenceFile.csv"); 
    /*
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
    */
}
void Dynamics::test(){
    std::cout<<"From header file" << std::endl;

}

