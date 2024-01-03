//
//  Dynamics.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//
// Use this file for Dynamic Model Functions

#include "Dynamics.hpp"

void Dynamics::bicycleKinematics(Vehicle* car){
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
    double dt = 0.04;
    //car -> ψ = std::fmod((car -> ψ + dψ * dt),(2*M_PI)); //Keep within 360 degrees
    car -> ψ = car -> ψ + dψ * dt;

    /*
     State Space Model=
     A[[x] [y] [ψ]] + BU, where A is Jacobian Matrix and B is
     Output model=[1 1 1 1]x + Du
     */
    SimulateSystem* du = new SimulateSystem(); //create state-space model first
    
    //Velocity and Steering Angle input
    auto inputSequence = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv",std::vector<std::string>{"ego_vel_mts_sec","ego_yaw_rad"}); // 2xTimesamples
    
    MatrixXd x0;x0.resize(3,1);x0.setZero(); //initial values set to 0, 3x1 Matrix
    MatrixXd A {{1,0,0},{0,1,0},{0,0,1}}; //Identity Matrix, 4x4 Matrix
    MatrixXd B {{cos(ψ) * dt,0},{sin(ψ) * dt,0},{0,dt}}; // 3x2 Matrix
    MatrixXd C;C.resize(3,3);C.setOnes(); // 3x3 Matrix
    MatrixXd D;D.resize(3,2);D.setZero(); // 4x2 Matrix
    //D Matrix will associate input coefficients
    D(0,0) = 1;
    D(2,1) = 1;
    du -> setMatrices(A, B, C, D, x0, inputSequence);
    //du -> modelResize(); //Resizing method if setMatrices was not used
    //du -> printSimulationParams();
    /*
     //Preview Input Sequence
    for(int i =0 ; i < 2; ++i){
        for(int j =0; j < 25; ++j){
            std::cout << inputSequence(i,j) << ",";
        }
        std::cout <<"\n" <<std::string(35,'/')<< std::endl;
    }
     */
    
    du -> runSimulation();

    du -> saveData("AKinematics.csv", "BKinematics.csv", "CKinematics.csv", "DKinematics.csv", "x0Kinematics.csv", "inputSequenceKinematics.csv", "simulatedStateSequenceKinematics.csv", "simulatedOutputSequenceKinematics.csv");
    //car -> v = car -> ψ >= M_PI/2 &&  car -> ψ <= 3*M_PI/2 ? -car -> v : car->v; //Giving Velocity its direction based on the heading angle
}
//4-Wheeler
void Dynamics::bicycleDynamics(Vehicle* car){
    double lf = car -> lf;
    double lr = car -> lr;
    double m = car -> m;
    double Iz = car -> Iz;
    double Cαf = car -> Cαf;
    double Cαr = car ->Cαr;
    double θVf = car -> θVf;
    
    //double δ = sqrt((δr - δf)*L/lw);
    
    //Assuming front angle tire is significantly larger than rear wheel angle difference,
    /*
     We also need to take into account of inputs such as steering and velocity.
     */
    double Vx = car -> v * cos(θVf); //Assuming front steering
    double Vy = car -> v * sin(θVf);
    
    double ψ = car -> ψ;
    double dψ = 0; //Rate of change of heading angle (perhaps shift to Vehicle class?)
    
    /*
     Return State Space Model with the respective inputs to predict behvaiour
     du = Au(t) + Bi(t), where A and B are the linear combinations of current state with input respectively
     State space sequence model: d/dt {y,y',ψ,ψ'}
    */
    
    MatrixXd A {{0,1,0,0},{0,-2*(Cαf + Cαr)/(m * Vx),0,-Vx -2 * (Cαf*lf - Cαr*lr)/m*Vx},{0,0,0,1},{0,-2 * (Cαf*lf - Cαr*lr)/Iz*Vx,0,-2 * (Cαf*lf*lf + Cαr*lr*lr)/Iz*Vx}}; //4x4 Matrix
    
    //MatrixXd B {{0},{δ * 2*Cαf*δ/m},{0},{δ * 2*lf*Cαf/Iz}};
    MatrixXd B {{0},{2*Cαf/m},{0},{2*lf*Cαf/Iz}}; //4x1 Matrix, input must be in 1xtimeSamples matrix
    
    SimulateSystem* du = new SimulateSystem();//Create State Space Model for Simulation
    //auto inputSequence = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/input_sequence/ramp_input.csv"); //Our input currently is the direction of the front tire,δ, assuming that this is our current input value for our tires
    auto inputSequence = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv",std::vector<std::string> {"ref_yaw_deg"}); //Actual Test input
    //std::cout << inputSequence << std::endl;
    
    MatrixXd C;C.resize(1, 4); C.setZero();C(0,1) = 1;// Output Matrix Coefficient, Velocity in Y only

    MatrixXd D; D.resize(1,1); D.setZero(); //Ackermann equation is controller gain for ackermann angle behaviour, which is difference between velocities of both the tires?
    //u matrix is (1,m) matrix
    
    MatrixXd x0;x0.resize(4,1);x0.setZero();
    
    x0(0,0) = 0;
    x0(1,0) = Vy;
    x0(2,0) = ψ;
    x0(3,0) = dψ; //Initial Conditions of state variables

    du -> setMatrices(A,B,C,D, x0, inputSequence);
    du -> printSimulationParams();
    du -> runSimulation();
    du -> saveData("A.csv","B.csv","C.csv","D.csv","x0.csv", "inputSequenceFile.csv", "simulatedStateSequence.csv", "simulatedOutputSequenceFile.csv");
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

