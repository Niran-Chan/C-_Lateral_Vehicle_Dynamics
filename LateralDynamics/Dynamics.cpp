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
    auto inputSequence = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv",std::vector<std::string>{"ego_vel_mts_sec","ego_yaw_r_rad_sec"}); // 2xTimesamples
    
    MatrixXd x0;x0.resize(3,1);x0.setZero(); //initial values set to 0, 3x1 Matrix
    x0(0,0) = -324.429047; //X
    x0(1,0) = 58.0676765;   //Y
    x0(2,0) = -2.66446209;  //ψ
    
    MatrixXd A {{1,0,0},{0,1,0},{0,0,1}}; //Identity Matrix, 4x4
    
  
    
    MatrixXd C {{1,0,0},{0,1,0},{0,0,1}}; // 3x3 Matrix
    MatrixXd D;D.resize(3,2);D.setZero(); // 4x2 Matrix
    
    //du -> setMatrices(A, B, C, D, x0, inputSequence);
    
    
    // TEST WITHOUT SIMULATION CLASS
    
    int r = C.rows();
    int n = A.rows();
    int timeSamples =inputSequence.cols();
    MatrixXd simulatedOutputSequence;simulatedOutputSequence.resize(r, timeSamples); simulatedOutputSequence.setZero();
    
    MatrixXd simulatedStateSequence;simulatedStateSequence.resize(n, timeSamples); simulatedStateSequence.setZero();
    

    
    for (int j = 0; j < timeSamples; j++)
    {
        if(j!=0) ψ = simulatedStateSequence.col(j-1)[2];
        MatrixXd B {{cos(ψ) * dt,0},{sin(ψ) * dt,0},{0,dt}}; // 3x2 Matrix ERROR IN LOGIC: NEED TAKE YAW RATE DURING SIMULATION TIME
        
        if (j == 0)
        {
            //x0.col(j) can work if testing various different initial conditions
            
            simulatedStateSequence.col(j) = x0; //Equate current, there is an assertion error here on size
            simulatedOutputSequence.col(j) = C * x0 ;
            //D * inputSequence; //Time Invariant system
            
        }
        else
        {
            simulatedStateSequence.col(j) = A * simulatedStateSequence.col(j - 1) + B * inputSequence.col(j - 1);
            simulatedOutputSequence.col(j) = C * simulatedStateSequence.col(j) + D * inputSequence.col(j) ;
            
        }
    }
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    const std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";
    
    std::ofstream fileSimulatedStateSequence(FILEPATH + "simulatedStateSequenceKinematicsWithoutClass.csv");
    if (fileSimulatedStateSequence.is_open())
    {
        auto transposedStateSequence =simulatedStateSequence.transpose();
        for(int i =1; i < transposedStateSequence.cols() + 1; ++i){
           std::string isComma = i == transposedStateSequence.cols() ? "" : ",";
            fileSimulatedStateSequence << "x" << std::to_string(i) << isComma;
        }
        fileSimulatedStateSequence << std::endl;
        fileSimulatedStateSequence << transposedStateSequence.format(CSVFormat);
        fileSimulatedStateSequence.close();
    }
    
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
    
    //du -> runSimulation();

    //du -> saveData("AKinematics.csv", "BKinematics.csv", "CKinematics.csv", "DKinematics.csv", "x0Kinematics.csv", "inputSequenceKinematics.csv", "simulatedStateSequenceKinematics.csv", "simulatedOutputSequenceKinematics.csv");

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
std::vector<double> Dynamics::AckermannModel(Vehicle* car){
    //Front wheel calculation
    double δf = car -> δf;
    double lw = car -> lw;
    double L = car -> L;
    double r = L/sinf(δf);
    double δi = atanl(L/(r-lw/2));
    double δo = atanl(L/(r+lw/2));
    double ackermannPercent = (δi - δo)/δi * 100;
    return std::vector<double> {δi,δo,ackermannPercent};
}
double Dynamics::PacejkaTireModel(double α){
    /*
     Estimates
     Stiffness Factor (B):
         Typical Range: 10 to 1000
         Starting Point: 100

     Shape Factor (C):
         Typical Range: 1 to 3
         Starting Point: 1.5

     Peak Lateral or Longitudinal Force (D):
         Typical Range: Varies widely based on the units used for force (e.g., N, kN, lb)
         Starting Point: Depends on the specific tire and testing conditions

     Curvature Factor (E):
         Typical Range: 0.1 to 1
         Starting Point: 0.1
     */
    double B = 100;
    double C = 1.5;
    double D = 15000;
    double E = 0.1;
    
    double Fy = D * sin(C * atanl(B * α - E * (B * α - atanl(B * α))));
    return Fy;
    //Fx=D⋅sin⁡(C⋅arctan⁡(B⋅κ−E⋅(B⋅κ−arctan⁡(B⋅κ))))Fx​=D⋅sin(C⋅arctan(B⋅κ−E⋅(B⋅κ−arctan(B⋅κ)))) //longitudinal force
}
void Dynamics::test(){
    std::cout<<"From header file" << std::endl;
    
}

