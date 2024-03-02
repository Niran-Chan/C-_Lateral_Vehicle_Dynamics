//
//  Dynamics.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//
// Use this file for Dynamic Model Functions

#include "Models.hpp"

//Environment Parameters
extern const double g = 9.81;

void Models::bicycleAnalyticKinematics(Vehicle* car,MatrixXd x0,std::string inputFilePath,std::vector<std::string> headers){


    double δfmax = car -> δfmax; //Max angle of Vehicle constricted by Ackermann
    double δrmax = car -> δrmax;
    double lw = car -> lw;
    double L = car -> L;
    double lr = car -> lr;
    double lf = car -> lf;
    double dt = 0.04;
    /*
     State Space Model=
     A[[x] [y] [ψ]] + BU, where A is Jacobian Matrix and B is
     Output model=[1 1 1 1]x + Du
     */
 
    SimulateSystem* du = new SimulateSystem(); //create state-space model first
    
    auto inputSequenceBR = du -> openData(inputFilePath,headers); // inputSequence before resizing
    MatrixXd inputSequence;inputSequence.resize(3,inputSequenceBR.cols());
    
    /*Convert steering_feedback data to yawRate_rad/s, Iterate through steering data and
        yaw_rate = V/wheelbase * tan(steering_angle), where wheelbase is distance between front and rear axles
     */
    double δf = 0.0;
    double dψ = 0.0;
    double ψ = -2.66446209;
    double δr = 0.0; //Assuming 0 Steering angle of rear wheel
    
    for(int i =0; i < inputSequence.cols();++i)
    {
        double V = inputSequenceBR.col(i)[0]; //Velocity
        double steerCommandPercent = inputSequenceBR.col(i)[1]; //Steering Feedback Percent, assuming steering_ratio=1
        
        δf = steerCommandPercent/100.0 * δfmax; //Steering Angle
       
        //if(i!=0)
          //  δf = steeringLowPassFilter(δf, inputSequence.col(i-1)[2], 0.5, dt); //Low Pass Filter to smoothen out sudden values in steering
        
        double β = atan((lf * tan(δr) + lr * tan(δf)) / L); //Slip Angle (Difference between Heading Direction and Current Vehicle Direction, radians)
  
        dψ = V/L * cos(β) * (tan(δf) - tan(δr)); //Steering Angle -> Yaw Rate, Kinematic Interpretation
        ψ = ψ + dψ*dt;
        inputSequence.col(i)[0] = V * cos(ψ + β); //Vx
        inputSequence.col(i)[1] = V * sin(ψ + β); //Vy
        inputSequence.col(i)[2] = dψ; //Yaw Rate
    }
    
    MatrixXd A {{1,0,0},{0,1,0},{0,0,1}}; //Jacobian Matrix
    MatrixXd B {{dt,0,0},{0,dt,0},{0,0,dt}}; //Input Coefficient Matrix
    MatrixXd C {{1,0,0},{0,1,0},{0,0,1}}; // Output Matrix
    MatrixXd D;D.resize(3,3);D.setZero();
    std::cout << "Analytic Kinematic Simulation\n" << std::string(40,'-') <<std::endl;
    du -> setMatrices(A, B, C, D, x0, inputSequence);
    du -> printSimulationParams();
    du -> runSimulation();
    du -> saveData("A_AnalyticsKinematics.csv", "B_AnalyticsKinematics.csv", "C_AnalyticsKinematics.csv", "D_AnalyticsKinematics.csv", "x0_AnalyticsKinematics.csv", "inputSequence_AnalyticsKinematics.csv", "simulatedState_AnalyticsKinematicsSequence.csv", "simulatedOutput_AnalyticsKinematicsSequence.csv");
    std::cout << std::string(40,'-') << std::endl;
}
void Models::bicycleNumericalKinematics(Vehicle* car,MatrixXd x0,std::string inputFilePath,std::vector<std::string> headers){
    //Linear Approximation of Kinematics Model
    //Given velocity, steering angle, linearise and approximate the analytical model
    double dt = 0.04;
    
    SimulateSystem* du = new SimulateSystem();
    auto inputSequenceRaw = du -> openData(inputFilePath,headers);
    MatrixXd inputSequence;inputSequence.resize(3,3);inputSequence.setZero();
    
    MatrixXd A {{1,0,0},{0,1,0},{0,0,1}}; //Jacobian Matrix
    MatrixXd B {{dt,0,0},{0,dt,0},{0,0,dt}}; //Input Coefficient Matrix
    MatrixXd C {{1,0,0},{0,1,0},{0,0,1}}; // Output Matrix
    MatrixXd D;D.resize(3,3);D.setZero();
    std::cout << "Numerical Kinematic Simulation\n" << std::string(40,'-') <<std::endl;
    du -> setMatrices(A, B, C, D, x0, inputSequence);
    du -> printSimulationParams();
    du -> runSimulation();
    du -> saveData("A_NumericalKinematics.csv", "B_NumericalKinematics.csv", "C_NumericalKinematics.csv", "D_NumericalKinematics.csv", "x0_NumericalKinematics.csv", "inputSequence_NumericalKinematics.csv", "simulatedState_NumericalKinematicsSequence.csv", "simulatedOutput_NumericalKinematicsSequence.csv");
    std::cout << std::string(40,'-') << std::endl;
    
}

void Models::bicycleAnalyticSlipDynamics(Vehicle* car){

    double lf = car -> lf;
    double lr = car -> lr;
    double m = car -> m;
    double Iz = car -> Iz;
    double Cαf = car -> Cαf;
    double Cαr = car ->Cαr;
    double δfmax = car ->δfmax;
    double dt = 0.04;
    //Calculate Slip Distance
    //double δ = sqrt((δr - δf)*L/lw);
    
    //Assuming front angle tire is significantly larger than rear wheel angle difference,
    /*
     We also need to take into account of inputs such as steering and velocity.
     */
    
    /*
     Return State Space Model with the respective inputs to predict behvaiour
     du = Au(t) + Bi(t), where A and B are the linear combinations of current state with input respectively
     State space sequence model: d/dt {y,y',ψ,ψ',vx,vx^-1}
    */

    
    SimulateSystem* du = new SimulateSystem();//Create State Space Model for Simulation

    auto inputSequenceVx = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/inputSequenceKinematics.csv",std::vector<std::string> {"x1"}); //Vx
    auto inputSequenceSteering = du -> openData("/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv",std::vector<std::string>{"steer_fdback_percent"}); //Steering
    MatrixXd inputSequence;inputSequence.resize(4,inputSequenceSteering.cols());
    for(int i =0; i < inputSequence.cols(); ++i){
        double steerCommandPercent = inputSequenceSteering.col(i)[0]; //Steering Feedback Percent, assuming steering_ratio=1
        double δf = steerCommandPercent/100.0 * δfmax; //Steering Angle
        inputSequence(0,i) = δf;
    }
    double Vx; //Temporary

    std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";
    int timeSamples = inputSequence.cols();
    std::cout << "Time Samples for Simulation: "<< timeSamples << std::endl;
    MatrixXd simulatedOutputSequence; simulatedOutputSequence.resize(4, timeSamples); simulatedOutputSequence.setZero();
    MatrixXd simulatedStateSequence;simulatedStateSequence.resize(4, timeSamples);  simulatedStateSequence.setZero();
    
    double A1 = -2*(Cαf + Cαr);  //Include coefficient 2 if we are including more wheels
    double A2= -2*(Cαf*lf - Cαr*lr);
    double A3 = -2*(Cαf*lf*lf + Cαr*lr*lr);
    
  
    for (int j = 0; j < timeSamples; j++)
    {
        
        Vx = inputSequenceVx.col(j)[0];
    
        MatrixXd C  {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};// Output Matrix Coefficient, y,Vy,Yaw Angle,Yaw Rate
        MatrixXd x0; x0.resize(4,1);
        x0(0,0) = 0.0; // y
        x0(1,0) = 0.0; // Vy
        x0(2,0) = 0.0; // Yaw Angle
        x0(3,0) = 0.0; // Yaw Rate
        
    
        if (j == 0)
        {
            
            simulatedStateSequence.col(j) = x0; //Equate current, there is an assertion error here on size
            simulatedOutputSequence.col(j) = C * x0 ;
            
        }
        else
        {
   
            double δf = inputSequence.col(j)[0];
            
            double d2y = simulatedStateSequence.col(j-1)[1]*A1/(m * Vx) + simulatedStateSequence.col(j-1)[3]*(-Vx + A2/(m*Vx)) + δf * 2*Cαf/m ;
            double dy = simulatedStateSequence.col(j-1)[1];
            double d2ψ = simulatedStateSequence.col(j-1)[1] * A2/(Iz*Vx) + (A3/(Iz*Vx))*simulatedStateSequence.col(j-1)[3] + 2*lf*Cαf*δf/Iz ;
            double dψ = simulatedStateSequence.col(j-1)[3];
     
            
            MatrixXd changedVar {{dy*dt + simulatedStateSequence.col(j-1)[0]},{d2y*dt + simulatedStateSequence.col(j-1)[1]},{dψ*dt + simulatedStateSequence.col(j-1)[2]},{d2ψ*dt + simulatedStateSequence.col(j-1)[3]}};
   
            simulatedStateSequence.col(j) = changedVar;
            if(j < 50)
            {
                std::cout << "δf: " <<  δf << "\tVx:" << Vx << "\t"  << std::endl;;
                std::cout << "y': " << dy << std::endl;
                std::cout << "y'': " << d2y << std::endl;
                std::cout << "dψ: " << dψ << std::endl;
                std::cout << "d2ψ: " << d2ψ << std::endl;
                std::cout << std::string(20,'-') << std::endl;
                
            }
            
            simulatedOutputSequence.col(j) = C * simulatedStateSequence.col(j);
            
            //simulatedStateSequence.col(j) = simulatedStateSequence.col(j-1) + dt * (A * simulatedStateSequence.col(j - 1) + B * inputSequence.col(j - 1));
            //simulatedOutputSequence.col(j) = C * simulatedStateSequence.col(j);
            
            
        }
    }
    //Export Files
    
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    std::ofstream fileInputSequence(FILEPATH + "inputSequenceDynamicFile.csv");
    if (fileInputSequence.is_open())
    {
        auto transposedInputSequence =inputSequence.transpose();
        for(int i =1; i < transposedInputSequence.cols() + 1; ++i){
           std::string isComma = i == transposedInputSequence.cols() ? "" : ",";
            fileInputSequence << "x" << std::to_string(i) << isComma;
        }
        fileInputSequence << std::endl;
        fileInputSequence << transposedInputSequence.format(CSVFormat);
        fileInputSequence.close();
    }
 
    std::ofstream fileSimulatedStateSequence(FILEPATH +"simulatedStateDynamicSequence.csv");
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
 
    std::ofstream fileSimulatedOutputSequence(FILEPATH +"simulatedOutputDynamicSequence.csv");
    if (fileSimulatedOutputSequence.is_open())
    {
        auto transposedOutputSequence =simulatedOutputSequence.transpose();
        for(int i =1; i < transposedOutputSequence.cols() + 1; ++i){
           std::string isComma = i == transposedOutputSequence.cols() ? "" : ",";
            fileSimulatedOutputSequence << "x" << std::to_string(i) << isComma;
        }
        fileSimulatedOutputSequence << std::endl;
        fileSimulatedOutputSequence << transposedOutputSequence.format(CSVFormat);
        fileSimulatedOutputSequence.close();
    }
 
    std::cout << "[+] Data Saved into Files WITHOUT Linearisation: " << FILEPATH << std::endl;
    
     }
void Models::bicycleNumericalSlipDynamics(Vehicle* car){
    //MatrixXd A {{0,1,0,0},{0,-2*(Cαf + Cαr)/(m * Vx),0,-Vx -2 * (Cαf*lf - Cαr*lr)/(m*Vx)},{0,0,0,1},{0,-2 * (Cαf*lf - Cαr*lr)/(Iz*Vx),0,-2 * (Cαf*lf*lf + Cαr*lr*lr)/(Iz*Vx)}}; //Kinematics input has Vx, 2 because of 2 wheels each
   
    /*
    MatrixXd A {{0,1,0,0},{0,-(Cαf + Cαr)/(m * Vx),0,-Vx -(Cαf*lf - Cαr*lr)/(m*Vx)},{0,0,0,1},{0,-(Cαf*lf - Cαr*lr)/(Iz*Vx),0,-(Cαf*lf*lf + Cαr*lr*lr)/(Iz*Vx)}};
    MatrixXd B {{0,0,0,0},{0,Cαf/m,0,0},{0,0,0,0},{0,0,0,lf*Cαf/Iz}}; //Input Matrix, y,Vy,Yaw Angle,Yaw Rate
    MatrixXd C {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};// Output Matrix Coefficient, y,Vy,Yaw Angle,Yaw Rate
    MatrixXd D; D.resize(4,4); D.setZero();
     */

    //LINEARISE the equations to take the respective inputs
    /*
     
     αf = δ - θVf
     αr = -θVr
     θVf = (y' + lf*ψ')/Vx
     θVr = (y' - lr*ψ')/Vx
     
     m(y'' + ψ'*Vx) = Fyf + Fyr
     Iz*ψ'' = lf*Fyf - lr*Fyr
     
     
     State Variables: y, y', ψ, ψ'
     Input: δ,Vx
     
     MatrixXd A {{0,1,0,0,0},{0, , 0, ,0},{0,0,0,1,0},{0, ,0, ,0},{0,0,0,0,1}}; //Calculate Linearised functions
     MatrixXd B {{
     MatrixXd C  {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};// Output Matrix Coefficient, y,Vy,Yaw Angle,Yaw Rate
     MatrixXd D; D.resize(4,4); D.setZero();
     */
    
    //LINEARISED
    /*
    MatrixXd A {{0,1,0,0},{0,A1/m * 1.0/Vx,0,-Vx+A2/Vx},{0,0,0,1},{0,A2/(Iz*Vx),0,A3/(Iz*Vx)}};
    MatrixXd B  {{0,0,0,0},{0,Cαf/m,0,0},{0,0,0,0},{0,0,0,lf*Cαf/Iz}}; //Input Matrix, y,Vy,Yaw Angle,Yaw Rate
    MatrixXd C  {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};// Output Matrix Coefficient, y,Vy,Yaw Angle,Yaw Rate
    MatrixXd D; D.resize(4,4); D.setZero();
    std::cout << std::string(35,'-') << std::endl;
    std::cout <<"A: \n" << A << std::endl;
    std::cout << "Multiply: \n"<< dt * A << std::endl;
    A = dt * A;
    B = dt * B;
    MatrixXd x0;x0.resize(4,1);x0.setZero();
    
    x0(0,0) = 0.0; // y
    x0(1,0) = 0.0; // Vy
    x0(2,0) = 0.0; // Yaw Angle
    x0(3,0) = 0.0; // Yaw Rate

    du -> setMatrices(A,B,C,D, x0, inputSequence);
    du -> printSimulationParams();
    du -> runSimulation();
    du -> saveData("ADynamic.csv","BDynamic.csv","CDynamic.csv","DDynamic.csv","x0Dynamic.csv", "inputSequenceDynamicFile.csv", "simulatedStateDynamicSequence.csv", "simulatedOutputDynamicSequence.csv");
     */
    //----------––––----------------------------------------–––––––––––––––––––––––––––––––––––––
    
   

}
std::vector<double> Models::ackermannModel(Vehicle* car,double δf,double pAck){
    //input
    //https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
    double γ = 1.0; //Steering Ratio,Assuming 100% steering ratio
    double δack = δf/γ;
    
    //params
    double lw = car -> lw;
    double L = car -> L;
    
    //output
    double r = L/sin(δf);
    double δi = atan(L/(r-lw/2));
    double δo = δi-pAck*(δi-δack);

    return std::vector<double> {δi,δo};
}
double Models::pacejkaTireModel(double α){
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
double steeringLowPassFilter(double δfcmd, double δfcmdPrev,double cutoffFreq,double dt)
{
    double x =2 * M_PI * cutoffFreq * dt; //Common variable
    double alpha =  x/(x+1);
    δfcmd =  δfcmdPrev + alpha * (δfcmd - δfcmdPrev);
    return δfcmd;
}
void Models::test(){
    std::cout<<"From header file" << std::endl;
    
}

std::vector<double> Models::sideslipModel(Vehicle* car,double δ,double β,double r,double Vx, double φ){
    //β -> slip angle, r -> dψ ;
    double Cαf= car -> Cαf;
    double Cαr = car -> Cαr;
    double m = car -> m;
    double lf = car -> lf;
    double lr = car -> lr;
    double Iz = car -> Iz;
    
    
    double dβ = -r + Cαf/(m*Vx) * (δ - β - lf*r/Vx ) + Cαr/(m*Vx) * (-β + lr*r/Vx) + g*sin(φ)/Vx;
    double dr = lf * Cαf / Iz * (δ - β - lf*r/Vx ) - lr * Cαr /Iz * (-β + lr*r/Vx);
    return std::vector<double>{dβ,dr};
    
}

