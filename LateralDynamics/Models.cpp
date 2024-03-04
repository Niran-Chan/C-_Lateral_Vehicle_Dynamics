//
//  Dynamics.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/10/23.
//
// Use this file for Dynamic Model Functions

#include "Models.hpp"
#include "HelperFunctions.hpp"

//Environment Parameters
extern const double g = 9.81;

MatrixXd Models::bicycleKinematicsStep(Vehicle* car,double V,double ψ,double steerCommandPercent,double steerRatio,double dt){

    double δfmax = car -> δfmax; //Max angle of Vehicle constricted by Ackermann
    double δrmax = car -> δrmax;
    double lw = car -> lw;
    double L = car -> L;
    double lr = car -> lr;
    double lf = car -> lf;

    double δf,dψ,β,dx,dy;
    double δr = 0.0; //Assuming 0 Steering angle of rear wheel
    MatrixXd finalVar;finalVar.resize(4,1);finalVar.setZero();

    δf = steerCommandPercent/100.0 * δfmax; //Steering Angle, assuming 1:1 steering ratio
    β = atan((lf * tan(δr) + lr * tan(δf)) / L); //Slip Angle (Difference between Heading Direction and Current Vehicle Direction, radians)
    
    dx = V * cos(ψ + β); //dx, which is Vx
    dy = V * sin(ψ + β); //dy, which is Vy
    dψ = V/L * cos(β) * (tan(δf) - tan(δr)); //Steering Angle -> Yaw Rate, Kinematic Interpretation
   
    
    finalVar(0,0) = dx;
    finalVar(1,0) = dy;
    finalVar(2,0) = dψ;
    finalVar(3,0) = β;

    return finalVar;
}

void Models::bicycleSlipDynamicsSimulation(Vehicle* car){

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
    
    HelperFunctions::toCsv(inputSequence, "inputSequenceDynamicsONLYFile.csv", FILEPATH);
    HelperFunctions::toCsv(simulatedStateSequence, "simulatedStateDynamicsONLYSequence.csv", FILEPATH);
    HelperFunctions::toCsv(simulatedOutputSequence,"simulatedOutputDynamicsONLYSequence.csv",FILEPATH);
 
    std::cout << "[+] Bicycle Slip Dynamic Simulation ONLY: " << FILEPATH << std::endl;
    
     }
MatrixXd Models::bicycleSlipDynamicsStep(Vehicle* car,MatrixXd sequence,double steerCommandPercent,double Vx,double φ,double dt){
    double lf = car -> lf;
    double lr = car -> lr;
    double m = car -> m;
    double Iz = car -> Iz;
    double Cαf = car -> Cαf;
    double Cαr = car ->Cαr;
    double δfmax = car -> δfmax;
    
    double δf = steerCommandPercent/100.0 * δfmax; //Steering Angle, assuming 1:1 steering ratio
    //Constants
    double A1 = -(Cαf + Cαr);  //Include coefficient 2 if we are including more wheels
    double A2= -(Cαf*lf - Cαr*lr);
    double A3 = -(Cαf*lf*lf + Cαr*lr*lr);
    
    //Calculation
    double d2y = sequence(1,0)*A1/(m * Vx) + sequence(3,0)*(-Vx + A2/(m*Vx)) + δf * 2*Cαf/m + m*sin(φ);
    double dy = sequence(1,0);
    double d2ψ = sequence(1,0) * A2/(Iz*Vx) + (A3/(Iz*Vx))*sequence(3,0) + 2*lf*Cαf*δf/Iz ;
    double dψ = sequence(3,0);

    MatrixXd du{{dy},{d2y},{dψ},{d2ψ}};
    return du;
    
}
MatrixXd Models::ackermannModel(Vehicle* car,double steerCommandPercent,double pAck){
    //input
    //https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
    double δfmax = car -> δfmax;
    double γ = 1.0; //Steering Ratio,Assuming 100% steering ratio
    double δf = steerCommandPercent/100.0 * δfmax;
    
    double δack = δf/γ;
    
    //params
    double lw = car -> lw;
    double L = car -> L;
    
    //output
    double r = L/sin(δf);
    double δi = atan(L/(r-lw/2));
    double δo = δi-pAck*(δi-δack);

    MatrixXd finalValues{{δi},{δo}};
    return finalValues;
}
MatrixXd Models::pacejkaTireModel(Vehicle* car, double steerCommandPercent,double ψ){
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
    double δfmax = car -> δfmax;
    double δf = steerCommandPercent/100.0 * δfmax; //Steering Angle, assuming 1:1 steering ratio
    double α = δf - ψ;
    double B = 100;
    double C = 1.5;
    double D = 15000;
    double E = 0.1;
    
    double Fy = D * sin(C * atanl(B * α - E * (B * α - atanl(B * α))));
    
    MatrixXd results;results.resize(2,1);results.setZero();
    results(0,0) = Fy;
    results(1,0) = α;
    return results;
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

