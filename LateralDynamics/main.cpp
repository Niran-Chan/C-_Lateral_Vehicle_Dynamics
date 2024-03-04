//
//  main.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath 
//

#include <iostream>
#include <vector>
#include <string>

//Header Files
#include "Models.hpp"
#include "Vehicle.hpp"
#include "HelperFunctions.hpp"
#include "SimulateSystem.hpp"

const std::string inputSequencePath = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv";
const std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";

int main(int argc, const char * argv[]) {
    
    //Initialise Initial Conditions
    Vehicle* car = new Vehicle();
    MatrixXd initialConditions;initialConditions.resize(3,1);initialConditions.setZero();
    initialConditions(0,0) = -324.429047; //X
    initialConditions(1,0) = 58.0676765;   //Y
    initialConditions(2,0) = -2.66446209;  //ψ
    
    //Import Data
    MatrixXd inputSequence = HelperFunctions::fromCsv(inputSequencePath,std::vector<std::string>{"ego_vel_mts_sec","steer_cmd_percent"}); //Velocity,Steering Angle

    
    //Initial Parameters
    double dt = 0.04;
    int timeSamples = inputSequence.cols();
    double x = initialConditions(0,0);
    double y = initialConditions(1,0);
    double ψ = initialConditions(2,0);

    double dx,dy,dψ,β;

    //Final Matrix is {x,y,ψ}
    MatrixXd finalMatrix;finalMatrix.resize(3,timeSamples);finalMatrix.setZero(); //{x,y,ψ} from Relevant Model
    MatrixXd dynamicsResults;dynamicsResults.resize(4,timeSamples);dynamicsResults.setZero(); //{x,y,ψ} from Dynamics Model
    MatrixXd kinematicsResults;kinematicsResults.resize(4,timeSamples);kinematicsResults.setZero(); //{x,y,ψ} from Kinematics Model
    MatrixXd pacejkaValues;pacejkaValues.resize(2,timeSamples);pacejkaValues.setZero();
    MatrixXd ackermannValues;ackermannValues.resize(2,timeSamples);ackermannValues.setZero();
    finalMatrix(0,0) = x;
    finalMatrix(1,0) = y;
    finalMatrix(2,0) = ψ;
    
    std::cout << "Initial Conditions:\n " << finalMatrix(0,0) << ","<< finalMatrix(1,0) <<"," <<finalMatrix(2,0)<< std::endl;
    
    
    for(int i =1;i < timeSamples; ++i){
        double V = inputSequence(0,i);
        double steerCommandPercent = inputSequence(1,i);
        
        //Kinematics
        if(i == 1)
            std::cout << "[+] Calculating Kinematics" << std::endl;
        kinematicsResults.col(i) = Models::bicycleKinematicsStep(car,V,ψ,steerCommandPercent, 1.0, dt);
        
        dx = kinematicsResults(0,i);
        dy = kinematicsResults(1,i);
        dψ = kinematicsResults(2,i);
        β = kinematicsResults(3,i);
        
        //Slip Dynamics
        if(i == 1)
            std::cout << "[+] Calculating Slip Dynamics" << std::endl;
        if(V > 10.0)
            dynamicsResults.col(i)= Models::bicycleSlipDynamicsStep(car,dynamicsResults.col(i-1),steerCommandPercent,V*cos(ψ),0.0,dt);
        else
            dynamicsResults.col(i) = dynamicsResults.col(i-1);
        dynamicsResults.col(i) *= dt;
        
        //Ackermann Model
        ackermannValues.col(i) = Models::ackermannModel(car, steerCommandPercent, 1.0); //Returns inner and outer angles during Steady-State Cornering
        
        //Pacejka Tire Model
        
        pacejkaValues.col(i) = Models::pacejkaTireModel(car,steerCommandPercent,ψ); //Slip Angle,Lateral Force Experienced by Vehicle
        
        ψ = ψ + dψ*dt;
        x = x + dx*dt;
        y = y + dy*dt;
       
        //Lateral Dynamics
        
        //Final Matrix
        finalMatrix(0,i) = x;
        finalMatrix(1,i) = y;
        finalMatrix(2,i) = ψ;
        
        
    }
    std::cout << "[+] Saving Files..." << std::endl;
    HelperFunctions::toCsv(kinematicsResults, "Kinematics.csv", FILEPATH);
    HelperFunctions::toCsv(dynamicsResults, "SlipDynamics.csv", FILEPATH);
    HelperFunctions::toCsv(pacejkaValues, "PacejkaTireValues.csv", FILEPATH);
    HelperFunctions::toCsv(ackermannValues, "AckermannValues.csv", FILEPATH);
    HelperFunctions::toCsv(finalMatrix, "PositionalValues.csv", FILEPATH);
    
    
    
    return 0;
}
