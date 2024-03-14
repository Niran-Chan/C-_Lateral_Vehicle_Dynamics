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
#include "Polynomial.hpp"
#include "ControlBlock.hpp"

//Dataset(s)
const std::string inputSequencePath = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/testdata/lat_logCSVFile-202312061600.csv";
const std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";

void runSimulation(){
    //Initialise Vehicle Class Instance
    Vehicle* car = new Vehicle();
    
    //Initialise Relevant Headers
    std::vector<std::string> headers{"ego_x_mts","ego_y_mts","ego_yaw_rad","ego_vel_mts_sec","steer_cmd_percent","steer_fdback_percent","ref_curvature_1_over_mts","time_sec"};
    
    //Import Data
    MatrixXd inputSequence = HelperFunctions::fromCsv(inputSequencePath,headers); //Velocity,Steering Angle,ackermann percent,time
    
 
     //Initial Parameters
    /*
    MatrixXd initialConditions;initialConditions.resize(3,1);initialConditions.setZero();
    initialConditions(0,0) = inputSequence(0,0); //X
    initialConditions(1,0) = inputSequence(1,0);   //Y
    initialConditions(2,0) = inputSequence(2,0);  //ψ
    */
    
    int timeSamples = inputSequence.cols();
    
    std::cout << "TimeSamples : " << timeSamples << std::endl;;
    double x = inputSequence(0,0);
    double y = inputSequence(1,0);
    double ψ = inputSequence(2,0);
    double t = 0.0;
    double dtAvg = (inputSequence(7,5) - inputSequence(7,0)) / 5.0; //First 5 Time steps as Average dt value
    double dx,dy,dψ,β,dt;
    std::cout << "Number of Rows: " << inputSequence.rows() << std::endl;
    std::cout << "Number of Cols: " << inputSequence.cols() << std::endl;
    //Final Matrix is {x,y,ψ,t}
    MatrixXd positionMatrix;positionMatrix.resize(4,timeSamples);positionMatrix.setZero(); //{x,y,ψ,t} from Relevant Model
    MatrixXd dynamicsResults;dynamicsResults.resize(4,timeSamples);dynamicsResults.setZero(); //{x,y,ψ} from Dynamics Model
    MatrixXd kinematicsResults;kinematicsResults.resize(4,timeSamples);kinematicsResults.setZero(); //{x,y,ψ} from Kinematics Model
    MatrixXd pacejkaFyValues;pacejkaFyValues.resize(2,timeSamples);pacejkaFyValues.setZero(); //Assuming Ackermann provides only 2 front tires
    MatrixXd pacejkaSlipAngleValues = pacejkaFyValues;
    
    MatrixXd ackermannValues;ackermannValues.resize(2,timeSamples);ackermannValues.setZero();
    MatrixXd sideSlipValues;sideSlipValues.resize(2,timeSamples);sideSlipValues.setZero();
    
   
    positionMatrix(0,0) = t;
    positionMatrix(1,0) = x;
    positionMatrix(2,0) = y;
    positionMatrix(3,0) = ψ;
    
    std::cout << "Initial Conditions:\n " << positionMatrix(0,0) << ","<< positionMatrix(1,0) <<"," <<positionMatrix(2,0)<< std::endl;
    int faultyDataCnt = 0;
    double acceptableErrorRange = dtAvg;
    std::cout << "PI:" << M_PI << std::endl;
    //Main Loop
    for(int i =1;i < timeSamples; ++i){
        
        double V = inputSequence(3,i);
        double steerCommandPercent = inputSequence(4,i);
        double pAck = inputSequence(5,i);
        double curvature = inputSequence(6,i);
        double currTime = inputSequence(7,i);
        if(currTime < 0) //Some errors have negative time
        {
            faultyDataCnt++;
            positionMatrix.col(i) = positionMatrix.col(i-1); //maintain current state
            continue;
        }
        
        dt = std::abs(currTime - t - dt) < acceptableErrorRange ? currTime - t : dtAvg; //Check if error experienced before and if yes, stick with previous dt value
       
        /*if(std::abs(currTime - t - dt) > acceptableErrorRange)
            std::cout << currTime - t << " replaced with " << dt <<" timestep" << std::endl;
        */
         //dt = currTime-t;
        t = currTime;
        
        if(std::abs(ψ) > M_PI) //Clamp range from -pi to pi
            ψ = -ψ;
        
        //Kinematics
        if(i == 1)
            std::cout << "[+] Bicycle Kinematics Included " << std::endl;
        
        kinematicsResults.col(i) = Models::bicycleKinematicsStep(car,V,ψ,steerCommandPercent, dt);

        dx = kinematicsResults(0,i);
        dy = kinematicsResults(1,i);
        dψ = kinematicsResults(2,i);
        β = kinematicsResults(3,i);
 
        
        //Slip Dynamics, Applicable for sufficient enough slip angle
        if(i == 1)
            std::cout << "[+] Bicycle Slip Dynamics Included" << std::endl;
        if(std::abs(V) > 4.0)
            dynamicsResults.col(i)= Models::bicycleSlipDynamicsStep(car,dynamicsResults.col(i-1),steerCommandPercent,V*cos(ψ),0.0,dt);
        else
            dynamicsResults.col(i) = dynamicsResults.col(i-1);
        dynamicsResults.col(i) *= dt;
        

        //Ackermann Model
        ackermannValues.col(i) = Models::ackermannModel(car, steerCommandPercent,curvature,pAck); //Returns inner and outer angles during Steady-State Cornering
        
        
        //Pacejka Tire Model
        auto pacejkaValuesMatrixPair = Models::pacejkaTireModel(car,ackermannValues.col(i),ψ); //Slip Angle,Lateral Force Experienced by Tires on Vehicle
        pacejkaFyValues.col(i) = pacejkaValuesMatrixPair.first;
        pacejkaSlipAngleValues.col(i) = pacejkaValuesMatrixPair.second;
        
        
        //Side Slip Model
        if(std::abs(V) > 0.0)
            sideSlipValues.col(i) = Models::sideslipModel(car,steerCommandPercent/100.0 * car -> δfmax,β,dψ,dx,0.0);
        
        //RK2
        /*
        double f1 = Models::bicycleKinematicsStep(car,V,ψ,steerCommandPercent,dt)(2,0);
        double f2 = Models::bicycleKinematicsStep(car,V,ψ + f1*dt/2.0,steerCommandPercent,dt)(2,0);
        
        ψ = ψ + f2*dt;
        x = x + dx*dt;
        y = y + dy*dt;
*/
        
        //RK4
        //if yaw
        auto f1 = Models::bicycleKinematicsStep(car,V,ψ,steerCommandPercent,dt);
        auto f2 = Models::bicycleKinematicsStep(car,V,ψ + dt*f1(2,0)/2.0,steerCommandPercent,dt);
        auto f3 = Models::bicycleKinematicsStep(car,V,ψ + dt*f2(2,0)/2.0,steerCommandPercent,dt);
        auto f4 = Models::bicycleKinematicsStep(car,V,ψ + dt*f3(2,0),steerCommandPercent,dt);
        
        x = x + (dt/6.0) * (f1(0,0) + 2.0 * f2(0,0) + 2.0 * f3(0,0) + f4(0,0));
        y = y + (dt/6.0) * (f1(1,0) + 2.0 * f2(1,0) + 2.0 * f3(1,0) + f4(1,0));;
        ψ = ψ + (dt/6.0) * (f1(2,0) + 2.0 * f2(2,0) + 2.0 * f3(2,0) + f4(2,0));
        //ψ = std::fmod(ψ,M_PI);
     
        //Forward Euler Scheme
        /*
        ψ = ψ + dψ*dt;
        x = x + dx*dt;
        y = y + dy*dt;
        */
        //Lateral Dynamics
        
        //Final Matrix
        positionMatrix(0,i) = t;
        positionMatrix(1,i) = x;
        positionMatrix(2,i) = y;
        positionMatrix(3,i) = ψ;
        
    }
    
    std::cout << "[-] Faulty Data Time Step Count: "<< faultyDataCnt << std::endl;
    //Altering of Matrices
    
    //Export
    std::cout << "[+] Saving Files..." << std::endl;
    HelperFunctions::toCsv(kinematicsResults, "Kinematics.csv", FILEPATH);
    HelperFunctions::toCsv(dynamicsResults, "SlipDynamics.csv", FILEPATH);
    HelperFunctions::toCsv(pacejkaFyValues, "PacejkaLateralForceValues.csv", FILEPATH);
    HelperFunctions::toCsv(pacejkaSlipAngleValues, "PacejkaSlipAngleValues.csv", FILEPATH);
    HelperFunctions::toCsv(ackermannValues, "AckermannValues.csv", FILEPATH);
    HelperFunctions::toCsv(positionMatrix, "PositionalValues.csv", FILEPATH);
    HelperFunctions::toCsv(sideSlipValues,"SideSlipDynamics.csv",FILEPATH);
    
}

int main(int argc, const char * argv[]) {
    std::vector<double> coeffs{1,2};
    int degree = 2;
    Polynomial A(coeffs,degree);
    Polynomial B(coeffs,degree);
    
    A.printPolynomial();
    B.printPolynomial();
    
    Polynomial C = A + B;
    C.printPolynomial();
    Polynomial D = A * B;
    D.printPolynomial();
    return 0;
}
