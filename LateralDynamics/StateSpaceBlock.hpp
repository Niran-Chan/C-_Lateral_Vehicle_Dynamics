//
//  StateSpaceBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//

#ifndef StateSpaceBlock_hpp
#define StateSpaceBlock_hpp

#include <stdio.h>
#include "ControlBlock.hpp"
#include "HelperFunctions.hpp"
#include "Eigen/Dense"

class StateSpaceBlock : public ControlBlock{
public:
    //Public Members
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
    Eigen::MatrixXd x0;
    Eigen::MatrixXd inputSequence;
    Eigen::MatrixXd state;
    Eigen::MatrixXd output;

    StateSpaceBlock(); //Default Constructor
    StateSpaceBlock(Eigen::MatrixXd A_,Eigen::MatrixXd B_,Eigen::MatrixXd C_,Eigen::MatrixXd D_,Eigen::MatrixXd x0_,Eigen::MatrixXd inputSequence_);
    
};

#endif /* StateSpaceBlock_hpp */
