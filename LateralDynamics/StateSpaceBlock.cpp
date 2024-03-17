//
//  StateSpaceBlock.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//

#include "StateSpaceBlock.hpp"

StateSpaceBlock::StateSpaceBlock(){
    A.resize(1, 1); A.setZero(); //Size 1x1 matrix
    B.resize(1, 1); B.setZero();
    C.resize(1, 1); C.setZero();
    x0.resize(1, 1); x0.setZero();
    inputSequence.resize(1,1);inputSequence.setZero();
    state = x0;
};
StateSpaceBlock::StateSpaceBlock(Eigen::MatrixXd A_,Eigen::MatrixXd B_,Eigen::MatrixXd C_,Eigen::MatrixXd D_,Eigen::MatrixXd x0_,Eigen::MatrixXd inputSequence_){
    A = A_;
    B = B_;
    C = C_;
    D = D_;
    x0 = x0_;
    inputSequence = inputSequence_;
    state = x0;
}
