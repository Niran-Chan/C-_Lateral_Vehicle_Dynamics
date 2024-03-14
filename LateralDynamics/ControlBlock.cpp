//
//  ControlBlock.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#include "ControlBlock.hpp""

ControlBlock::ControlBlock(){
    this -> nextBlock = NULL;
}
ControlBlock::ControlBlock(Polynomial* num,Polynomial* denom){
    this -> tf.numPoly = num;
    this -> tf.denomPoly = denom;
    this -> nextBlock = NULL;
}
ControlBlock::ControlBlock(Polynomial* num,Polynomial* denom,ControlBlock* nextBlock){
    this -> tf.numPoly = num;
    this -> tf.denomPoly = denom;
    this -> nextBlock = nextBlock;
}

