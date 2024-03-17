//
//  ControlBlock.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 16/3/24.
//

#include "ControlBlock.hpp"

ControlBlock::ControlBlock(){}; //Default Constructor
ControlBlock::ControlBlock(Polynomial num,Polynomial denom){
    this -> tf.numPoly = num;
    this -> tf.denomPoly = denom;
    this -> nextBlock = NULL;
}
ControlBlock::ControlBlock(Polynomial num,Polynomial denom,ControlBlock nextBlock){
    this -> tf.numPoly = num;
    this -> tf.denomPoly = denom;
    this -> nextBlock = &nextBlock;
}
void ControlBlock::setTF(Polynomial newNum, Polynomial newDenom){
    tf.numPoly = newNum;
    tf.denomPoly = newDenom;
}
void ControlBlock::printTF(){
    std::cout << "\n-----Transfer Function-----\n\n";
    tf.numPoly.printPolynomial();
    std::cout << "----------------------\n";
    tf.denomPoly.printPolynomial();
    std::cout << "\n\n------------------------------\n" << std::endl;
    
}

std::complex<double> ControlBlock::evaluateTF(std::complex<double> const& s){
    auto num = tf.numPoly.polyParser(s);
    auto denom = tf.denomPoly.polyParser(s);
    return num/denom;
}
