//
//  ControlBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#ifndef ControlBlock_hpp
#define ControlBlock_hpp

#include <stdio.h>
#include <string>
#include <unordered_map>
#include <stack>
#include <fstream>
#include <iostream>
#include "Polynomial.hpp"

class ControlBlock{
public:
    
    //DataType TransferFunction
    
    struct TransferFunction{
        Polynomial numPoly;
        Polynomial denomPoly;
    } tf; //Transferfunction of type TransferFunction
    
    //Linked List Methodology
    ControlBlock* nextBlock = NULL;
    ControlBlock* prevBlock = NULL;
    
    
    /*!
            Default Constructor. Initialises Empty Instance
     */
    ControlBlock();
    /*!
                Overloaded Constructor. Initialise with Transfer Function
     \param tf Transfer function in s-domain
     */
    ControlBlock(Polynomial num,Polynomial denom);
    /*!
                Overloaded Constructor
        \param tf Transfer function in s-domain
        \param nextBlock Another ControlBlock instance as the next block to continue to.
     */
//Overloaded Constructor
    ControlBlock(Polynomial num, Polynomial denom, ControlBlock nextBlock);
    void setTF(Polynomial newNum,Polynomial newDenom);
};


#endif /* ControlBlock_hpp */
