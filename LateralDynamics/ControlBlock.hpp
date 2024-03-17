//
//  ControlBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 16/3/24.
//

#ifndef ControlBlock_hpp
#define ControlBlock_hpp

#include <stdio.h>
#include "Polynomial.hpp"

class ControlBlock{
    //DataType TransferFunction
public: //Change later to protected for derived classes
    
    //Linked List Methodology
    ControlBlock* nextBlock = NULL;
    ControlBlock* prevBlock = NULL;
    
    
    struct TransferFunction{
        Polynomial numPoly;
        Polynomial denomPoly;
    } tf; //Transferfunction of type TransferFunction
    
    
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
    
    //Methods
    
    void setTF(Polynomial newNum,Polynomial newDenom);
    void printTF();
    std::complex<double> evaluateTF(std::complex<double> const& s);
};

#endif /* ControlBlock_hpp */
