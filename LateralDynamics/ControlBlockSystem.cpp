//
//  ControlBlockSystem.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#include "ControlBlockSystem.hpp"

ControlBlockSystem::ControlBlockSystem(){};

void ControlBlockSystem::runBlocks(ControlBlock startBlock,ControlBlock endBlock,int nSteps){
    
    ControlBlock* startPtr = &startBlock;
    ControlBlock* endPtr = &endBlock;
    ControlBlock* currBlock = startPtr; //Pointer to Starting Block
    nSteps +=1; //Allocate one extra for initial step
    
    while(nSteps){
        if(currBlock == startPtr) //Dereference pointer to check
            nSteps--;
        if(nSteps == 0 && currBlock == endPtr)
            break;
        
        //Evaluate polynomial expression
        
        //Traverse to next block
        currBlock = currBlock -> nextBlock;
    }
    
}
ControlBlock ControlBlockSystem::combineBlocks(ControlBlock A,ControlBlock B,COMBINE_STATE){
    
    ControlBlock::TransferFunction Atf = A.tf,Btf = B.tf;
    ControlBlock C = ControlBlock(); //Constructing new Object
    
    if(SERIES){
        Polynomial newNum = Atf.numPoly * Btf.numPoly;
        Polynomial newDenom = Atf.denomPoly * Btf.denomPoly;
        
        C.setTF(newNum,newDenom);
        
        //Remove reference from A to B and combine both control blocks
        

    }
    else if(ADD_PARALLEL){
        Polynomial newNum = Atf.numPoly * Btf.denomPoly + Btf.numPoly * Atf.denomPoly;
        Polynomial newDenom = Atf.denomPoly * Btf.denomPoly;
        
        C.setTF(newNum,newDenom);
    }
    else if(SUBTRACT_PARALLEL){
        Polynomial newNum = Atf.numPoly * Btf.denomPoly - Btf.numPoly * Atf.denomPoly;
        Polynomial newDenom = Atf.denomPoly * Btf.denomPoly;
        
        C.setTF(newNum,newDenom);
    }
    else
        std::cout << "[-] No combination of blocks is allocated. Returning empty block between A and B." << std::endl;
    
    //Remove reference from A to B and combine both control blocks
    C.nextBlock = B.nextBlock;
    C.prevBlock = A.prevBlock;
    return C;
}


