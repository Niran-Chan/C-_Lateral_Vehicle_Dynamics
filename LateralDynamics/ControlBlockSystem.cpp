//
//  ControlBlockSystem.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#include "ControlBlockSystem.hpp"

ControlBlockSystem::ControlBlockSystem(){};

void ControlBlockSystem::runBlocks(ControlBlock* startBlock,ControlBlock* endBlock,int nSteps){
    ControlBlock* currBlock = startBlock;
    
    nSteps +=1; //Allocate one extra for initial step
    
    while(nSteps){
        if(currBlock == startBlock)
            nSteps--;
        if(nSteps == 0 && currBlock == endBlock)
            break;
        
        //Evaluate polynomial expression
        
        //Traverse to next block
        currBlock = currBlock -> nextBlock;
    }
}
ControlBlock* ControlBlockSystem::addBlocks(ControlBlock* A,ControlBlock* B){
    ControlBlock::TransferFunction Atf = A -> tf,Btf = B -> tf;
  //  ControlBlock::TransferFunction newNum = Atf.numPoly * Btf.denomPoly + Btf.numPoly * Atf.denomPoly;
    ControlBlock* C = new ControlBlock();
    return C;
}
