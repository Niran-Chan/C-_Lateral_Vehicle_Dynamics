//
//  ControlBlockSystem.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#ifndef ControlBlockSystem_hpp
#define ControlBlockSystem_hpp

#include <stdio.h>
#include "ControlBlock.hpp"
class ControlBlockSystem{
public:
    
    ControlBlockSystem();
    
    void runBlocks(ControlBlock* startBlock,ControlBlock* endBlock,int nSteps);
    ControlBlock* addBlocks(ControlBlock* A,ControlBlock* B);
    
};
#endif /* ControlBlockSystem_hpp */
