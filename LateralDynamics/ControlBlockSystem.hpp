//
//  ControlBlockSystem.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#ifndef ControlBlockSystem_hpp
#define ControlBlockSystem_hpp

#include <stdio.h>
#include "LTIBlock.hpp"
class ControlBlockSystem{
public:
    //States of combining blocks
    enum COMBINE_STATE{
        ADD_PARALLEL,SUBTRACT_PARALLEL,SERIES
    };
    ControlBlockSystem();
    
    void runBlocks(ControlBlock startBlock,ControlBlock endBlock,int nSteps);
    ControlBlock combineBlocks(ControlBlock A,ControlBlock B,COMBINE_STATE);
    
};
#endif /* ControlBlockSystem_hpp */
