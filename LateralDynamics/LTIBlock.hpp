//
//  LTIBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#ifndef LTIBlock_hpp
#define LTIBlock_hpp

#include <stdio.h>
#include <string>
#include <unordered_map>
#include <stack>
#include <fstream>
#include <iostream>
#include "Polynomial.hpp"
#include "ControlBlock.hpp"

class LTIBlock : public ControlBlock {
public:
    //Inherit Constructors
    using ControlBlock::ControlBlock;
    
    //Inherit Control Block Methods
};


#endif /* LTIBlock_hpp */
