//
//  SharedStructs.h
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 20/3/24.
//

#ifndef SharedStructs_h
#define SharedStructs_h
#include "StateSpaceBlock.hpp"
#include <vector>

//LPVBlock
struct GridPoint{
    double var1; //Soon to be replaced
    std::vector<double> points;
    StateSpaceBlock ss;
};

//KD Tree
struct Node
{
    std::vector<double> point; // To store k dimensional point
    Node *left, *right;
    GridPoint gridpnt;
};


#endif /* SharedStructs_h */
