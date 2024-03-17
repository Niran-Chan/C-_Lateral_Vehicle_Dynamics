//
//  LPVBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//

#ifndef LPVBlock_hpp
#define LPVBlock_hpp

#include <stdio.h>
#include <unordered_map>

#include "ControlBlock.hpp"
#include "StateSpaceBlock.hpp"
#include "SimulateSystem.hpp"
#include "Eigen/Dense"

class LPVBlock : public ControlBlock{
    //Grid based LPV Models
    typedef struct GridPoint{
        double var1;
        double var2;
        Eigen::MatrixXd output;
    }
    GridPoint;
    
    Eigen::Matrix<GridPoint, Dynamic,Dynamic> grid;
    
    //Custom Constructor for Block
    LPVBlock(StateSpaceBlock ss,Eigen::MatrixXd ranges);
    void printGrid();
    
    std::pair<LPVBlock::GridPoint,LPVBlock::GridPoint> getGridPoint(double x);
    Eigen::MatrixXd getLinearEstimation(double x);
};
#endif /* LPVBlock_hpp */
