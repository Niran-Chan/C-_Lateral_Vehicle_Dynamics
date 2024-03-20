//
//  LPVBlock.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//

#ifndef LPVBlock_hpp
#define LPVBlock_hpp

#include <stdio.h>
#include <map>

#include "ControlBlock.hpp"
#include "StateSpaceBlock.hpp"
#include "SimulateSystem.hpp"
#include "Eigen/Dense"
#include "SharedStructs.h"
#include "KDTree.hpp"

class LPVBlock : public ControlBlock{

public:
    //Grid based LPV Models
    
    //std::map<double,GridPoint> grid;
    KDTree grid; //KD Binary Space Partition Tree for efficient neighbour search, Instance of Tree
    Node* root;
    
    //Custom Constructor for Block
    LPVBlock(); //Empty default constructor
    LPVBlock(StateSpaceBlock ss,Eigen::MatrixXd ranges);
    void addModel(std::vector<double> points,StateSpaceBlock ss); //Populate Current Grid
    void printGrid();
    
    std::pair<GridPoint,GridPoint> getGridPoint(std::vector<double> targetPoints);
    StateSpaceBlock getLinearEstimation(std::vector<double> targetPoints);
};
#endif /* LPVBlock_hpp */
