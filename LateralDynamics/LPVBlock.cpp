//
//  LPVBlock.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//
//Logic from https://www.mathworks.com/help/control/ug/linear-parameter-varying-models.html

#include "LPVBlock.hpp"



LPVBlock::LPVBlock(StateSpaceBlock ss,Eigen::MatrixXd ranges){
    //Grid of State Space Models
    //Each row in ranges represents starting,ending and increment value of the various variables
    //Outputs start to cluster together to form a domain
    //Start with 2 Variables

    int x = ranges(0,1) - ranges(0,0), y = ranges(1,1) - ranges(1,0);
    grid.resize(x,y);
    grid.setZero();
    
    int xCnt = 0;
    for(int i = ranges(0,0);i <= ranges(0,1);i+=ranges(0,2)){
        int yCnt = 0;
        for(int j = ranges(1,0);j <= ranges(1,1);j += ranges(1,2)){
            
            SimulateSystem StateSpaceSys; //Instance of State Space Simulation
            StateSpaceSys.setMatrices(ss.A, ss.B, ss.C, ss.D, ss.x0, ss.inputSequence);
            
            //Run simulation with current parameters, StateSpace Block updated
            StateSpaceSys.runStep(ss);
            
            //get results and set grid point
            GridPoint gridpnt;
            gridpnt.var1 = i;
            gridpnt.var2 = j;
            gridpnt.output = ss.output;
            
            grid(xCnt,yCnt) = gridpnt; //Assign matrix location with grid point
            yCnt++;
        }
        xCnt++;
    }
    
};
std::pair<LPVBlock::GridPoint,LPVBlock::GridPoint> LPVBlock::getGridPoint(double target){
    //Use binary search to find closest approximation
    int x =0,low = 0,high = grid.rows();
    
    bool found = false;
    std::pair<GridPoint,GridPoint> res;
    while(!found){
        int mid = (high+low)/2;
        if(grid(mid,0).var1 < target)
            low = mid;
        else if (grid(mid,0).var1 == target)
        {
            return {grid(mid,0),grid(mid,0)};
        }
            
    }
    
    return res;
}
Eigen::MatrixXd LPVBlock::getLinearEstimation(double x){
    //given value of one variable, i can now estimate values of other variables
    //Find closest grid point on grid
    auto closestGridPoints = getGridPoint(x);
    MatrixXd res;res.resize(2,1);
    if(&closestGridPoints.first == &closestGridPoints.second)
    {
        res(0,0) = closestGridPoints.first.var1;
        res(1,0) = closestGridPoints.second.var2;
    }
    return res;
}

void LPVBlock::printGrid(){
    std::cout << grid << std::endl;
}
