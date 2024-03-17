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
    while(!found || high >= low + 1){
        int mid = (high+low)/2;
   
        if (grid(mid,0).var1 == target)
        {
            return {grid(mid,0),grid(mid,0)};
        }
        else if(grid(mid,0).var1 < target)
            low = mid;
        else if (grid(mid,0).var1 > target)
            high = mid-1;
        else
            low++;
        
    }
    res = {grid(low,0),grid(high,0)};
    return res;
}
Eigen::MatrixXd LPVBlock::getLinearEstimation(double x){
    //given value of one variable, i can now estimate values of other variables
    //Find closest grid point on grid
    auto closestGridPoints = getGridPoint(x);
    auto p1 = closestGridPoints.first;
    auto p2 = closestGridPoints.second;
    
    MatrixXd res;res.resize(3,1);
    if(p1.var1 == p2.var1) //Same grid point
    {
        res = closestGridPoints.first.output;
    }
    else{
        //Conduct Linear Approximation Operation
   
        auto gradient = (p2.output - p1.output) * (p2.output - p2.output).inverse();
        res = p1.output + gradient * (x - p1.var1);
        
    }
    return res;
}

void LPVBlock::printGrid(){
    std::cout << grid << std::endl;
}

