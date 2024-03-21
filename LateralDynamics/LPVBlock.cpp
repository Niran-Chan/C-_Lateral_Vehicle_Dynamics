//
//  LPVBlock.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 17/3/24.
//
//Logic from https://www.mathworks.com/help/control/ug/linear-parameter-varying-models.html

#include "LPVBlock.hpp"


LPVBlock::LPVBlock(){}
LPVBlock::LPVBlock(StateSpaceBlock ss,Eigen::MatrixXd ranges){
    //Grid of State Space Models
    //Each row in ranges represents starting,ending and increment value of the various variables
    //Outputs start to cluster together to form a domain
    //Start with just 2 variables

    /*
    int x = ranges(0,1) - ranges(0,0);
   
    
    int xCnt = 0;
    for(int i = ranges(0,0);i <= ranges(0,1);i+=ranges(0,2)){
        int yCnt = 0;
            
        SimulateSystem StateSpaceSys; //Instance of State Space Simulation
        StateSpaceSys.setMatrices(ss.A, ss.B, ss.C, ss.D, ss.x0, ss.inputSequence);
        
        GridPoint gridpnt;
        gridpnt.var1 = i;
        //gridpnt.output = ss.output;
        
        grid[xCnt] = gridpnt; //Assign matrix location with grid point

        xCnt++;
    }
    */
};
std::pair<GridPoint,GridPoint> LPVBlock::getGridPoint(std::vector<double> targetPoints){
    
    /*
    std::pair<LPVBlock::GridPoint,LPVBlock::GridPoint> res;
    
    //Call Map lower and upper bounds
 
    if(grid.find(target) != grid.end()){
        return {grid[target],grid[target]};
    }
    //Prev Gridpoint
    auto prevGridIt = grid.upper_bound(target); //Pointer to Iterator,pair{Key,Val}
    auto nextGridIt = grid.lower_bound(target);
    if(prevGridIt != grid.end() && nextGridIt != grid.end())
        return {prevGridIt -> second,nextGridIt -> second};
    
    return res;
     */
    
    //Traverse grid to find nearest neighbour

    GridPoint gridempty;

    Node* nd = grid.findNeighbour(root, targetPoints);
    if(nd == NULL || !nd -> left && !nd -> right)
        return {gridempty,gridempty};
    if(!(nd -> left) || !(nd -> right)) //one of the point on left or right is NULL
    {
        if(!nd -> left)
            return {nd -> right -> gridpnt,nd -> right -> gridpnt};
        if(!nd -> right)
            return {nd -> left -> gridpnt,nd -> left -> gridpnt};
    }
    return {nd -> left -> gridpnt,nd -> right -> gridpnt};
    }
StateSpaceBlock LPVBlock::getLinearEstimation(std::vector<double> targetPoints){
    //given value of one variable, i can now estimate values of other variables
    //Find closest grid point on grid
    auto closestGridPoints = getGridPoint(targetPoints);
    auto p1 = closestGridPoints.first;
    auto p2 = closestGridPoints.second;
    
    StateSpaceBlock res;
    if(p1.points.size() == 0 && p2.points.size() == 0){
        std::cout << "No linear estimation performed as out of range" << std::endl;
        return res;
    }
    if(p1.points == p2.points) //Same grid point
    {
        res = closestGridPoints.first.ss;
    }
    else{
        //Conduct Linear Approximation Operation (1D)
        auto gradient = (p2.ss - p1.ss) * (1.0/(p2.points[0] - p1.points[0]));
        res = p1.ss + (gradient * (targetPoints[0] - p1.points[0]));
    }
    return res;
}

void LPVBlock::printGrid(){
    /*
    for(std::map<double,LPVBlock::GridPoint>::iterator i = grid.begin(); i != grid.end();++i)
    {
        std::cout << "key: " << i -> first << "Matrix A of gridpoint: " << i -> second.ss.A<< std::endl;
    }
     */
    grid.printTree(root);
    }
void LPVBlock::addModel(std::vector<double> points,StateSpaceBlock ss){
    GridPoint gridpnt;
    //gridpnt.var1 = x;
    gridpnt.points = points;
    gridpnt.ss = ss;
    //grid[x] = gridpnt; //Assign matrix location with grid point
    root = grid.insert(root,gridpnt);
}
