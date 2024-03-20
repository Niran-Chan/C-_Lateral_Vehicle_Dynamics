//
//  KDTree.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 20/3/24.
//

#ifndef KDTree_hpp
#define KDTree_hpp

#include <iostream>
#include <vector>
#include "SharedStructs.h"


// A structure to represent node of kd tree
class KDTree{
public:
    

    //Constructors
    KDTree();
    
    //Methods
    struct Node* newNode(GridPoint gridpnt);
    Node* insertRec(Node* root, GridPoint gridpnt, long long int depth);
    Node* insert(Node *root, GridPoint gridpnt);
    bool arePointsSame(std::vector<double> p1, std::vector<double> p2);
    bool searchRec(Node* root, std::vector<double> point, long long int depth);
    bool search(Node* root, std::vector<double> point);
    Node* printTree(Node* root);
    Node* findNeighbourRec(Node* root,std::vector<double>point,long long int depth);
    Node* findNeighbour(Node* root,std::vector<double> targetPoints);
    
};

#endif /* KDTree_hpp */
