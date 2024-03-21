//
//  KDTree.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 20/3/24.
// Imported from https://www.geeksforgeeks.org/search-and-insertion-in-k-dimensional-tree/ , modified

#include "KDTree.hpp"
 
const int k = 1; //1 Dimensional, can change in the future
 
KDTree::KDTree(){};
// A method to create a node of K D tree
Node* KDTree::newNode(GridPoint gridpnt)
{
    struct Node* temp = new Node;
 
    for (int i=0; i<k; i++)
       temp->point.push_back(gridpnt.points[i]);
 
    temp->left = temp->right = NULL; //Connect no nodes yet
    temp -> gridpnt = gridpnt; //Assign gridpnt to node
    return temp;
}
 
// Inserts a new node and returns root of modified tree

Node* KDTree::insertRec(Node *root, GridPoint gridpnt, long long int depth)
{
    if (root == NULL || root -> point.size() == 0)
       return newNode(gridpnt);
 
    // Calculate current dimension (cd) of comparison
    long long int cd = depth % k;
 
    // Compare the new point with root on current dimension 'cd'
    // and decide the left or right subtree

    if (gridpnt.points[cd] < (root->point[cd]))
        root->left  = insertRec(root->left, gridpnt, depth + 1);
    else
        root->right = insertRec(root->right, gridpnt, depth + 1);
 
    return root;
}
 
// Function to insert a new point with given point in
// KD Tree and return new root. It mainly uses above recursive
// function "insertRec()"
Node* KDTree::insert(Node *root,GridPoint gridpnt)
{
    return insertRec(root, gridpnt, 0);
}
 
// A utility method to determine if two Points are same
// in K Dimensional space
bool KDTree::arePointsSame(std::vector<double> p1, std::vector<double> p2)
{
    // Compare individual pointinate values
    for (int i = 0; i < k; ++i)
        if (p1[i] != p2[i])
            return false;
 
    return true;
}
 
// Searches a Point represented by " in the K D tree.
// The parameter depth is used to determine current axis.
bool KDTree::searchRec(Node* root, std::vector<double> point, long long int depth)
{
    // Base cases
    if (root == NULL)
        return false;
    if (arePointsSame(root->point, point))
        return true;
 
    // Current dimension is computed using current depth and total
    // dimensions (k)
    long long int cd = depth % k;
 
    // Compare point with root with respect to cd (Current dimension)
    if (point[cd] < root->point[cd])
        return searchRec(root->left, point, depth + 1);
 
    return searchRec(root->right, point, depth + 1);
}
 
// Searches a Point in the K D tree. It mainly uses
// searchRec()
bool KDTree::search(Node* root, std::vector<double> point)
{
    // Pass current depth as 0
    return searchRec(root, point, 0);
}
Node* KDTree::findNeighbourRec(Node* root,std::vector<double>point,long long int depth){
    
    // Base cases
    if (root == NULL) //End of Tree
        return root;
    
    if (arePointsSame(root->point, point))
        return root;
    
    // Current dimension is computed using current depth and total
    // dimensions (k)
    long long int cd = depth % k;
    
    // Compare point with root with respect to cd (Current dimension)
    Node* res;
    
    //If we found actual node, return it. If not, return node that is closest to reaching
    if (point[cd] < root->point[cd])
    { res = findNeighbourRec(root->left, point, depth + 1);
        if(res == NULL)
            return root;
        return res;
    }
    res = findNeighbourRec(root->right, point, depth + 1);
    if(res == NULL)
        return root;
    return res;
}
Node* KDTree::findNeighbour(Node* root,std::vector<double> targetPoints){
    return findNeighbourRec(root,targetPoints,0);
}
Node* KDTree::printTree(Node* root){
    std::queue<Node*> q;
    q.push(root);
    int depth =0 ;
    //BFS
    while(!q.empty()){
        auto currNode = q.front();
        q.pop();
        std::cout <<"x: " <<currNode -> point[0] << ",";
        if(q.empty()) //reached end of current Depth
        {std::cout << "\tDepth: " << depth << "\n|\n";
            depth++;
        }
        
        if(currNode -> left)
            q.push(currNode -> left);
        if(currNode -> right)
            q.push(currNode -> right);
        
    }
    return root;
    
}
