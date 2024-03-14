//
//  Polynomial.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#include "Polynomial.hpp"

Polynomial::Polynomial(){}; //Default Constructor


Polynomial::Polynomial(std::vector<double> coeffs,int degree){
    int j =0;
    for(int i = degree; i >=0; --i,++j){
        std::pair<double,int> currPair = {0.0,i};
        if(j < coeffs.size())
            currPair.first = coeffs[j];
        if(currPair.first != 0.0)
            polyPairs.push_back(currPair);
    }
}

Polynomial::Polynomial(std::vector<std::pair<double,int>> polyPairs){
    this -> polyPairs = polyPairs;
}

void Polynomial::printPolynomial(){
    std::string plusSign;
    bool first = true;
    //Iterate through coefficients, assign them from largest degress
    std::string res = "";
    for(auto a : polyPairs){
        plusSign = first ? "" : "+";
        res += plusSign + std::to_string(a.first) + "s^" + std::to_string(a.second);
        first = false;
    }
    std::cout << "Polynomial: " << res << std::endl;
}

//Operator Overloads
Polynomial Polynomial::operator+(Polynomial const &b){
    auto polyA = polyPairs;
    auto polyB = b.polyPairs;
    std::vector<std::pair<double,int>> newPairs;
    int i =0,j = 0;
    while(i < polyA.size() && j < polyB.size())
    {
        if(polyA[i].second == polyB[j].second) //Same degree
        {
            newPairs.push_back({polyA[i].first + polyB[j].first,polyA[i].second});
            i++;j++;
        }
        else if(polyA[i].second > polyB[j].second)
            i++;
        
        else if(polyB[j].second > polyA[i].second)
            j++;
    }
    Polynomial polyC;
    polyC.polyPairs = newPairs;
    return polyC;
}
Polynomial Polynomial::operator-(Polynomial const&b){
    auto polyA = polyPairs;
    auto polyB = b.polyPairs;
    std::vector<std::pair<double,int>> newPairs;
    int i =0,j = 0;
    while(i < polyA.size() && j < polyB.size())
    {
        if(polyA[i].second == polyB[j].second) //Same degree
        {
            newPairs.push_back({polyA[i].first - polyB[j].first,polyA[i].second});
            i++;j++;
        }
        else if(polyA[i].second > polyB[j].second)
            i++;
        
        else if(polyB[j].second > polyA[i].second)
            j++;
    }
    Polynomial polyC;
    polyC.polyPairs = newPairs;
    return polyC;
}
Polynomial Polynomial::operator *(Polynomial const &b){
    //Multiplication affects both coefficients and degrees
    auto polyPairsA = polyPairs;
    auto polyPairsB = b.polyPairs;
    std::vector<std::pair<double,int>> polyPairsC;
    
    for(auto& pairA : polyPairsA){
        for(auto& pairB : polyPairsB){
            std::pair<double,int> pairC = {pairA.first * pairB.first,pairA.second + pairB.second};
            polyPairsC.push_back(pairC);
        }
    }
    //Some coefficients and powers might have the same degree
    //Simplify said polynomials
    std::sort(polyPairsC.begin(),polyPairsC.end(),[](auto &left, auto &right) {
        return left.second > right.second;
    });
    
    int i =0,j = 1;
    std::vector<std::pair<double,int>> polyPairsD;
    while(j < polyPairsC.size()){
        std::pair<double,int> pairD {polyPairsC[i].first,polyPairsC[i].second};
        
        while(polyPairsC[i].second == polyPairsC[j++].second)
        {
            pairD.first += polyPairsC[j].first;
        }
        
        i = j;
        j++;
        
        polyPairsD.push_back(pairD);
    }
    while(i < polyPairsD.size()){
        polyPairsD.push_back(polyPairsC[i++]);
    }
    
    Polynomial C (polyPairsD);
    return C;
}

//Base on long division

Polynomial Polynomial::operator/(Polynomial const&b){
    auto polyA = polyPairs;
    auto polyB = b.polyPairs;
    std::vector<std::pair<double,int>> newPairs;
    Polynomial polyC;
    polyC.polyPairs = newPairs;
    return polyC;
}

double Polynomial::polyParser(double s){
    //Parse out our polynomial
    double finalVal = 0.0;
    for(auto &a: polyPairs){
        finalVal += a.first * std::pow(s,a.second);
    }
    return finalVal;
}
