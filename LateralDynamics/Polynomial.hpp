//
//  Polynomial.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 14/3/24.
//

#ifndef Polynomial_hpp
#define Polynomial_hpp

#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>

class Polynomial{
public:
    
    std::vector<std::pair<double,int>> polyPairs;
    
    Polynomial(); //Default Constructor
    Polynomial(std::vector<double> coeffs,int degree);
    Polynomial(std::vector<std::pair<double,int>> polyPairs);
    
    
    //Methods
    void printPolynomial();
    double polyParser(double s);
    
    
    //Operator Overloads
    Polynomial operator+(Polynomial const& b);
    Polynomial operator-(Polynomial const&b);
    Polynomial operator*(Polynomial const &b);
    Polynomial operator/(Polynomial const &b);
   
    
};


#endif /* Polynomial_hpp */
