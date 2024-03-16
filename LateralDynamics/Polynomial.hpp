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
#include "Eigen/Dense"

class Polynomial{
public:
    
    Eigen::ArrayXd polyPairs;
    
    Polynomial(); //Default Constructor
    Polynomial(Eigen::ArrayXd coeffs);
    
    
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
