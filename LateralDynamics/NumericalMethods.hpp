//
//  NumericalMethods.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 12/3/24.
//

#ifndef NumericalMethods_hpp
#define NumericalMethods_hpp

#include <stdio.h>
#include <math.h>
#include "Eigen/Dense"

namespace NumericalMethods{
/*!
    \param x value to factorial
    \return factorial of x
*/

double factorial(int x);
/*!
    Taylor series expansion of  ie. (x+5)^n
    \param x base value in equation
    \param n Order of expansion
 */
double taylor(double x, int n);
double RK2();
double RK4();
double forwardEuler();

}

#endif /* NumericalMethods_hpp */
