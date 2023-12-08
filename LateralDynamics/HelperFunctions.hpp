//
//  HelperFunctions.hpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 24/10/23.
//

#ifndef HelperFunctions_hpp
#define HelperFunctions_hpp

#include <stdio.h>
#include <string>
#include <vector>

namespace HelperFunctions{
void toCsv(std::string fileName,std::vector<std::string> headers,std::vector<double> x,std::vector<double> y);
void fromCsv(std::string);
void test();
}

#endif /* HelperFunctions_hpp */

