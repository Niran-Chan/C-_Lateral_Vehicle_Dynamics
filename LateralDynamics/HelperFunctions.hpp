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
void storeAsVector(std::vector<double> vecIn,std::vector<std::vector<double>>& vecOut);
void toCsv(std::string fileName,std::vector<std::string> headers,std::vector<std::vector<double>> x);
void fromCsv(std::string);
void test();
}

#endif /* HelperFunctions_hpp */

