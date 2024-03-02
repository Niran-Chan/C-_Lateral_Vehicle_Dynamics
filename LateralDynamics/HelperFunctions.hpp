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
#include <iostream>
#include <fstream>
#include "Eigen/Dense"


namespace HelperFunctions{

/*!
 Matrix to CSV
 \param fileToSave the matrix to save
 \param fileName Name of the file to be saved as
 \param filePath File Path
 */
void toCsv(Eigen::MatrixXd fileToSave,std::string fileName,std::string filePath);
/*!
 CSV to Matrix
 \param fileToOpen FilePath to File
 \param headers headers to find
 \return Transposed matrix into following format: (Number of headers,Total number of data )
 */
Eigen::MatrixXd fromCsv(std::string fileToOpen,std::vector<std::string> headers);
}

#endif /* HelperFunctions_hpp */

