//
//  HelperFunctions.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 24/10/23.
//

#include "HelperFunctions.hpp"
#include "SimulateSystem.hpp"


//Basic CSV implementation
void HelperFunctions::toCsv(MatrixXd fileToSave,std::string fileName,std::string filePath){
    
    const IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
 
    std::ofstream fileStream(filePath + fileName);
    std::cout << "File Path: " << filePath + fileName << std::endl;
    if (fileStream.is_open())
    {
        auto transposedFile=fileToSave.transpose();
        
        for(int i =1; i < transposedFile.cols() + 1; ++i){
           std::string isComma = i == transposedFile.cols() ? "" : ",";
            fileStream << "x" << std::to_string(i) << isComma;
        }
        
        fileStream << std::endl;
        fileStream << transposedFile.format(CSVFormat);
        fileStream.close();
        std::cout << "[+] Data Saved into file using helper functions: " << filePath << std::endl;
    }
    else
        std::cout << "[-] Data unable to be saved : " << filePath << std::endl;

 
}
Eigen::MatrixXd HelperFunctions::fromCsv(std::string fileToOpen,std::vector<std::string> headers)
{
        //Same as SimulateSystem Method
       std::vector<double> matrixEntries;
       std::ifstream matrixDataFile(fileToOpen);
    
       // this variable is used to store the row of the matrix that contains commas
       std::string matrixRowString;
    
       // this variable is used to store the matrix entry;
       std::string matrixEntry;
    
       // this variable is used to track the number of rows
       int matrixRowNumber = 0;
       int nElems =0; //number of elements processed
       int validElems = 0; //number of elements that are valid and included
       int localCols = 0; //current local col
       
       std::unordered_map<std::string,std::vector<double>> headerMap;
       std::unordered_map<double,std::string> colMap;
       
       while (std::getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
       {
           std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
           localCols = 0;
           while (std::getline(matrixRowStringStream, matrixEntry,',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
           {
               
              
                   if(matrixRowNumber == 0) //Initialise Header Map first
                   {
                      // std::cout << "Matrix Entry Value : " << matrixEntry << std::endl;
                       headerMap[matrixEntry] = {};
                       colMap[localCols] = matrixEntry; //Key: Column Number, Value: Header name
                   }
           
               else{
                   std::string currHeader = colMap[localCols];
                   headerMap[currHeader].push_back(std::stod(matrixEntry)); //push back column value
                  
                   }
               nElems ++;
               localCols++;
               }
           matrixRowNumber++; //update the column numbers
       }
       
    double initialValidElems = 0.0;
       for(auto &header : headers){
           std::string result = headerMap.find(header)!=headerMap.end() ? "yes" : "no";
           std::cout << "Existence of header " << header << " : "<< result  << std::endl;
           for(auto val : headerMap[header]){
               matrixEntries.push_back(val);
               validElems++;
           }
           initialValidElems = validElems; //For Debugging if unequal number of cols
       }
    double cols = validElems/headers.size();
    //MUST BE ROWMAJOR!
       return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> (matrixEntries.data(),headers.size(),cols);
}


//External Function Template Instantiation
//template <double,double> void toCsv();
