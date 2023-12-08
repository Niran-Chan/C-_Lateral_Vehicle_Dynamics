//
//  HelperFunctions.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 24/10/23.
//

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "HelperFunctions.hpp"

//Basic CSV implementation (Non generic)
void HelperFunctions::toCsv(std::string fileName,std::vector<std::string> headers,std::vector<double> x,std::vector<double> y){
    
    //Create header names
    //Replace your full path for storing files
    std::string filePath = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/"+fileName;
    std::cout << filePath << std::endl;
    std::ofstream myFile(filePath);
    if (myFile.is_open())
    {
        std::cout << fileName << " is open" << "\n";
    }

    
     for(auto colName: headers){
         myFile << colName << ",";
     }
     myFile << "\n";
     //Create Data
     if(x.size() < y.size())
         swap(x,y);
     for(int i =0; i < x.size();++i){
         myFile << x[i] << ",";
         if(i >= y.size())
             myFile << -1;
         else
             myFile << y[i];
         myFile << "\n";
     }
     myFile.close();
    if(!myFile.is_open())
        std::cout << fileName << " is closed" << std::endl;
}
void HelperFunctions::fromCsv(std::string fileName)
{
    // Read from the text file
    std::string filePath ="/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/"+fileName;
    std::ifstream MyReadFile(filePath);
    std::string myText;
    // Use a while loop together with the getline() function to read the file line by line
    while (std::getline (MyReadFile, myText)) {
      // Output the text from the file
      std::cout << myText;
    }

    // Close the file
    MyReadFile.close();
    
}

void HelperFunctions::test(){
    std::cout << "Hello from test " << std::endl;
}


//External Function Template Instantiation
//template <double,double> void toCsv();
