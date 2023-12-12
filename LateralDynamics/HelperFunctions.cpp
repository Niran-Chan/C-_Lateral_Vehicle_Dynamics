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

//Temporary Method to Store Input Data before converting to CSV
void HelperFunctions::storeAsVector(std::vector<double> vecIn,std::vector<std::vector<double>>& vecOut)
{
    std::vector<double> temp;
    for(auto &a : vecIn){
        temp.push_back(a);
    };
    vecOut.push_back(temp);
}

//Basic CSV implementation (Non generic)
void HelperFunctions::toCsv(std::string fileName,std::vector<std::string> headers,std::vector<std::vector<double>> x){
    
    //Create header names
    //Replace your full path for storing files
    std::string filePath = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/"+fileName;
    std::cout << "File Location: " << filePath << std::endl;
    std::ofstream myFile(filePath);
    if (myFile.is_open())
    {
        std::cout << fileName << " is open" << "\n";
    }

    std::cout << "Writing Files..." << std::endl;
    
     for(auto colName: headers){
         myFile << colName << ",";
     }
     myFile << "\n";
     //Ensure that table sizes are matched

     for(int i =0; i < x.size();++i){
         for(int j =0; j < x[i].size();++j){
             if(j == x[i].size() - 1) //Last line no comma
                 myFile << x[i][j];
             else
                 myFile << x[i][j] << ",";
         }
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
