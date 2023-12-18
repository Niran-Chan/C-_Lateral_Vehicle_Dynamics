//
//  SimulateSystem.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 16/12/23.
//

#include "SimulateSystem.hpp"


const std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";
//Fixed File Path (eventually to relative directories)

SimulateSystem::SimulateSystem() //Default Constructor
{
    m = 0; n = 0; r = 0;
    A.resize(1, 1); A.setZero(); //Size 1x1 matrix
    B.resize(1, 1); B.setZero();
    C.resize(1, 1); C.setZero();
    x0.resize(1, 1); x0.setZero();
    inputSequence.resize(1, 1); inputSequence.setZero();
    simulatedStateSequence.resize(1, 1); simulatedStateSequence.setZero();
    simulatedOutputSequence.resize(1, 1); simulatedOutputSequence.setZero();
    timeRowVector.resize(1, 1); timeRowVector.setZero(); //step used for each iteration
}
 
SimulateSystem::SimulateSystem(MatrixXd Amatrix, MatrixXd Bmatrix, MatrixXd Cmatrix, MatrixXd initialState, MatrixXd inputSequenceMatrix)
{
    A = Amatrix; B = Bmatrix; C = Cmatrix; x0 = initialState; inputSequence = inputSequenceMatrix;
    n = A.rows();
    m = B.cols();
    r = C.rows();
    timeSamples = inputSequence.cols();
 
    simulatedOutputSequence.resize(r, timeSamples); simulatedOutputSequence.setZero();
    simulatedStateSequence.resize(n, timeSamples);  simulatedStateSequence.setZero();
     
    timeRowVector.resize(1, timeSamples);
 
    for (int i = 0; i < timeSamples; i++)
    {
        timeRowVector(0, i) = i + 1;
    }
 
}
 
SimulateSystem::~SimulateSystem() //destructor
{
}
//METHODS


void SimulateSystem::setMatrices(MatrixXd A,MatrixXd B,MatrixXd C,MatrixXd x0,MatrixXd inputSequence){
    //Private Class Setter
    this -> A = A;
    this -> B = B;
    this -> C = C;
    this -> x0 = x0;
    this -> inputSequence = inputSequence;
    this -> n = A.rows();
    this -> m = B.cols();
    this -> r = C.rows();
    this -> timeSamples = inputSequence.cols();
 
    simulatedOutputSequence.resize(r, timeSamples); simulatedOutputSequence.setZero();
    simulatedStateSequence.resize(n, timeSamples);  simulatedStateSequence.setZero();

    timeRowVector.resize(1, timeSamples);
 
    for (int i = 0; i < timeSamples; i++)
    {
        timeRowVector(0, i) = i + 1;
    }
    std::cout << timeSamples << std::endl;
}
void SimulateSystem::getMatrices(){
    //Private Class Getter
    std::cout << "Important Matrices" << std::endl;
    std::cout << "A:\n " << this -> A << std::endl;
    std::cout << "B:\n " << this -> B << std::endl;
    std::cout << "C:\n " << this -> C << std::endl;
    std::cout << "x0:\n" << this -> x0 << std::endl;
    //std::cout << "Input Sequence:\n " << this -> inputSequence << std::endl;
    std::cout << "Time Samples: " << this -> timeSamples << std::endl;
    
}
 
std::tuple<MatrixXd, MatrixXd, MatrixXd> SimulateSystem::getStateOuputTime()
{
    std::tuple<MatrixXd, MatrixXd, MatrixXd>   result(simulatedStateSequence, simulatedOutputSequence, timeRowVector);
    return result;
}
 
void SimulateSystem::saveData(std::string AFile, std::string BFile, std::string CFile, std::string x0File, std::string inputSequenceFile, std::string simulatedStateSequenceFile, std::string simulatedOutputSequenceFile) const
{
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    
    std::ofstream fileA(FILEPATH + AFile);
    if (fileA.is_open())
    {
        fileA << A.format(CSVFormat);
        fileA.close();
    }
 
    std::ofstream fileB(FILEPATH + BFile);
    if (fileB.is_open())
    {
        fileB << B.format(CSVFormat);
        fileB.close();
    }
 
     
    std::ofstream fileC(FILEPATH + CFile);
    if (fileC.is_open())
    {
        fileC << C.format(CSVFormat);
        fileC.close();
    }
 
 
 
    std::ofstream fileX0(FILEPATH + x0File);
    if (fileX0.is_open())
    {
        fileX0 << x0.format(CSVFormat);
        fileX0.close();
    }
 
 
    std::ofstream fileInputSequence(FILEPATH + inputSequenceFile);
    if (fileInputSequence.is_open())
    {
        fileInputSequence << inputSequence.format(CSVFormat);
        fileInputSequence.close();
    }
 
    std::ofstream fileSimulatedStateSequence(FILEPATH + simulatedStateSequenceFile);
    if (fileSimulatedStateSequence.is_open())
    {
        fileSimulatedStateSequence << simulatedStateSequence.format(CSVFormat);
        fileSimulatedStateSequence.close();
    }
 
    std::ofstream fileSimulatedOutputSequence(FILEPATH + simulatedOutputSequenceFile);
    if (fileSimulatedOutputSequence.is_open())
    {
        fileSimulatedOutputSequence << simulatedOutputSequence.format(CSVFormat);
        fileSimulatedOutputSequence.close();
    }
 
 
 
}
 
MatrixXd SimulateSystem::openData(std::string fileToOpen)
{
 
    // the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
    // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
    // NOTE THAT THIS FUNCTION IS CALLED BY THE FUNCTION: SimulateSystem::openFromFile(std::string Afile, std::string Bfile, std::string Cfile, std::string x0File, std::string inputSequenceFile)
     
    // the input is the file: "fileToOpen.csv":
    // a,b,c
    // d,e,f
    // This function converts input file data into the Eigen matrix format
 
 
 
    // the matrix entries are stored in this variable row-wise. For example if we have the matrix:
    // M=[a b c
    //    d e f]
    // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
    // later on, this vector is mapped into the Eigen matrix format
    std::vector<double> matrixEntries;
 
    // in this object we store the data from the matrix
    std::ifstream matrixDataFile(fileToOpen);
 
    // this variable is used to store the row of the matrix that contains commas
    std::string matrixRowString;
 
    // this variable is used to store the matrix entry;
    std::string matrixEntry;
 
    // this variable is used to track the number of rows
    int matrixRowNumber = 0;
    int ncols =0 ;
 
    while (std::getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
    {
        std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
 
        while (std::getline(matrixRowStringStream, matrixEntry,',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
        {
            matrixEntries.push_back(std::stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
            ncols ++;
            }
        matrixRowNumber++; //update the column numbers
    }
    std::cout << "Number of rows: " << matrixRowNumber << std::endl;
    std::cout << "Number of cols: " << ncols << std::endl;
    // here we conver the vector variable into the matrix and return the resulting object,
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    //std::cout <<"Number of columnds found: " <<  matrixRowNumber << std::endl;
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
 
}
 
 
void SimulateSystem::openFromFile(std::string Afile, std::string Bfile, std::string Cfile, std::string x0File, std::string inputSequenceFile)
{
    // this function acts as a constructor in some way.
    // call this function after a default constructor is being called
 
    A = openData(Afile);
    B = openData(Bfile);
    C = openData(Cfile);
    x0= openData(x0File);
    inputSequence=openData(inputSequenceFile);
 
    n = A.rows();
    m = B.cols();
    r = C.rows();
    timeSamples = inputSequence.cols();
 
    simulatedOutputSequence.resize(r, timeSamples); simulatedOutputSequence.setZero();
    simulatedStateSequence.resize(n, timeSamples);  simulatedStateSequence.setZero();
 
    timeRowVector.resize(1, timeSamples);
 
    for (int i = 0; i < timeSamples; i++)
    {
        timeRowVector(0, i) = i + 1;
    }
 
     
}
 
void SimulateSystem::runSimulation()
{
    for (int j = 0; j < timeSamples; j++)
    {
        if (j == 0)
        {
            simulatedStateSequence.col(j) = x0; //Equate current
            simulatedOutputSequence.col(j) = C * x0;
        }
        else
        {
            simulatedStateSequence.col(j) = A * simulatedStateSequence.col(j - 1) + B * inputSequence.col(j - 1);
            simulatedOutputSequence.col(j) = C * simulatedStateSequence.col(j);
        }
         
    }
}
