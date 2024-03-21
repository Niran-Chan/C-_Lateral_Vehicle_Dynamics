//
//  SimulateSystem.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 16/12/23.
//

#include "SimulateSystem.hpp"
#include "StateSpaceBlock.hpp"

const std::string FILEPATH = "/Users/niran/Documents/Y4S1/ME4101A(FYP)/LateralDynamics/LateralDynamics/graphs/";
//Fixed File Path (eventually to relative directories)
const IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

double currStep = 0;

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
    timeRowVector.resize(1, 1); timeRowVector.setZero(); //Vector contatining frame of iteration
}
 
SimulateSystem::SimulateSystem(StateSpaceBlock ssBlk)
{
    A = ssBlk.A; B = ssBlk.B; C = ssBlk.C; x0 = ssBlk.x0; inputSequence = ssBlk.inputSequence;
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


void SimulateSystem::setMatrices(MatrixXd& A,MatrixXd& B,MatrixXd& C,MatrixXd& D,MatrixXd& x0,MatrixXd& inputSequence){
    //Private Class Setter
    this -> A = A;
    this -> B = B;
    this -> C = C;
    this -> D = D;
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
    std::cout << "[~] Time Samples Set: " << inputSequence.cols() << std::endl;
    std::cout << "[+] Simulation Params Set!" << std::endl;
}
void SimulateSystem::setMatricesNR(StateSpaceBlock ss){

        this -> A = A;
        this -> B = B;
        this -> C = C;
        this -> D = D;
        this -> x0 = x0;
        this -> inputSequence = inputSequence;
        this -> n = A.rows();
        this -> m = B.cols();
        this -> r = C.rows();
    
}
std::vector<MatrixXd> SimulateSystem::getMatrices(){
    //Private Class Getter
    /*
    std::cout << "Important Matrices" << std::endl;
    std::cout << "A:\n " << this -> A << std::endl;
    std::cout << "B:\n " << this -> B << std::endl;
    std::cout << "C:\n " << this -> C << std::endl;
    std::cout << "x0:\n" << this -> x0 << std::endl;
    //std::cout << "Input Sequence:\n " << this -> inputSequence << std::endl;
    std::cout << "Time Samples: " << this -> timeSamples << std::endl;
     */
    std::vector<MatrixXd> res {};
    res.push_back(this -> A);
    res.push_back(this -> B);
    res.push_back(this -> C);
    res.push_back(this -> D);
    res.push_back(this -> x0);
    res.push_back(this -> inputSequence);
    res.push_back(this -> simulatedStateSequence);
    res.push_back(this -> simulatedOutputSequence);
    return res;
}
void SimulateSystem::printSimulationParams(bool moreParams){
    
    std::cout << std::string(45,'-') << "\n";
    std::cout << "\t\t\t\tSystem Params\t\t\t\t\t" << "\n";
    std::cout << std::string(45,'-') << "\n";
    std::cout << "Matrix A \n" << this -> A<< std::endl;
    std::cout << "Matrix B \n" << this -> B<< std::endl;
    std::cout << "Matrix C \n" << this -> C<< std::endl;
    std::cout << "Matrix D \n" << this -> D<<std::endl;
    std::cout << "Initial State, x0 \n" << this -> x0<< std::endl;
    if(moreParams) //More Verbose
    {std::cout << "Input Sequence \n" << this -> inputSequence<< std::endl;
    }
    std::cout << "Time Samples for Simulation: " << this -> timeSamples<< " Frames "<<  std::endl;
    std::cout << std::string(45,'-') << std::endl;
}
 
std::tuple<MatrixXd, MatrixXd, MatrixXd> SimulateSystem::getStateOuputTime()
{
    std::tuple<MatrixXd, MatrixXd, MatrixXd>   result(simulatedStateSequence, simulatedOutputSequence, timeRowVector);
    return result;
}

void SimulateSystem::saveData(std::string AFile, std::string BFile, std::string CFile,std::string DFile,std::string x0File, std::string inputSequenceFile, std::string simulatedStateSequenceFile, std::string simulatedOutputSequenceFile) const
{
    //const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
    
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
    std::ofstream fileD(FILEPATH + DFile);
    if (fileD.is_open())
    {
        fileD << D.format(CSVFormat);
        fileD.close();
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
        auto transposedInputSequence =inputSequence.transpose();
        for(int i =1; i < transposedInputSequence.cols() + 1; ++i){
           std::string isComma = i == transposedInputSequence.cols() ? "" : ",";
            fileInputSequence << "x" << std::to_string(i) << isComma;
        }
        fileInputSequence << std::endl;
        fileInputSequence << transposedInputSequence.format(CSVFormat);
        fileInputSequence.close();
    }
 
    std::ofstream fileSimulatedStateSequence(FILEPATH + simulatedStateSequenceFile);
    if (fileSimulatedStateSequence.is_open())
    {
        auto transposedStateSequence =simulatedStateSequence.transpose();
        for(int i =1; i < transposedStateSequence.cols() + 1; ++i){
           std::string isComma = i == transposedStateSequence.cols() ? "" : ",";
            fileSimulatedStateSequence << "x" << std::to_string(i) << isComma;
        }
        fileSimulatedStateSequence << std::endl;
        fileSimulatedStateSequence << transposedStateSequence.format(CSVFormat);
        fileSimulatedStateSequence.close();
    }
 
    std::ofstream fileSimulatedOutputSequence(FILEPATH + simulatedOutputSequenceFile);
    if (fileSimulatedOutputSequence.is_open())
    {
        auto transposedOutputSequence =simulatedOutputSequence.transpose();
        for(int i =1; i < transposedOutputSequence.cols() + 1; ++i){
           std::string isComma = i == transposedOutputSequence.cols() ? "" : ",";
            fileSimulatedOutputSequence << "x" << std::to_string(i) << isComma;
        }
        fileSimulatedOutputSequence << std::endl;
        fileSimulatedOutputSequence << transposedOutputSequence.format(CSVFormat);
        fileSimulatedOutputSequence.close();
    }
 
    std::cout << "[+] Data Saved into Files: " << FILEPATH << std::endl;
 
}
 
MatrixXd SimulateSystem::openData(std::string fileToOpen,std::vector<std::string> headers)
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
    
    //To make it selective, we need to take local column values as well
    std::vector<double> matrixEntries;

    // in this object we store the data from the matrix
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

    // here we conver the vector variable into the matrix and return the resulting object,
    // note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
    
    for(auto &header : headers){
        std::string result = headerMap.find(header)!=headerMap.end() ? "yes" : "no";
        std::cout << "Existence of header " << header << " : "<< result  << std::endl;
        for(auto val : headerMap[header]){
            matrixEntries.push_back(val);
            validElems++;
        }
    }
  //  std::cout << "Number of rows: " << headers.size() << std::endl;
  //  std::cout << "Mumber of elements processed: " << nElems << std::endl;
  //  std::cout << "Number of valid elements: " << validElems << std::endl;
  //  std::cout << "Stride/step for array memory management : " << matrixEntries.size() / validElems << std::endl;
  //  std::cout<< "Size of output array: " << matrixEntries.size() << std::endl;
    
    //Access contiguous data in array as column data,Change to colmajor from RowMajor to transpose
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>> (matrixEntries.data(),headers.size(),validElems/headers.size());
    
    //Change Stride denominator to matrixRosNumber if matrix not transposed
    //Stride length if cols not specified: matrixEntries.size() / validCols
}

//Easier method to call for resizing state space and output model

void SimulateSystem::modelResize(){
    n = A.rows();
    m = B.cols();
    r = C.rows();
    timeSamples = inputSequence.cols();

    simulatedOutputSequence.resize(r, timeSamples); simulatedOutputSequence.setZero();// C Rows x Time Samples
    simulatedStateSequence.resize(n, timeSamples);  simulatedStateSequence.setZero();
    //A rows x Time Samples
 
    timeRowVector.resize(1, timeSamples);
 
    for (int i = 0; i < timeSamples; i++)
    {
        timeRowVector(0, i) = i + 1;
    }
 
}
 
void SimulateSystem::resetSimulationTime(){
    //Set Global Curr Time Variable to 0
    currStep = 0;
}
void SimulateSystem::runSimulation()
{
    for (int j = currStep; j < timeSamples; j++)
        runStep();
}

void SimulateSystem::runStep(){
    if(currStep == 0)
    {
        simulatedStateSequence.col(currStep) = x0;
        simulatedOutputSequence.col(currStep) = C * x0 ;
        currStep++;
    }
    else{
        simulatedStateSequence.col(currStep) = A * simulatedStateSequence.col(currStep - 1) + B * inputSequence.col(currStep - 1);
        simulatedOutputSequence.col(currStep) = C * simulatedStateSequence.col(currStep) + D * inputSequence.col(currStep);
        currStep++;
    }
}
