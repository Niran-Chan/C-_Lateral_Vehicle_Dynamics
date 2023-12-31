/// Class for simulating a linear state-space model
///  x_{k+1}=Ax_{k}+Bu_{k}
///  y_{k} = Cx_{k}

/// The class is implemented using the Eigen library:
/// http://eigen.tuxfamily.org/index.php?title=Main_Page
//*******************************************************************************************************************************
// AUTHOR: Aleksandar Haber,Niranjan Gopinath
// DEVELOPMENT: June 2020 - December 2023
// NOTE: This implementation is not optimized for extremely large-scale problems, since all the matrices are passed by value
//*******************************************************************************************************************************
#ifndef SimulateSystem_hpp
#define SimulateSystem_hpp

#include <iostream>
#include "Eigen/Dense"
#include<tuple>
#include<string>
#include<fstream>
#include<vector>
 
using namespace Eigen;
 
// MatrixXd is an Eigen typdef for Matrix<double, Dynamic, Dynamic>
 
 
class SimulateSystem {
public:
     
    //CONSTRUCTORS & DESTRUCTORS
    //!Default Constructor
    /*!Sets all the variables to 1x1 dimensional matrices and sets all the variables to zero. This is most ideal if input sequence has yet to be imported.
     */
    SimulateSystem();
     
    //!Overloaded Constructor
    /*!Assigns all private variables
     */
    SimulateSystem(MatrixXd Amatrix, MatrixXd Bmatrix, MatrixXd Cmatrix, MatrixXd Dmatrix, MatrixXd initialState, MatrixXd inputSequenceMatrix);
    
    //!Default Destructor
    /*!Destroys instance of SimulateSystem*/
    ~SimulateSystem();
    // Default destructor  - currently just an empty implementation in the ".cpp" file
     
    // MatrixXd is an Eigen typdef for Matrix<double, Dynamic, Dynamic>

    //METHODS
    /*!
      \return Tuple of Simulated State, Simulated Output sequence,Time Row vector
     \sa printSimulationParams
    */
    std::tuple<MatrixXd, MatrixXd, MatrixXd> getStateOuputTime();
    
    /*!Setter values of private variables in SimulatedSystem Class
        \param A Eigen Matrix of State Variables Coefficient
        \param B Eigen Matrix of Input Coefficient
        \param C Eigen Matrix of Output State Model Coefficient
        \param x0 Eigen Matrix of Initial State Variables
        \param inputSequence Eigen Matrix of input sequence, usually existing as Scalar Input u ( State Space Model = Ax + Bu )
     
     */
    void setMatrices(MatrixXd A,MatrixXd B,MatrixXd C, MatrixXd D,MatrixXd x0,MatrixXd inputSequence);
    ///<Setter for private variables A, B, C, D,x0, InputSequence and Time Samples. Time Samples is generated from number of columns in inputSequence.
    
    /*!Get values of private variables in SimulatedSystem Class
    \return vector in {A,B,C,D,x0,inputSequence}
     \sa printSimulationParams
     */
    std::vector<MatrixXd> getMatrices();
    ///<Getter in the form A,B,C,D,x0 and InputSequence
    
    /*!Print Simulation Parameters. Ideal for debugging
        \param moreParams=false default parameter. Set to true to print more information.
     */
    void printSimulationParams(bool moreParams=false);
    ///<Print Simulation Params at current state. Insert argument to show more information about params.
    
    /*!Run Simulation*/
    void runSimulation();
    ///<Run simulation and save state models into class instance
    
    /*!Export data into CSV Format
     \param AFile Matrix A File Name
     \param BFile Matrix B File Name
     \param CFile Matrix C File Name
     \param DFile Matrix D File Name
     \param x0File Matrix x0 File Name
     \param inputSequenceFile Matrix inputSequence File Name
     \param simulatedStateSequenceFile Matrix simulatedStateSequence File Name
     \param simulatedOutputSequenceFile Matrix simulatedOutputSequence File Name
     */
    void saveData(std::string AFile, std::string BFile, std::string CFile,std::string DFile, std::string x0File, std::string inputSequenceFile, std::string simulatedStateSequenceFile, std::string simulatedOutputSequenceFile) const;

     
    /*!Import CSV File
        \param fileToOpen String of file name with file path if not current working directory
        \param headers select which headers are important and to ignore the rest. Default is no headers omitted.
        \return Eigen Matrix of imported CSV
     */
    MatrixXd openData(std::string fileToOpen,std::vector<std::string> headers = {});
    ///<Opens file in argument and loads the entries into the Eigen matrix MatrixXd. File in CSV must have also been a matrix.
    
    /*!Start SimulateSystem Class Instance from already stored CSV files
        \param Afile String File Name of Matrix A
     \param Bfile String File Name of Matrix B
     \param Cfile String File Name of Matrix C
     \param Dfile String File Name of Matrix D
     \param x0file String File Name of State Variables
     \param inputSequenceFile String File Name of input sequence
                
     */
    void openFromFile(std::string Afile, std::string Bfile, std::string Cfile,std::string D, std::string x0File, std::string inputSequenceFile);
    ///<A form of constructor, making use of openData multiple times to open and save files into Eigen Matrices.
    // this function calls the function MatrixXd openData(std::string fileToOpen);
    // this function acts as a constructor in some way.
    // call this function after a default constructor is being called
    // the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
    // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
 
 
private:
    // MatrixXd is an Eigen typdef for Matrix<double, Dynamic, Dynamic>
    MatrixXd A,B,C,D; ///<A and B are State Model Coefficient Matrices, where stateModel= Ax + Bu, where x is state variables and u is Input. C and D are state model coefficient and input coefficient respectively for state model output
    MatrixXd x0;    ///<Initial value of state variables, x
    MatrixXd inputSequence; ///<Input Sequence, U, where input can be scalar or a relevant Eigen Matrix
                            //dimensions: m\times  timeSamples
    MatrixXd simulatedStateSequence; ///<Instantaneous State Space Model at time t
    // dimensions: n\times  timeSamples
    MatrixXd simulatedOutputSequence; ///<Instantaneuous Output Model, where y = Cx + Du, and y is output
    
    // dimensions: r\times  timeSamples
    MatrixXd timeRowVector;           ///<Vector containing time values for each frame
    
    //[0,1,2,3,\ldots, timeSamples-1]
     
    int m; ///<Input Dimension
    int n; ///<State Dimension
    int r; ///<Output Dimension
    int timeSamples; ///<Defined Time of Simulation
    
    //m - input dimension, n- state dimension, r-output dimension, timeSamples- number of time samples
 
};

#endif /* SimulateSystem_hpp */
