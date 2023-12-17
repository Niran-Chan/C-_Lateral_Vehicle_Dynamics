import numpy as np
import matplotlib.pyplot as plt

def randInput():
    # Generate random matrices
    A = 0.06 * np.random.randn(100, 100)
    B = 0.1 * np.random.randn(100, 10)
    C = np.ones((10, 100))
    input_data = np.random.rand(10, 1000) 
    x0 = np.random.randn(100, 1)

    # Get sizes of matrices
    m, sampleTime = input_data.shape #Returns (Rows,Columns)
    r, _ = C.shape
    n, _ = A.shape

    # Initialize state and output matrices
    python_state = np.zeros((n, sampleTime))
    python_output = np.zeros((r, sampleTime))

    # Simulation loop
    for i in range(sampleTime):
        if i == 0:
            python_state[:, i] = x0.flatten()
            python_output[:, i] = np.dot(C, x0).flatten()
        else:
            python_state[:, i] = np.dot(A, python_state[:, i - 1]) + np.dot(B, input_data[:, i - 1])
            python_output[:, i] = np.dot(C, python_state[:, i])

    # Plot the first row of python_output in red
    #plt.plot(python_output[0, :], 'r')
    #plt.show()

    # Export matrices to CSV files
    np.savetxt("AFile.csv", A, delimiter=",")
    np.savetxt("BFile.csv", B, delimiter=",")
    np.savetxt("CFile.csv", C, delimiter=",")
    np.savetxt("x0File.csv", x0, delimiter=",")
    np.savetxt("inputFile.csv", input_data, delimiter=",")

def step_unit(t,dt,file_name):
    #Define a unit step function
    # f(t) =0 for t < 1
    # f(t) = 1 for t >= 1
    # Each column represents the value of Input at that period of time
    '''
                      0   if x1 < 0
heaviside(x1, x2) =  x2   if x1 == 0
                      1   if x1 > 0
     '''
    curr_t =0
    fin = [] # Each Column represent a unit of time and value of input 
    time_delay = 5 
    while(curr_t < t):
        ft = np.heaviside(curr_t - time_delay,0.5) #Scalar value
        fin.append([ft])
        curr_t += dt
    #print(fin)
    fin = np.array(fin)
    np.savetxt(file_name,fin,delimiter=",")
    print("[+] Exported Step Unit Function,",time_delay,"s time delay,file ",file_name)


    
if __name__ == '__main__':
    t = 10 #Total Time
    dt = 0.01 #Time step
    step_unit(t,dt,"step_input.csv")