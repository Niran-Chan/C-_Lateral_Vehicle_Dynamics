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

    #Define a unit step function
    # f(t) =0 for t < 1
    # f(t) = 1 for t >= 1
    # Each column represents the value of Input at that period of time
    '''
                      0   if x1 < 0
heaviside(x1, x2) =  x2   if x1 == 0
                      1   if x1 > 0
     '''

def general_input(t,dt,file_name,t_start,t_delay,fn,plot=True):
    f = []
    m = 0;
    for i in np.arange(t_start,t,dt):
        f.append([fn(i,t_delay)])
        m+=1
        #print(f)
    f = np.array(f)
    f = f.reshape(1,m)
    np.savetxt(file_name,f,delimiter=",")
    
    if(plot):
        plt.plot(f)
        plt.show()
    
    print("[+] Exported ",file_name,",",t_delay,"s time delay")


if __name__ == '__main__':
    t = 10 #Total Time
    dt = 0.01 #Time step
    t_delay=5 #Time delay
    t_start = 0 #Time Start
    #step_input(t,dt,"step_input.csv")
    #ramp_input(t,dt,"ramp_input.csv")
    #sinusoidal_input()
    
    #Generate Test Files
    
    #Ramp Input
    general_input(t,dt,"ramp_input.csv",t_start,t_delay,fn = lambda t_start,t_delay:(1 * (t_start - t_delay) )* np.heaviside((t_start - t_delay),0.5),plot=False)
    
    #Step Input
    general_input(t,dt,"step_input.csv",t_start,t_delay,fn = lambda t_start,t_delay:np.heaviside(t_start - t_delay,0.5),plot=False)
    
    #Sinusoidal Input
    general_input(t,dt,"sinusoidal_input.csv",t_start,t_delay,fn = lambda t_start,t_delay:np.heaviside((t_start-t_delay),0.5)*np.sin(t_start-t_delay),plot=False)

    #Impulse Input
    general_input(t,dt,"impulse_input.csv",t_start,t_delay,fn = lambda t_start,t_delay: 1 if t_start == t_delay else 0,plot=False)