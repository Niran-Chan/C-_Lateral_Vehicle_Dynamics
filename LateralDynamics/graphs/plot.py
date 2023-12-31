import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

try:
    
    df = pd.read_csv(sys.argv[1],verbose=True,skip_blank_lines=True)
    subplots=True
    plot_axes = []
    if(len(sys.argv) >= 3):
        plot_axes = list(sys.argv[2::])
        subplots = False
    if(subplots):
        df.plot(subplots=True,sharex=True) #Show each subplot
    else:
        for graph in plot_axes:
            df.plot(graph)
    plt.show()

    #Condition to check if csv file uses rows as time vector
    #Check if header can be converted to float
    '''
    is_row_time = False
    try:
        for header in df.columns:
            if(int(header)):
                continue

    except:
        is_row_time = True;
    

    if(is_row_time):
        #Tranpose
        print("[+] Transposing matrix to Col Time vector")
        df = np.transpose(df)
    
    #After transpose, column headers are on the left
    headers = df.index.to_list()
    '''

    
    #for i in range(len(df.columns)):
     #   print(df[i]["time_sec"])
    
    #df.drop(df.filter(regex="Unname"),axis=1, inplace=True) #Delete Unnamed Cols unknowingly passed from cpp
    #ignore_headers = "time_stamp_us"
    #Values are read as each column being a new step in time,meaning that each row is the header
    #df_cleaned = list(filter( lambda x: (type(x)== str and x is "nan" not in x) or x is not None , df.columns)) #Remove NaN Values
    
    #plt.plot(np.asarray(df,float),='time_sec')


    #plt.show() 
except Exception as e:
    print("[-] Error. Try running in the following format\ne.g.\tpython3 ./plot.py ../its/in/here/simulate.csv <whichever headers you want to use as x-axis labels>")
    print("[-] ",e)
# State Space,x' = Ax + Bu ((4x4) * (4x1) + (1x1)*(1x1))
# State output = Cx + Du