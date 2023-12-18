import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

try:
    
    df = pd.read_csv(sys.argv[1])
    #df.drop(df.filter(regex="Unname"),axis=1, inplace=True) #Delete Unnamed Cols unknowingly passed from cpp

    #df.plot(x="Time",subplots=True) #Plot all subplots w.r.t Time

    #print(df)
    plt.plot(df)
    

    plt.show() 
except:
    print("[-] Error. Run in the following format\ne.g.\tpython3 ./plot.py ../its/in/here/simulate.csv")
