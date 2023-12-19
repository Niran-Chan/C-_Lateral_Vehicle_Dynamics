import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

try:
    
    df = pd.read_csv(sys.argv[1],verbose=True,skip_blank_lines=True)
    #df.drop(df.filter(regex="Unname"),axis=1, inplace=True) #Delete Unnamed Cols unknowingly passed from cpp
    
    #Values are read as each column being a new step in time,meaning that each row is the header
    df_cleaned = list(filter( lambda x: "nan" not in x , df.columns )) #Remove NaN Values
    plt.plot(np.asarray(df_cleaned,float))

    plt.show() 
except Exception as e:
    print("[-] Error. Try running in the following format\ne.g.\tpython3 ./plot.py ../its/in/here/simulate.csv")
    print("[-] ",e)
