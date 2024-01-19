import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def flag_detect(args):
    cnt_flags = 0
    final_flags = []
    for flag in args:
        if flag == "-h":
            print("Tool to easily plot graphs. Use it in the following format: \npython3 ./plot.py ../its/in/here/simulate.csv <FLAGS> <whichever headers you want to use as x-axis labels>\n")
            print("FLAGS\n",50 * "-")
            print("-h:\t Help")
            print("-n, --norm:\t Normalise Data")
            quit()
        elif flag == "-n" or flag == "--norm":
            final_flags.append("normalise")
            cnt_flags +=1

    return (final_flags,cnt_flags)
def normalization(df):
    # copy the data 
    df_max_scaled = df.copy() 
    
    # apply normalization techniques 
    for column in df_max_scaled.columns: 
        df_max_scaled[column] = df_max_scaled[column]  / df_max_scaled[column].abs().max() 
    df = df_max_scaled
    return df
def graphing():
    try:
        subplots=True
        args = sys.argv[2::]
        flags = flag_detect(args)
        print("FLAGS: ",flags,"\n")
        cnt_headers = len(sys.argv) - flags[1]

        df = pd.read_csv(sys.argv[1],skip_blank_lines=True)
        df=df.astype(float)

        for flag in flags[0]:
            if(flag == "normalise"):
                df = normalization(df)

        if(cnt_headers != 0):
            plot_axes = list(sys.argv[2+flags[1]::])
            print("[~] Headers indicated: ",plot_axes)
            subplots = False

        if(subplots == True):
            print("[~] No specific headers supplied\n")
            df.plot(subplots=True) #Show each subplot
        
        if(subplots == False):
            for graph in plot_axes:
                print("Plotting")
                df[graph].plot(label=graph)

        plt.tight_layout()
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
        print("[-] ",e)
        print("[-] Maybe try running in the following format\ne.g.\tpython3 ./plot.py ../its/in/here/simulate.csv <FLAGS> <whichever headers you want to use as x-axis labels>")
    
    # State Space,x' = Ax + Bu ((4x4) * (4x1) + (1x1)*(1x1))
    # State output = Cx + Du

if __name__ == "__main__":
    graphing()