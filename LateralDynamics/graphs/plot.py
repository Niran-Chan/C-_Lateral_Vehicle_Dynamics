import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('data1.csv')
df.drop(df.filter(regex="Unname"),axis=1, inplace=True) #Delete Unnamed Cols unknowingly passed from cpp

df.plot(x="Time",subplots=True) #Plot all subplots w.r.t Time

plt.show() 

