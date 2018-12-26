import matplotlib
import matplotlib.pyplot as plt
import csv
from tkinter import filedialog
from tkinter import *
 
root = Tk()
root.filename =  filedialog.askopenfilename(title = "Select file",filetypes = (("csv files","*.csv"),("all files","*.*")))

t = []
st = []
et = [];
bt = [];
h = [];
f = [];

with open(root.filename, 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    plots.__next__(); # skip first row
    for row in plots:
        t.append(float(row[0])/60)
        st.append(float(row[1]))
        et.append(float(row[2]))
        bt.append(float(row[3]))
        h.append(float(row[4]))
        f.append(float(row[5]))


fig, axs = plt.subplots(2, 1)
axs[0].plot(t,st, label='Control Value',linewidth=4)
axs[0].plot(t,bt,label='Bean Temp')
axs[0].plot(t,et, label='Env Temp')
axs[0].grid(True)
axs[0].set_xlim(0,t[-1])
axs[0].set_ylabel('Temperature (deg F)')
axs[0].set_title('Roast Profile')
axs[0].legend(loc='lower right',shadow=True)

axs[1].plot(t,h,label='Heat')
axs[1].plot(t,f,label='Fan')
axs[1].grid(True)
axs[1].set_xlim(0,t[-1])
axs[1].set_xlabel('Time (minutes)')
axs[1].set_ylabel('Percent')        
axs[1].legend(loc='lower right',shadow=True)
plt.show()

