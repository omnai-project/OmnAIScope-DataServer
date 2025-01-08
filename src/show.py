import matplotlib.pyplot as plt
import time
import os

# data directory check 
script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_dir, "../build/Release/", "data.txt")
print(f"Versuchter Pfad zu data.txt: {filename}")

#for data 
x_data, y_data = [], []
last_position = 0

# only read the last data point 
def read_data(filename):
    x, y = [], []
    stop_signal = False
    try:
        with open(filename, "r") as file:
            for line in file:
                if line.strip() == "STOP":
                    stop_signal = True
                    break
                parts = line.strip().split(",")
                if len(parts) == 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
    except (FileNotFoundError, PermissionError):
        # If data is written currently pass 
        pass
    return x, y, stop_signal

plt.ion()
fig, ax = plt.subplots()

while True:
    # read data 
    x, y , stop_signal = read_data(filename)
    
    # Plot 
    ax.clear()
    ax.plot(x, y, '-o', markersize=2)

    if x and y:  # axis range fits data values
        ax.set_xlim(min(x), max(x))  
        ax.set_ylim(40, 50)

    ax.set_title("Live-Datenvisualisierung")
    ax.set_xlabel("X-Werte")
    ax.set_ylabel("Y-Werte")
    ax.grid(True)
    
    if stop_signal:
        print("Stop-Signal erkannt. Beende Python-Skript.")
        break

    plt.draw()
    plt.pause(0.1)
    time.sleep(0.1)
    
    
