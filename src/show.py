import matplotlib.pyplot as plt
import time

script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_dir, "build", "data.txt")

def read_data(filename):
    x, y = [], []
    try:
        with open(filename, "r") as file:
            for line in file:
                parts = line.strip().split(",")
                if len(parts) == 2:
                    x.append(float(parts[0]))
                    y.append(float(parts[1]))
    except FileNotFoundError:
        pass
    return x, y

plt.ion()  # Interaktive Visualisierung
fig, ax = plt.subplots()

while True:
    x, y = read_data(filename)
    ax.clear()
    ax.plot(x, y, "-o", markersize=2)
    plt.draw()
    plt.pause(0.1)  # Aktualisierungsrate
    time.sleep(0.1)
