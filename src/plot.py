import matplotlib.pyplot as plt
import os

# Datei mit den Daten
script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_dir, "../build/Release/", "data.txt")
print(f"Versuchter Pfad zu data.txt: {filename}")

# Listen für die Daten
x = []
y = []
filtered_x = []
filtered_y = []


# Datei lesen und Daten extrahieren
try:
    with open(filename, "r") as file:
        for line in file:
            parts = line.strip().split(",")
            if len(parts) == 2:
                x_value = float(parts[0])  
                y_value = float(parts[1])  
            if 100000 <= x_value <= 10000000:
                filtered_x.append(x_value)
                filtered_y.append(y_value)

except FileNotFoundError:
    print(f"Die Datei '{filename}' wurde nicht gefunden.")
    exit(1)

# Erstelle eine Figur und Achsen
fig, ax = plt.subplots(figsize=(10, 6))

# Daten plotten
ax.plot(filtered_x, filtered_y, '_', label="Daten", markersize=1)

# Achsen begrenzen
if x and y:  # axis range fits data values
    ax.set_xlim(100000, 10000000)  
    ax.set_ylim(min(filtered_y), max(filtered_y))

# Achsentitel, Gitter und Legende hinzufügen
ax.set_title("Datenvisualisierung")
ax.set_xlabel("X-Werte")
ax.set_ylabel("Y-Werte")
ax.grid(True)
ax.legend()

# Plot anzeigen
plt.show()

