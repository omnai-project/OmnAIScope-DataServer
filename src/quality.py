import argparse
import matplotlib.pyplot as plt
import os
import itertools
import numpy as np

# Argumente definieren
parser = argparse.ArgumentParser(description="Generate an eye diagram from x-y values in a file.")
parser.add_argument("--file", type=str, default="../build/Release/data.txt", help="Path to the input file containing x and y values.")
parser.add_argument("--output", type=str, default="eye_diagram.png", help="Name of the output PNG file.")
parser.add_argument("--f", type=str, required=True, help="Frequency to display in the diagram header.")
args = parser.parse_args()

# Datei lesen und Daten extrahieren
def read_data(file_path):
    x = []
    y = []
    try:
        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split(",")
                if len(parts) == 2:
                    x_value = float(parts[0])
                    y_value = float(parts[1])
                    x.append(x_value)
                    y.append(y_value)
    except FileNotFoundError:
        print(f"The file '{file_path}' was not found.")
        exit(1)
    return x, y

# Verteilungsdichtefunktion berechnen
def calculate_density_function(y):
    density, bins = np.histogram(y, bins=100, density=True)
    bin_centers = 0.5 * (bins[1:] + bins[:-1])
    return density, bin_centers

# Augendiagramm erstellen
def create_eye_diagram(x, y, output_filename, frequency, mean_y):
    if not x or not y:
        print("No data to plot.")
        return

    # Farben für die Wechsel
    colors = itertools.cycle(['b', 'g', 'r', 'c', 'm', 'y', 'k'])

    # Identifizieren der Startregion
    above = y[0] > mean_y
    color = next(colors)

    # Daten für die aktuelle Sektion
    segment_x = []
    segment_y = []

    # Plot erstellen
    fig, ax = plt.subplots(figsize=(20, 12))
    ax_eye = plt.subplot2grid((1, 2), (0, 0), fig=fig)
    ax_density = plt.subplot2grid((1, 2), (0, 1), fig=fig)
    
    skipped_first_curve = False

    for i in range(len(x)):
        if (y[i] > 45) != above:  # Wechsel der Region
            if not skipped_first_curve:  # Skip the first curve
                skipped_first_curve = True
                segment_x = []
                segment_y = []
                above = not above
                continue

            # Plotten der aktuellen Sektion
            ax_eye.plot(range(len(segment_x)), segment_y, color=color, linestyle='-', marker='.', markersize=1)
            color = next(colors)
            above = not above
            segment_x = []
            segment_y = []

        # Hinzufügen des aktuellen Punktes zur Sektion
        segment_x.append(len(segment_x))  # Immer bei x=0 starten und inkrementieren
        segment_y.append(y[i])


    # Plotten der letzten Sektion
    if skipped_first_curve and segment_x and segment_y:
        ax_eye.plot(segment_x, segment_y, color=color, marker='.', markersize=1)

    # Verteilungsdichtefunktion plotten
    density, bin_centers = calculate_density_function(y)
    ax_density.plot(density, bin_centers, color='blue', marker='.', markersize=2)
    ax_density.set_title("Density Function (Rotated)")
    ax_density.set_xlabel("Density")
    ax_density.set_ylabel("Amplitude")
    ax_density.grid(True)

    # Diagrammtitel und Achsenbeschriftungen
    ax_eye.set_title(f"Eye Diagram (Frequency: {frequency})")
    ax_eye.set_xlabel("Relative Time (Resets at 0)")
    ax_eye.set_ylabel("Amplitude")
    ax_eye.grid(True)

    # Speichern des Plots als PNG
    plt.tight_layout()
    plt.savefig(output_filename)
    print(f"Eye diagram saved as {output_filename}")

    # Plot anzeigen
    #plt.show()

# Hauptprogramm
if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, args.file)

    print(f"Attempting to read file: {filename}")
    x, y = read_data(filename)
    mean_y = sum(y) / len(y) if y else 0
    create_eye_diagram(x, y, args.output, args.f, mean_y)
