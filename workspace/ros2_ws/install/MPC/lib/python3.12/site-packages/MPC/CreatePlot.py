#!/usr/bin/env python3
# plot_mpc_trajectories.py

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def main():
    # ---------------------------
    # 1. Pfad zur CSV-Datei
    # ---------------------------
    csv_path = '/home/oli/Desktop/Oliver/Uni/MA/Plots/MPC_dynObs_Data'  # ggf. anpassen
    
    # ---------------------------
    # 2. CSV einlesen
    # ---------------------------
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"CSV-Datei nicht gefunden unter: {csv_path}")
    df = pd.read_csv(csv_path)
    
    # ---------------------------
    # 3. Spalten für Prädiktionen ermitteln
    # ---------------------------
    pred_x_cols = sorted([c for c in df.columns if c.startswith('pred_x_')],
                         key=lambda s: int(s.split('_')[-1]))
    pred_y_cols = sorted([c for c in df.columns if c.startswith('pred_y_')],
                         key=lambda s: int(s.split('_')[-1]))
    
    # ---------------------------
    # 4. Plot aufsetzen (X gegen Y) mit Straßen-Darstellung
    # ---------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    
    # 4.1: Straße als graues Rechteck y in [0,2], x in [0,6]
    road = Rectangle((0, 0), 6, 2, color='lightgray', zorder=0)
    ax.add_patch(road)
    # Mittellinie gestrichelt bei y=1
    ax.plot([0, 6], [1, 1], linestyle='--', color='black', lw=1, zorder=1)
    
    # 4.2: aktueller Weg (Actual trajectory) in Blau
    ax.plot(df['actual_x'], df['actual_y'],
            color='blue', lw=2, label='Actual trajectory', zorder=2)
    
    # 4.3: prädizierte Trajektorien in Rot und sehr dünn (nur jede 10.)
    for idx, row in df.iterrows():
        if idx % 10 != 0:
            continue
        xs = np.concatenate([[row['actual_x']], row[pred_x_cols].values])
        ys = np.concatenate([[row['actual_y']], row[pred_y_cols].values])
        ax.plot(xs, ys,
                color='red', lw=0.5, alpha=0.7,
                solid_capstyle='round', zorder=2)
    
    # ---------------------------
    # 5. Achsenbeschriftung, Titel, Legende und Limits
    # ---------------------------
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Actual (blau) vs. Predicted (rot) MPC Trajectories auf zweispuriger Straße')
    ax.legend(loc='upper left', fontsize='small', frameon=True)
    ax.set_aspect('equal')
    ax.set_xlim(0, 6)
    ax.set_ylim(0, 2)
    
    # ---------------------------
    # 6. Ausgabe-Verzeichnis und PDF
    # ---------------------------
    out_dir = '/home/oli/Desktop/Oliver/Uni/MA/Plots'
    os.makedirs(out_dir, exist_ok=True)
    pdf_file = os.path.join(out_dir, 'mpc_2d_trajectories_road.pdf')
    fig.tight_layout()
    fig.savefig(pdf_file)
    print(f"PDF gespeichert unter: {pdf_file}")
    plt.show()

if __name__ == '__main__':
    main()
