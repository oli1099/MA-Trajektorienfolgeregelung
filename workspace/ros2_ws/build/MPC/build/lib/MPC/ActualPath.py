#!/usr/bin/env python3
import os, math
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from scipy.interpolate import interp1d
from cycler import cycler
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset
# ab Matplotlib 3.7 gibt’s auch ax.indicate_inset_zoom(..) als Kurzbefehl
from matplotlib.animation import FuncAnimation, PillowWriter
#import matplotlib as mpl



# Liste der Ordner, die jeweils actual_path, predictions und theta CSVs enthalten
folders = [
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryMPC',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=1_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=3_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=3_Np=25_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=5_Q=100_T=0.1_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=20',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=30',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.2_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=10_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=15_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=15_Np=25_Q=100_T=0.1_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=25_Np=25_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=43_Q=100_T=0.1',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=35_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=35_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=30_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=20_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=10_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=15_Q=0.1_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=15_Q=1_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=15_Q=10_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=15_Np=15_Q=100_T=0.1_Lidar',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.15',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.3',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.1',

    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_Q=100_T=30_Ts=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_Q=100_T=30_Ts=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=5_Q=100_T=0.1_T=23',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_T=23',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=25_Q=100_T=0.1_T=23',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=35_Q=100_T=0.1_T=23',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_T=23_Ts=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=25_Q=100_T=0.1_T=23_Ts=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_T=23_Ts=0.05',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=25_Q=100_T=0.1_T=23_Ts=0.05',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=50_Q=100_T=0.1_T=23',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.15',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.01_V_ref=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.05_V_ref=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.3',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.01_V_ref=0.3',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.25',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.05_V_ref=0.3',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.15_V_ref=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.2_V_ref=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.01_V_ref=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.2_V_ref=0.3',

]

#mpl.use('agg')

# 2) rcParams für LaTeX‐Integration
#'''mpl.rcParams.update({
 #   "font.family":   "serif",
  #  "text.usetex":   True,
   # "pgf.texsystem": "pdflatex",
    #"pgf.rcfonts":   False,
    # Preamble als einzelner String mit \n
    #"pgf.preamble": r"\usepackage[T1]{fontenc}" + "\n" + r"\usepackage{lmodern}"
#})'''


# Dateinamen
actual_path_file        = 'mpc_data_actual_path.csv'
predictions_file        = 'mpc_data_predictions.csv'
actual_theta_file       = 'mpc_data_actual_theta.csv'
predicted_theta_file    = 'mpc_data_predicted_theta.csv'
solve_times_file        = 'mpc_data_solve_times.csv'
control_inputs_file     = 'mpc_data_control_inputs.csv'
reference_file        = 'mpc_data_reference.csv'

# Obstacle-Parameter
obstacle = {
    'obsXrl': 1.0,      # x-Koordinate (links unten)
    'obsYrl': 0.25,     # y-Halbhöhe
    'obslength': 0.75   # Breite in x-Richtung
}
safezone = 0.1

# Labels und Plot-Stile
labels = [
  #'Soll-Trajektorie',
   # 'Nc=1',
    #'Nc=3, Np=15',
    r'$N_p=15$',
    #'Nc=3, Np=25', 
    r'$N_c=5$',
    #r'$\|\mathbf{u}\|\leq 10$',
    #r'$\|\mathbf{u}\|\leq 20$',
    #'Nc=5, Np=25 u=30',
    #'Nc=5, Np=25, Ts=0.2',
    r'$N_c=25$',
    #'Nc=15, Np=25',
    #'Nc=10, Np=15', 
    #'Nc=15, Np=15',
    #'Nc=25, Np=25', 
    #'Np=43', 
    #r'$T_s=0,1$',
    #r'$T_s=0,2$',
    #r'$N_c=25$',
    #'Nc=5,Np=35', 
    #'Nc=5,Np=30', 
    #'Nc=5,Np=25', 
    #'Np=20', 
    #'Nc=5,Np=10',
    #'Q=0.1',
    #'Q=1',
    #'Q=10',
    #'Q=1000_Lidar'
    #'LA=0.15',
    #'LA=0.3',
    #'LA=0.1',
    
    #'MPCTrajectory_Q=100_T=30_Ts=0.2',
    #'MPCTrajectory_Q=100_T=30_Ts=0.1',
    #r'$N=5$',
    #r'$N=15$',
    r'$N_p=35$',
    #r'$N=35$',
    #'N=15, Ts=0.2',
    #'N=25, Ts=0.2',
    #'N=15_Ts=0.05',
    #'N=25_Ts=0.05',
    #'N=50',
    #r'$V_\mathrm{ref}=0.1$',
    #r'$V_\mathrm{ref}=0.15$',
    #'L=0.01_V_ref=0.2',
    #r'$V_\mathrm{ref}=0.2$',
    #'L=0.1_V_ref=0.3',
    #r'$L_\mathrm{ref}=0.01$',
    #r'$L_\mathrm{ref}=0.05$',
    #r'$L_\mathrm{ref}=0.1$',
    #r'$V_\mathrm{ref}$=0.25',
    #'L=0.05_V_ref=0.3',
    #'L=0.15_V_ref=0.2',
    #r'$L_\mathrm{ref}=0.2$',
    #'L=0.01_V_ref=0.1',
    #'L=0.2_V_ref=0.3'
]


colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
linestyles = [':', '-.', '--', '-']

my_palette = [
    # Base
   
     "#009D81",
     "#FFC857",
     "#9E9A00", 
     "#ff627e", 
      
      
     "#0057FF",
     
    "#709c5b",  # 20% Tint
      # 40% Tint
    "#99D8CD",  # 60% Tint
    "#CCEBE6",  # 80% Tint
]
plt.rcParams['axes.prop_cycle'] = cycler(color=my_palette)


def plot_actual_paths():
    """
    Plottet alle actual paths für alle Konfigurationen.
    """
    fig, ax = plt.subplots(figsize=(7.29,2))
    for idx, folder in enumerate(folders):
        path = os.path.join(folder, actual_path_file)
        if os.path.isfile(path):
            df = pd.read_csv(path)
            ax.plot(
                df['x'], df['y'],
                #color=colors[idx % len(colors)], linestyle=linestyles[idx % len(linestyles)],
                linewidth=1.5, label=labels[idx]
            )
        else:
            print(f"Datei nicht gefunden: {path}")
    
        # --- Straße von y=0 bis y=0.5 mit zwei Spuren skizzieren ---
    # Dynamisch X-Grenzen ermitteln, damit sich der Untergrund anpasst
    x_min, x_max = ax.get_xlim()
    # Graues Straßenband (Höhe 0.5 m, ab y=0)
    road = Rectangle(
        (x_min, 0.0),         # linke untere Ecke bei y=0
        x_max - x_min,        # Breite
        0.5,                  # Höhe
        facecolor='gray',
        alpha=0.3,
        zorder=0)
    
    #ax.add_patch(road)
    # Fahrbahnrand oben und unten (weiße durchgezogene Linien)
    ax.plot([x_min, x_max], [0.0, 0.0], color='white', linewidth=1.5, zorder=1)
    ax.plot([x_min, x_max], [0.5, 0.5], color='white', linewidth=1.5, zorder=1)
    # Mittellinie (gestrichelt) bei y=0.25
    ax.plot([x_min, x_max], [0.25, 0.25],
            color='white', linestyle='--', linewidth=1.5, zorder=1)
    # -------------------------------------------------------------------


    # Hindernis
    ox = obstacle['obsXrl']
    oy = 0.0
    ow = obstacle['obslength']
    oh = obstacle['obsYrl']
    # Grau gefüllt
    obs_patch = Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5)
    # Safezone (rot, gestrichelt)
    sz_patch = Rectangle(
        (ox - safezone, oy - safezone),
        ow + 2*safezone,
        oh + 2*safezone,
        fill=False, linestyle='--', edgecolor='red', linewidth=1.5
    )
    ax.add_patch(obs_patch)
    ax.add_patch(sz_patch)

    #ax.set_title('Actual Paths Vergleich mit Hindernis')
    ax.set_xlabel('x in m')
    ax.set_ylabel('y in m')
    ax.legend(loc='best', fontsize='small')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.show()
    #plt.tight_layout()
    #plt.savefig("actualPath.pgf")


def plot_single_with_predictions(folder_index):
    """
    Plottet actual path und prediction-Trajektorien für eine Konfiguration.
    Actual path blau, Predictions rot gestrichelt.
    """
    if not 0 <= folder_index < len(folders):
        raise IndexError(f"Index außerhalb gültigen Bereichs: 0-{len(folders)-1}")

    folder = folders[folder_index]
    label = labels[folder_index]
    fig, ax = plt.subplots(figsize=(8, 6))

    # Actual path
    path_act = os.path.join(folder, actual_path_file)
    if os.path.isfile(path_act):
        df_act = pd.read_csv(path_act)
        ax.plot(
            df_act['x'], df_act['y'],
            color='blue', linestyle='-', linewidth=2,
            label=f"Actual {label}"
        )
    else:
        print(f"Actual-Datei nicht gefunden: {path_act}")

    # Predictions
    path_pred = os.path.join(folder, predictions_file)
    if os.path.isfile(path_pred):
        df_pred = pd.read_csv(path_pred)
        for traj_id, traj in df_pred.groupby('trajectory_id'):
            ax.plot(
                traj['x'], traj['y'],
                color='green', linestyle='--', linewidth=1, alpha=0.7,
                label="Prediction" if traj_id == 0 else None
            )
    else:
        print(f"Predictions-Datei nicht gefunden: {path_pred}")

    # Hindernis
    ox = obstacle['obsXrl']
    oy = 0.0
    ow = obstacle['obslength']
    oh = obstacle['obsYrl']
    ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
    ax.add_patch(Rectangle(
        (ox - safezone, oy - safezone),
        ow + 2*safezone,
        oh + 2*safezone,
        fill=False, linestyle='--', edgecolor='red'
    ))

    ax.set_title(f'Konfiguration: {label} mit Hindernis')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.legend(loc='best', fontsize='small')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    plt.tight_layout()
    plt.show()


def plot_all_actual_theta():
    """
    Plottet alle actual theta-Verläufe in einem gemeinsamen Plot.
    """
    plt.figure(figsize=(10, 5))
    for idx, folder in enumerate(folders):
        theta_path = os.path.join(folder, actual_theta_file)
        if os.path.isfile(theta_path):
            df_theta = pd.read_csv(theta_path)
            plt.plot(df_theta['theta'], color=colors[idx % len(colors)],
                     linestyle=linestyles[idx % len(linestyles)],
                     linewidth=1.5, label=labels[idx])
        else:
            print(f"Actual Theta-Datei nicht gefunden: {theta_path}")
    plt.title('Actual Theta Vergleich aller Konfigurationen')
    plt.xlabel('Zeit Schritt')
    plt.ylabel('Theta (rad)')
    plt.legend(loc='best', fontsize='small', ncol=2)
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_single_theta_with_predictions(folder_index):
    """
    Plottet actual theta und predicted theta für eine einzelne Konfiguration.
    Actual Theta blau, Predicted Theta rot gestrichelt.
    """
    if not 0 <= folder_index < len(folders):
        raise IndexError(f"Index außerhalb gültigen Bereichs: 0-{len(folders)-1}")
    folder = folders[folder_index]
    label = labels[folder_index]

    plt.figure(figsize=(8, 5))
    # Actual Theta
    theta_path = os.path.join(folder, actual_theta_file)
    if os.path.isfile(theta_path):
        df_theta = pd.read_csv(theta_path)
        plt.plot(df_theta['theta'], color='blue', linestyle='-', linewidth=2, label=f"Actual Theta {label}")
    else:
        print(f"Actual Theta-Datei nicht gefunden: {theta_path}")
    # Predicted Theta
    pred_path = os.path.join(folder, predicted_theta_file)
    if os.path.isfile(pred_path):
        df_pred = pd.read_csv(pred_path)
        for traj_id, traj in df_pred.groupby('trajectory_id'):
            plt.plot(traj['theta'], color='red', linestyle='--', linewidth=1,
                     alpha=0.7, label="Predicted Theta" if traj_id == 0 else None)
    else:
        print(f"Predicted Theta-Datei nicht gefunden: {pred_path}")

    plt.title(f'Konfiguration: {label} (Theta actual & predicted)')
    plt.xlabel('Zeit Schritt')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def plot_solve_times_single(folder_index):
    """
    Plottet die Laufzeit pro Zeitschritt für eine einzelne Konfiguration.
    :param folder_index: Index in der `folders`-Liste.
    """
    if not 0 <= folder_index < len(folders):
        raise IndexError(f"Index außerhalb gültigen Bereichs: 0-{len(folders)-1}")
    folder = folders[folder_index]
    label = labels[folder_index]
    path = os.path.join(folder, solve_times_file)
    if not os.path.isfile(path):
        print(f"Solve times-Datei nicht gefunden: {path}")
        return

    df = pd.read_csv(path)
    df.index.name = 'iteration'
    plt.figure(figsize=(8, 5))
    plt.plot(df['solve_time'], linestyle='-', label=f"Solve Time {label}")
    plt.title(f'Laufzeit pro Zeitschritt: {label}')
    plt.xlabel('Iteration')
    plt.ylabel('Zeit (s)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_solve_times_single2(folder_index):
    """
    Plottet die Laufzeit pro Zeitschritt für eine einzelne Konfiguration,
    verwendet dabei die Spalte 'x' auf der x-Achse und zeigt zusätzlich:
      • eine Linie für die Durchschnittszeit (in s und ms)
      • eine Linie für den Schwellenwert von 100 ms
      • Legende über dem Diagramm, keine Titel oder Labels oberhalb
    """
    if not 0 <= folder_index < len(folders):
        raise IndexError(f"Index außerhalb gültigen Bereichs: 0-{len(folders)-1}")
    folder = folders[folder_index]
    # config_label wird nicht mehr als Überschrift genutzt
    path = os.path.join(folder, solve_times_file)
    if not os.path.isfile(path):
        print(f"Solve times-Datei nicht gefunden: {path}")
        return

    df = pd.read_csv(path)
    if 'x' not in df.columns or 'solve_time' not in df.columns:
        raise KeyError("Spalte 'x' oder 'solve_time' nicht gefunden.")

    # Durchschnittszeit berechnen
    mean_time = df['solve_time'].mean()
    mean_time_ms = mean_time * 1000

    fig, ax = plt.subplots(figsize=(7.29, 4.2))

    # Solve-Times
    ax.plot(df['x'], df['solve_time'], linestyle='-', marker='o', label="Lösungszeit")

    # Durchschnittslinie
    ax.axhline(mean_time, color='orange', linestyle='--',
               label=f'Durchschnitt: {mean_time_ms:.0f} ms')

    # Schwellenwert-Linie (0.1 s = 100 ms)
    ax.axhline(0.1, color='red', linestyle=':',
               label='Schwellenwert: 100 ms')

    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'Laufzeit [s]')
    ax.grid(True)

    # Legende oberhalb des Plots
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02),
              ncol=3, frameon=True)

    plt.tight_layout()
    plt.show()



def plot_solve_times_summary():
    """
    Plottet Zusammenfassung der Solve-Times für alle Konfigurationen:
    - Gesamtzeit
    - Durchschnittszeit pro Zeitschritt
    - Schnellste und langsamste Zeit
    """
    totals, means, mins, maxs = [], [], [], []
    for folder in folders:
        path = os.path.join(folder, solve_times_file)
        if os.path.isfile(path):
            df = pd.read_csv(path)
            times = df['solve_time']
            totals.append(times.sum())
            means.append(times.mean())
            mins.append(times.min())
            maxs.append(times.max())
        else:
            totals.append(0)
            means.append(0)
            mins.append(0)
            maxs.append(0)
            print(f"Solve times-Datei nicht gefunden: {path}")

    x = range(len(folders))
    width = 0.2

    plt.figure(figsize=(12, 6))
    #plt.bar([i - 1.5*width for i in x], totals, width, label='Totalzeit')
    plt.bar([i - 0.5*width for i in x], means, width, label='Mittelzeit/Schritt')
    plt.bar([i + 0.5*width for i in x], mins, width, label='Schnellste Zeit')
    plt.bar([i + 1.5*width for i in x], maxs, width, label='Langsamste Zeit')

    plt.xticks(list(x), labels, rotation=45, ha='right')
    plt.title('Solve-Times Zusammenfassung aller Konfigurationen')
    plt.xlabel('Konfiguration')
    plt.ylabel('Zeit (s)')
    plt.legend()
    plt.grid(axis='y')
    plt.tight_layout()
    plt.show()

    return totals, means, mins, maxs

def plot_control_inputs(folder_index):
    """
    Plottet die Steuergrößen (u1-u4) für eine einzelne Konfiguration.
    :param folder_index: Index in der `folders`-Liste.
    """
    if not 0 <= folder_index < len(folders):
        raise IndexError(f"Index außerhalb gültigen Bereichs: 0-{len(folders)-1}")
    folder = folders[folder_index]
    ci_path = os.path.join(folder, control_inputs_file)
    if not os.path.isfile(ci_path):
        print(f"Control-Inputs-Datei nicht gefunden: {ci_path}")
        return

    df_ci = pd.read_csv(ci_path)
    fig, ax = plt.subplots(figsize=(8, 5))
    t = df_ci.index.values

    for col in ['u1','u2','u3','u4']: 
        if col in df_ci.columns:
            ax.plot(t, df_ci[col], label=col)
    
    ax.set_title(f'Steuergrößen (u) für {labels[folder_index]}')
    ax.set_xlabel('Zeit (Schritte)')
    ax.set_ylabel('u-Wert')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()
    #plt.tight_layout()
    #plt.savefig("control_inputs.pgf")

def compute_errors(ref_folder, act_folder, N=1000):
    """
    Vergleicht Soll- vs. Ist-Trajektorie über einen
    normalisierten Parameter s in [0,1].
    N = Anzahl Auswertungspunkte.
    Liefert: max_error, rmse.
    """
    # 1) Dateien einlesen
    f_ref = os.path.join(ref_folder, 'mpc_data_actual_path.csv')
    f_act = os.path.join(act_folder, 'mpc_data_actual_path.csv')
    df_ref = pd.read_csv(f_ref).sort_values('t').reset_index(drop=True)
    df_act = pd.read_csv(f_act).sort_values('t').reset_index(drop=True)

    # 2) Normalisiertes s für jede Trajektorie
    t_ref = df_ref['t'].values
    t_act = df_act['t'].values
    s_ref = (t_ref - t_ref[0]) / (t_ref[-1] - t_ref[0])
    s_act = (t_act - t_act[0]) / (t_act[-1] - t_act[0])

    # 3) Interpolationsfunktionen für x,y
    fx_ref = interp1d(s_ref, df_ref['x'].values, kind='linear')
    fy_ref = interp1d(s_ref, df_ref['y'].values, kind='linear')
    fx_act = interp1d(s_act, df_act['x'].values, kind='linear')
    fy_act = interp1d(s_act, df_act['y'].values, kind='linear')

    # 4) Gemeinsamer s-Vektor
    s_common = np.linspace(0, 1, N)

    # 5) Abtasten
    x_ref_c = fx_ref(s_common)
    y_ref_c = fy_ref(s_common)
    x_act_c = fx_act(s_common)
    y_act_c = fy_act(s_common)

    # 6) Fehler
    dx = x_act_c - x_ref_c
    dy = y_act_c - y_ref_c
    err = np.abs(dy)

    # 7) Metriken
    max_error = err.max()
    rmse      = np.sqrt(np.mean(err**2))
    return max_error, rmse

def compute_lateral_errors(ref_folder, act_folder, N=1000):
    """
    Vergleicht Soll- vs. Ist-Trajektorie über den gemeinsamen x-Bereich
    und liefert: max_lat_error, rmse_lat.
    """
    # 1) Dateien einlesen
    f_ref = os.path.join(ref_folder, actual_path_file)
    f_act = os.path.join(act_folder, actual_path_file)
    df_ref = pd.read_csv(f_ref).sort_values('x').drop_duplicates('x')
    df_act = pd.read_csv(f_act).sort_values('x').drop_duplicates('x')

    # 2) Interpolationsfunktionen y = f(x)
    f_ref_y = interp1d(df_ref['x'], df_ref['y'],
                       kind='linear',
                       bounds_error=False,
                       fill_value='extrapolate')
    f_act_y = interp1d(df_act['x'], df_act['y'],
                       kind='linear',
                       bounds_error=False,
                       fill_value='extrapolate')

    # 3) Gemeinsamen x-Bereich bestimmen
    x_min = max(df_ref['x'].min(), df_act['x'].min())
    x_max = min(df_ref['x'].max(), df_act['x'].max())
    x_common = np.linspace(x_min, x_max, N)

    # 4) y-Werte auslesen und Querfehler berechnen
    y_ref_c = f_ref_y(x_common)
    y_act_c = f_act_y(x_common)
    err_lat = np.abs(y_act_c - y_ref_c)

    # 5) Metriken
    max_lat_error = err_lat.max()
    rmse_lat      = np.sqrt(np.mean(err_lat**2))
    return max_lat_error, rmse_lat


def plot_error_vs_x(ref_index, act_index, N=1000):
    ref_folder = folders[ref_index]
    act_folder = folders[act_index]
    df_ref = pd.read_csv(os.path.join(ref_folder, actual_path_file)).sort_values('x').drop_duplicates('x')
    df_act = pd.read_csv(os.path.join(act_folder, actual_path_file)).sort_values('x').drop_duplicates('x')

    f_ref_y = interp1d(df_ref['x'], df_ref['y'], kind='linear', bounds_error=False, fill_value='extrapolate')
    f_act_y = interp1d(df_act['x'], df_act['y'], kind='linear', bounds_error=False, fill_value='extrapolate')

    x_min = max(df_ref['x'].min(), df_act['x'].min())
    x_max = min(df_ref['x'].max(), df_act['x'].max())
    x_common = np.linspace(x_min, x_max, N)

    err_lat = np.abs(f_act_y(x_common) - f_ref_y(x_common))

    plt.figure(figsize=(7.29, 4.2))
    plt.plot(x_common, err_lat, linestyle='-', linewidth=1.5)
    plt.xlabel('X-Position [m]')
    plt.ylabel('Querfehler [m]')
    plt.title(f'Querfehler vs. X: {labels[act_index]} vs. {labels[ref_index]}')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_all_lateral_errors(ref_index=0, act_indices=None, N=1000):
    """
    Querfehler vs. x – Legende steht oberhalb der Figure.
    """
    if act_indices is None:
        act_indices = [i for i in range(len(folders)) if i != ref_index]

    # ---------------- Referenz vorbereiten -----------------
    df_ref = pd.read_csv(os.path.join(folders[ref_index], actual_path_file))
    df_ref = df_ref.sort_values('x').drop_duplicates('x')
    f_ref_y = interp1d(df_ref['x'], df_ref['y'],
                       kind='linear', bounds_error=False, fill_value='extrapolate')

    # Gemeinsamen x-Bereich ermitteln
    x_min, x_max = df_ref['x'].min(), df_ref['x'].max()
    for i in act_indices:
        df_act = pd.read_csv(os.path.join(folders[i], actual_path_file))
        x_min = max(x_min, df_act['x'].min())
        x_max = min(x_max, df_act['x'].max())
    x_common = np.linspace(x_min, x_max, N)

    # ---------------- Plot -----------------
    fig, ax = plt.subplots(figsize=(7.29, 4))
    for i in act_indices:
        df_act = pd.read_csv(os.path.join(folders[i], actual_path_file))
        df_act = df_act.sort_values('x').drop_duplicates('x')
        f_act_y = interp1d(df_act['x'], df_act['y'],
                           kind='linear', bounds_error=False, fill_value='extrapolate')

        err_lat = np.abs(f_act_y(x_common) - f_ref_y(x_common))

        ax.plot(x_common, err_lat,
                linestyle=linestyles[i % len(linestyles)],
                linewidth=1.5,
                label=labels[i])

    ax.set_xlabel(r'$x$ in $\mathrm{m}$', fontsize=11)
    ax.set_ylabel(r'$e(x)$ in $\mathrm{m}$', fontsize=11)
    ax.grid(True)

    # -------- Legende OBERHALB der gesamten Figure ----------
    handles, labls = ax.get_legend_handles_labels()
    fig.legend(handles, labls,
               loc='upper center',          # oberhalb
               bbox_to_anchor=(0.5, 0.98),  # x-Mitte, y-Position
               ncol=len(labls),             # alles in eine Zeile
               frameon=True)

    # Platz für die Legende lassen
    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.show()

def plot_multiple_with_predictions(folder_indices):
    """
    Subplots (untereinander) mit gemeinsamer Legende oberhalb.
    """
    n = len(folder_indices)
    fig, axes = plt.subplots(
        nrows=n, ncols=1,
        figsize=(7.29, 2.6*n),
        sharex=True, sharey=True
    )
    if n == 1:                          # nur ein Subplot → Liste
        axes = [axes]

    # ---------- Zeichnen ----------
    legend_handles, legend_labels = None, None
    for ax, idx in zip(axes, folder_indices):
        folder = folders[idx]
        label  = labels[idx]

        # Actual
        df_act = pd.read_csv(os.path.join(folder, actual_path_file))
        ln_act, = ax.plot(df_act['x'], df_act['y'],
                          linestyle='-', linewidth=2,
                          label=f"Tatsächlicher Fahrtverlauf")

        # Predictions
        df_pred = pd.read_csv(os.path.join(folder, predictions_file))
        for traj_id, traj in df_pred.groupby('trajectory_id'):
            ln_pred, = ax.plot(traj['x'], traj['y'],
                               color=my_palette[1], linestyle='--',
                               linewidth=1, alpha=0.7,
                               label="Vorhergesagte Trajektorie" if traj_id == 0 else None)

        # Hindernis
        ox, oy = obstacle['obsXrl'], 0.0
        ow, oh = obstacle['obslength'], obstacle['obsYrl']
        ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
        ax.add_patch(Rectangle((ox-safezone, oy-safezone),
                               ow+2*safezone, oh+2*safezone,
                               fill=False, linestyle='--', edgecolor='red'))

        ax.set_title(label, fontsize=11)
        ax.set_ylabel(r'$y$ in $\mathrm{m}$', fontsize=11)
        ax.grid(True)

        # Handles/Labels nur einmal sammeln (vom ersten Ax)
        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()

    # ---------- gemeinsame Elemente ----------
    axes[-1].set_xlabel(r'$x$ in $\mathrm{m}$')

    # Figure-Legende OBEN
    fig.legend(legend_handles, legend_labels,
               loc='upper center',
               bbox_to_anchor=(0.5, 0.98),    # x-Mitte, etwas oberhalb
               ncol=len(legend_labels),        # alles in eine Zeile
               frameon=True)

    # Platz für die Legende lassen
    plt.tight_layout(rect=[0, 0, 1, 0.91])      # oder plt.subplots_adjust(top=0.9)
    plt.show()



def plot_multiple_with_predictions_zoom(folder_indices):
    """
    Untereinander angeordnete Plots mit Zoom-Inset (x: 1.65–1.85 m, y: 0.35–0.45 m).
    • Vorhersagen im Hauptplot gelb (#FFC857).
    • Im Zoom-Fenster werden nur der tatsächliche Pfad und das Hindernis gezeigt.
    """
    n = len(folder_indices)
    fig, axes = plt.subplots(
        nrows=n, ncols=1, figsize=(7.29, 2.1*n),
        sharex=True, sharey=True
    )
    if n == 1:
        axes = [axes]

    legend_handles, legend_labels = None, None
    for ax, idx in zip(axes, folder_indices):
        folder = folders[idx]
        label  = labels[idx]

        # -------- Hauptkurven ------------------------------------
        df_act = pd.read_csv(os.path.join(folder, actual_path_file))
        ax.plot(df_act['x'], df_act['y'],
                lw=2, label="Tatsächlicher Fahrtverlauf")

        df_pred = pd.read_csv(os.path.join(folder, predictions_file))
        for i, (_, traj) in enumerate(df_pred.groupby('trajectory_id')):
            ax.plot(traj['x'], traj['y'],
                    color='#FFC857', ls='--', lw=1, alpha=0.8,
                    label="Vorhergesagte Trajektorie" if i == 0 else None)

        # -------- Hindernis --------------------------------------
        ox, oy = obstacle['obsXrl'], 0.0
        ow, oh = obstacle['obslength'], obstacle['obsYrl']
        ax.add_patch(Rectangle((ox, oy), ow, oh,
                               color='gray', alpha=0.5))
        ax.add_patch(Rectangle((ox-safezone, oy-safezone),
                               ow+2*safezone, oh+2*safezone,
                               fill=False, ls='--', ec='red'))

        ax.set_title(label, fontsize=11)
        ax.set_ylabel('y in m', fontsize=11)
        ax.grid(True)

        # -------- Zoom-Inset (ohne Predictions) ------------------
        axins = inset_axes(ax, width="15%", height="35%",
                           loc='upper right', borderpad=1)

        # nur tatsächliche Trajektorie zeichnen
        axins.plot(df_act['x'], df_act['y'], lw=2, color=my_palette[0])

        # Hindernis
        axins.add_patch(Rectangle((ox, oy), ow, oh,
                                  color='gray', alpha=0.5))
        axins.add_patch(Rectangle((ox-safezone, oy-safezone),
                                  ow+2*safezone, oh+2*safezone,
                                  fill=False, ls='--', ec='red'))

        # feste Zoom-Grenzen
        axins.set_xlim(1.65, 1.9)
        axins.set_ylim(0.28, 0.42)
        axins.set_xticks([]); axins.set_yticks([])

        mark_inset(ax, axins, loc1=2, loc2=4,
                   fc="none", ec="black", lw=0.8)

        # -------- Legende nur einmal sammeln ---------------------
        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()

    axes[-1].set_xlabel('x in m', fontsize=11)
    fig.legend(legend_handles, legend_labels,
               loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=len(legend_labels), frameon=True)
    

    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.show()



def plot_multiple_with_reference(folder_indices):
    """
    Untereinander angeordnete Sub-Plots mit
    • Actual-Pfad (durchgezogen)
    • Referenztrajektorie (strich-punktiert), falls vorhanden
    • gemeinsamer Legende oberhalb der Figure
    """
    n = len(folder_indices)
    fig, axes = plt.subplots(
        nrows=n, ncols=1,
        figsize=(7.29, 2.1 * n),      # gleicher Figure-Höhen-Faktor
        sharex=True, sharey=True
    )
    if n == 1:
        axes = [axes]                 # immer iterierbar

    legend_handles, legend_labels = None, None
    for ax, idx in zip(axes, folder_indices):
        folder = folders[idx]
        label  = labels[idx]

       
        # ---------- 1) Actual Path -------------------------------
        df_act = pd.read_csv(os.path.join(folder, actual_path_file))
        ln_act, = ax.plot(df_act['x'], df_act['y'],
                          lw=2, label="Tatsächlicher Fahrtverlauf")

        # ---------- 2) Referenztrajektorie (optional) ------------
        ref_file = os.path.join(folder, reference_file)
        if os.path.isfile(ref_file):
            df_ref = pd.read_csv(ref_file)
            ax.plot(df_ref['x'], df_ref['y'],
                    ls='-.', lw=1.5, color=my_palette[1],
                    label="Referenztrajektorie")
        else:
            print(f"Keine Referenz in {folder}, übersprungen.")

        # ---------- 3) Hindernis --------------------------------
        ox, oy = obstacle['obsXrl'], 0.0
        ow, oh = obstacle['obslength'], obstacle['obsYrl']
        ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
        ax.add_patch(Rectangle((ox - safezone, oy - safezone),
                               ow + 2*safezone, oh + 2*safezone,
                               fill=False, ls='--', ec='red'))

        # ---------- Achsentitel, Gitter usw. --------------------
        ax.set_title(label, fontsize=11)
        ax.set_ylabel(r'$y$ in $\mathrm{m}$',fontsize=11)
        ax.grid(True)

        # ---------- Legenden-Handles einmalig sammeln -----------
        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()

    # nur unterste Achse bekommt x-Beschriftung
    axes[-1].set_xlabel(r'$x$ in $\mathrm{m}$',fontsize=11)

    # gemeinsame Legende OBEN (gleiche Optik wie predictions-Plot)
    fig.legend(legend_handles, legend_labels,
               loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=len(legend_labels), frameon=True)

    # Layout anpassen, damit Legende nicht abgeschnitten wird
    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.show()

import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

def plot_multiple_with_reference2(folder_indices):
    """
    Untereinander angeordnete Sub-Plots mit:
    • Tatsächlicher Fahrtverlauf (durchgezogen)
    • Vorhergesagte Trajektorie (gestrichelt)
    • Referenztrajektorie (strich-punktiert), falls vorhanden
    • Gemeinsamer Legende oberhalb der Figure (zentriert)
    """
    n = len(folder_indices)
    fig, axes = plt.subplots(
        nrows=n, ncols=1,
        figsize=(7.29, 2.1 * n),
        sharex=True, sharey=True
    )
    if n == 1:
        axes = [axes]

    for ax, idx in zip(axes, folder_indices):
        folder = folders[idx]
        label  = labels[idx]

        # ---------- Vorhergesagte Trajektorie --------------------
        df_pred = pd.read_csv(os.path.join(folder, predictions_file))
        traj_ids = df_pred['trajectory_id'].unique()
        for i, (traj_id, traj) in enumerate(df_pred.groupby('trajectory_id')):
            ax.plot(traj['x'], traj['y'],
                    color=my_palette[1], linestyle='--',
                    linewidth=1, alpha=0.7,
                    label="Vorhergesagte Trajektorie" if i == 0 else None)

        # ---------- Tatsächlicher Fahrtverlauf ------------------
        df_act = pd.read_csv(os.path.join(folder, actual_path_file))
        ax.plot(df_act['x'], df_act['y'],
                lw=2, color='C0',alpha=0.8, label="Tatsächlicher Fahrtverlauf")

        # ---------- Referenztrajektorie (optional) --------------
        ref_file = os.path.join(folder, reference_file)
        if os.path.isfile(ref_file):
            df_ref = pd.read_csv(ref_file)
            ax.plot(df_ref['x'], df_ref['y'],
                    ls='-.', lw=1.5, color=my_palette[3],alpha=0.6,
                    label="Referenztrajektorie")
        else:
            print(f"Keine Referenz in {folder}, übersprungen.")

        # ---------- Hindernis -----------------------------------
        ox, oy = obstacle['obsXrl'], 0.0
        ow, oh = obstacle['obslength'], obstacle['obsYrl']
        ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
        ax.add_patch(Rectangle((ox - safezone, oy - safezone),
                               ow + 2*safezone, oh + 2*safezone,
                               fill=False, ls='--', ec='red'))

        # ---------- Achsentitel, Gitter usw. --------------------
        ax.set_title(label, fontsize=11)
        ax.set_ylabel(r'$y$ in $\mathrm{m}$', fontsize=11)
        ax.grid(True)

    # Nur unterste Achse bekommt x-Beschriftung
    axes[-1].set_xlabel(r'$x$ in $\mathrm{m}$', fontsize=11)

    # ---------- Gemeinsame Legende oberhalb --------------------
    legend_handles = [
        Line2D([0], [0], color='C0',alpha=0.9, lw=2, label="Tatsächlicher Fahrtverlauf"),
        Line2D([0], [0], color=my_palette[1], lw=1, linestyle='--', label="Vorhergesagte Trajektorie"),
        Line2D([0], [0], color=my_palette[3],alpha=0.6, lw=1.5, linestyle='-.', label="Referenztrajektorie"),
    ]

    fig.legend(handles=legend_handles,
               loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=3, frameon=True)

    # Platz für die Legende lassen
    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.show()

def plot_multiple_theta(folder_indices):
    """
    Subplots im 2-Spalten-Raster mit:
    • je Plot: theta über x
    • Titel über jedem Plot
    • y-Achsenticks und Label nur links
    • x-Achsenticks und Label nur unten
    • Gemeinsame Legende über der Figure
    """
    n = len(folder_indices)
    ncols = 2
    nrows = math.ceil(n / ncols)

    fig, axes = plt.subplots(
        nrows=nrows, ncols=ncols,
        figsize=(3.65 * ncols, 2.1 * nrows),
        sharex=True, sharey=True
    )
    axes = axes.flatten()

    legend_handles, legend_labels = None, None

    for plot_no, (ax, idx) in enumerate(zip(axes, folder_indices)):
        folder = folders[idx]
        label  = labels[idx]
        theta_path = os.path.join(folder, actual_theta_file)
        path_path  = os.path.join(folder, actual_path_file)

        if os.path.isfile(theta_path) and os.path.isfile(path_path):
            df_theta = pd.read_csv(theta_path)
            df_path  = pd.read_csv(path_path)
            ln, = ax.plot(df_path['x'], df_theta['theta'],
                          linestyle='-', linewidth=1.5,
                          label=r"Orientierung $\psi$")
        else:
            missing = theta_path if not os.path.isfile(theta_path) else path_path
            ax.text(0.5, 0.5,
                    f"Datei nicht gefunden:\n{missing}",
                    ha='center', va='center', color='red')
            ax.axis('off')
            continue

        ax.set_title(label, fontsize=11)
        ax.grid(True)

        # Nur linke Spalte → y-Achsenbeschriftung
        if plot_no % ncols == 0:
            ax.set_ylabel(r'$\psi$ in $\mathrm{rad}$', fontsize=11)
        else:
            ax.tick_params(labelleft=False)

        # Nur unterste Zeile → x-Achsenbeschriftung
        if plot_no >= (nrows - 1) * ncols:
            ax.set_xlabel(r'$x$ in $\mathrm{m}$', fontsize=11)
        else:
            ax.tick_params(labelbottom=False)

        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()

    # Überzählige leere Subplots entfernen
    for ax in axes[n:]:
        ax.remove()

    # Gemeinsame Legende oberhalb
    fig.legend(legend_handles, legend_labels,
               loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=len(legend_labels), frameon=True)

    plt.tight_layout(rect=[0, 0, 1, 0.91])
    plt.show()

def plot_multiple_control_inputs(folder_indices):
    """
    Erzeugt für jede Index in folder_indices einen eigenen Subplot
    untereinander mit den Steuergrößen u1–u4.
    """
    n = len(folder_indices)
    fig, axes = plt.subplots(
        nrows=n, ncols=1,
        figsize=(12, 2.1*n),
        sharex=True
    )

    # Falls nur ein Subplot zurückkommt:
    if n == 1:
        axes = [axes]

    for ax, idx in zip(axes, folder_indices):
        folder = folders[idx]
        label  = labels[idx]
        ci_path = os.path.join(folder, control_inputs_file)

        if not os.path.isfile(ci_path):
            ax.text(0.5, 0.5, f"Nicht gefunden:\n{ci_path}",
                     ha='center', va='center', color='red')
            ax.set_ylabel(label)
            continue

        df_ci = pd.read_csv(ci_path)
        t = df_ci.index.values

        # Alle u-Spalten plotten, falls vorhanden
        for col in ['u1','u2','u3','u4']:
            if col in df_ci.columns:
                ax.plot(t, df_ci[col], label=col, linewidth=1)

        ax.set_ylabel(label)
        ax.grid(True)
        ax.legend(fontsize='x-small')

    axes[-1].set_xlabel('Zeit (Schritte)')
    plt.tight_layout()
    plt.show()

def plot_multiple_control_inputs_vs_x(folder_indices):
        """
        Erzeugt für jede Index in folder_indices einen eigenen Subplot
        untereinander mit den Steuergrößen u1–u4 gegen die X-Position.
        Variante 1: Annahme gleiche Länge von actual_path und control_inputs.
        """
        
        n = len(folder_indices)
        ncols = 2
        nrows = math.ceil(n / ncols)
        fig, axes = plt.subplots(nrows=nrows, ncols=ncols,
                             figsize=(7.29 * ncols, 1.8 * nrows),
                             sharex=False)
    # axes flach machen für einfachen Zugriff
        axes = axes.flatten()

        # Falls nur ein Subplot erzeugt wurde, packen wir ihn in eine Liste
        if n == 1:
            axes = [axes]

        for ax, idx in zip(axes, folder_indices):
            folder = folders[idx]
            label  = labels[idx]

            # CSVs einlesen
            ci_path = os.path.join(folder, control_inputs_file)
            ap_path = os.path.join(folder, actual_path_file)
            if not os.path.isfile(ci_path) or not os.path.isfile(ap_path):
                ax.text(0.5, 0.5, f"Datei fehlt:\n{ci_path}\n{ap_path}",
                        ha='center', va='center', color='red')
                ax.set_ylabel(label)
                continue

            df_ci = pd.read_csv(ci_path)
            df_ap = pd.read_csv(ap_path)
            # X-Positionen 1:1 zum Index
            x = df_ap['x'].values

            # Alle u-Spalten plotten
            for col in ['u1','u2','u3','u4']:
                if col in df_ci.columns:
                    ax.plot(x, df_ci[col].values, label=col, linewidth=1)

            ax.set_ylabel(label)
            ax.grid(True)
            ax.legend(fontsize='x-small')

        axes[-1].set_xlabel('X [m]')
        plt.tight_layout()
        plt.savefig('control_inputs_vs_x.svg', format='svg')
        plt.show()

def plot_multiple_control_inputs_vs_x2(folder_indices,
                                       control_inputs_file='mpc_data_control_inputs.csv',
                                       actual_path_file='mpc_data_actual_path.csv'):
    """
    Für jeden Index in folder_indices einen Sub-Plot (Gitter 2 Spalten) mit
    u1–u4 über x.
    • Legende für u-Kurven einmalig oberhalb der Figure
    • Titel je Sub-Plot = Konfig-Label
    • x-Beschriftung nur in der untersten Zeile
    • y-Beschriftung ('rad/s') + Ticks nur links außen
    """
    n = len(folder_indices)
    ncols = 2
    nrows = math.ceil(n / ncols)

    fig, axes = plt.subplots(
        nrows=nrows, ncols=ncols,
        figsize=(3.65 * ncols, 2.3 * nrows),
        sharex=False
    )
    axes = axes.flatten()

    legend_handles, legend_labels = None, None

    for plot_no, (ax, idx) in enumerate(zip(axes, folder_indices)):
        folder = folders[idx]
        label  = labels[idx]

        ci_path = os.path.join(folder, control_inputs_file)
        ap_path = os.path.join(folder, actual_path_file)

        if not os.path.isfile(ci_path) or not os.path.isfile(ap_path):
            ax.text(0.5, 0.5, "Datei fehlt", ha='center', va='center', color='red')
            ax.axis('off')
            continue

        df_ci = pd.read_csv(ci_path)
        df_ap = pd.read_csv(ap_path)

        if 't' in df_ci.columns and 't' in df_ap.columns:
            df_ap = df_ap.sort_values('t')
            x_ci = np.interp(df_ci['t'], df_ap['t'], df_ap['x'])
        else:
            Nmin = min(len(df_ci), len(df_ap))
            x_ci = df_ap['x'].values[:Nmin]
            df_ci = df_ci.iloc[:Nmin]

        name_map = {'u1': r'$u_1$', 'u2': r'$u_2$', 'u3': r'$u_3$', 'u4': r'$u_4$'}
        for col in ['u1', 'u2', 'u3', 'u4']:
            if col in df_ci.columns:
                ln, = ax.plot(x_ci, df_ci[col].values, lw=1, label=name_map[col])

        ax.set_title(label, fontsize=10)
        ax.grid(True)

        # y-Achsenbeschriftung + Ticks nur links
        if plot_no % ncols == 0:
            ax.set_ylabel(r'$\omega$ in $\mathrm{rad/s}$')
        else:
            ax.tick_params(labelleft=False)

        if legend_handles is None:
            legend_handles, legend_labels = ax.get_legend_handles_labels()

    for ax in axes[-ncols:]:
        ax.set_xlabel(r'$x$ in $\mathrm{m}$')

    for ax in axes[n:]:
        ax.axis('off')

    fig.legend(legend_handles, legend_labels,
               loc='upper center', bbox_to_anchor=(0.5, 0.98),
               ncol=len(legend_labels), frameon=True)

    plt.tight_layout(rect=[0, 0, 1, 0.89])
    plt.show()





if __name__ == '__main__':
    # Beispielaufrufe:
    ref = folders[0]
    print("Vergleich über normalisiertes s∈[0,1]:")
    for idx, fol in enumerate(folders[1:], start=1):
        me_lat, r_lat = compute_lateral_errors(ref, fol)
        print(f"{labels[idx]:20s} → MaxLatErr = {me_lat:.4f} m,  RMSE_Lat = {r_lat:.4f} m")
    
    
    #plot_all_lateral_errors(ref_index=0)
    #plot_multiple_with_predictions_zoom([0, 1])
    plot_multiple_with_predictions([ 1,2])
    #plot_multiple_with_reference2([ 1,2])
    #plot_multiple_control_inputs_vs_x2([1,2])
    #plot_multiple_theta([0, 1])
    #plot_actual_paths()
    
    #plot_control_inputs(1)
    #plot_error_vs_x(ref_index=0, act_index=2, N=1000)
    #plot_single_with_predictions(2)    
    #plot_all_actual_theta()
    #plot_single_theta_with_predictions(2)
    plot_solve_times_single(3)
    plot_solve_times_single2(3)
    plot_solve_times_summary()
    #plot_multiple_control_inputs([ 0,1,2,3])
    
    totals, means, mins, maxs = plot_solve_times_summary()

    # Option A: Einfaches Listen-Print
    print("Gesamtzeiten pro Konfiguration:    ", [float(t) for t in totals])
    print("Durchschnittszeiten pro Schritt:  ", [float(m) for m in means])
    print("Minimale Solve-Times:             ", [float(mn) for mn in mins])
    print("Maximale Solve-Times:             ", [float(mx) for mx in maxs])
    
    # Option B: Pro Konfiguration schön formatieren
    for label, tot, avg, lo, hi in zip(labels, totals, means, mins, maxs):
       print(f"{label:>5s} → total: {tot:.4f}s, mean: {avg:.4f}s, "
            f"min: {lo:.4f}s, max: {hi:.4f}s")

    
    pass
