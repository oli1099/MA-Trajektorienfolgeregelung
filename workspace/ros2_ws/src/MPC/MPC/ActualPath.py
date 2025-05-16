#!/usr/bin/env python3
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
from scipy.interpolate import interp1d


# Liste der Ordner, die jeweils actual_path, predictions und theta CSVs enthalten
folders = [
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=1_Np=15_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=3_Np=15_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=15_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=10_Np=15_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=15_Np=15_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=43_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=40_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=35_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=30_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=25_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=20_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=10_Q=100_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=15_Q=0.1_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=15_Q=1_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=5_Np=15_Q=10_T=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPC_dynObs_Nc=15_Np=15_Q=100_T=0.1_Lidar',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.15',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.3',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/PP_LA=0.1',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/Trajectory',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_Q=100_T=30_Ts=0.2',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_Q=100_T=30_Ts=0.1',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_TracetoryTime',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_T=18',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.1',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.2',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.3',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.2_V_ref=0.2',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.05_V_ref=0.2',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.05_V_ref=0.3',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.01_V_ref=0.3',

]

# Dateinamen
actual_path_file        = 'mpc_data_actual_path.csv'
predictions_file        = 'mpc_data__predictions.csv'
actual_theta_file       = 'mpc_data_actual_theta.csv'
predicted_theta_file    = 'mpc_data_predicted_theta.csv'
solve_times_file        = 'mpc_data_solve_times.csv'
control_inputs_file     = 'mpc_data_control_inputs.csv'

# Obstacle-Parameter
obstacle = {
    'obsXrl': 1.0,      # x-Koordinate (links unten)
    'obsYrl': 0.25,     # y-Halbhöhe
    'obslength': 0.75   # Breite in x-Richtung
}
safezone = 0.1

# Labels und Plot-Stile
labels = [
   # 'Nc=1', 
    #'Nc=3', 
    #'Nc=5',
    #'Nc=10', 
    #'Nc=15', 
    #'Np=43', 
    #'Nc=5,Np=40',
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
    'Ref',
    #'MPCTrajectory_Q=100_T=30_Ts=0.2',
    #'MPCTrajectory_Q=100_T=30_Ts=0.1',
    #'N=15',
    #'N=15_T=18',
    'L=0.1_V_ref=0.1',
    'L=0.1_V_ref=0.2',
    'L=0.1_V_ref=0.3',
    'L=0.2_V_ref=0.2',
    'L=0.05_V_ref=0.2',
    'L=0.05_V_ref=0.3',
    'L=0.01_V_ref=0.3',
]
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
linestyles = ['-', '--', '-.', ':']


def plot_actual_paths():
    """
    Plottet alle actual paths für alle Konfigurationen.
    """
    fig, ax = plt.subplots(figsize=(10, 6))

    for idx, folder in enumerate(folders):
        path = os.path.join(folder, actual_path_file)
        if os.path.isfile(path):
            df = pd.read_csv(path)
            ax.plot(
                df['x'], df['y'],
                color=colors[idx % len(colors)], linestyle=linestyles[idx % len(linestyles)],
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
        zorder=0
    )
    ax.add_patch(road)
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
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.legend(loc='best', fontsize='small')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    plt.tight_layout()
    plt.show()


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
    plt.plot(df['solve_time'], linestyle='-', marker='o', label=f"Solve Time {label}")
    plt.title(f'Laufzeit pro Zeitschritt: {label}')
    plt.xlabel('Iteration')
    plt.ylabel('Zeit (s)')
    plt.legend()
    plt.grid(True)
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
    plt.bar([i - 1.5*width for i in x], totals, width, label='Totalzeit')
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
    err = np.hypot(dx, dy)

    # 7) Metriken
    max_error = err.max()
    rmse      = np.sqrt(np.mean(err**2))
    return max_error, rmse
if __name__ == '__main__':
    # Beispielaufrufe:
    ref = folders[0]
    print("Vergleich über normalisiertes s∈[0,1]:")
    for idx, fol in enumerate(folders[1:], start=1):
        me, r = compute_errors(ref, fol)
        print(f"{labels[idx]:20s} → MaxErr = {me:.4f} m,  RMSE = {r:.4f} m")

    plot_actual_paths()
    plot_control_inputs(6)
    plot_single_with_predictions(5)    
    plot_all_actual_theta()
    plot_single_theta_with_predictions(5)
    plot_solve_times_single(5)
    plot_solve_times_summary()

    
    pass
