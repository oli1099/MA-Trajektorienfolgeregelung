import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter
from cycler import cycler

folders = [
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryMPC',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=1_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=3_Np=15_Q=100_T=0.1_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=3_Np=25_Q=100_T=0.1_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=5_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=20',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10_u=30',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.2_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=5_Np=25_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=10_Np=15_Q=100_T=0.1_k=10',
    #'/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=15_Np=15_Q=100_T=0.1_k=10',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/DynObs_Nc=15_Np=25_Q=100_T=0.1_k=10',
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
    r'$N_c=3$',
    #'Nc=3, Np=25', 
    r'$N_c=5$',
    #r'$\|\mathbf{u}\|\leq 10$',
    #r'$\|\mathbf{u}\|\leq 20$',
    #'Nc=5, Np=25 u=30',
    #'Nc=5, Np=25, Ts=0.2',
    r'$N_c=15$',
    #'Nc=15, Np=25',
    #'Nc=10, Np=15', 
    #'Nc=15, Np=15',
    #'Nc=25, Np=25', 
    #'Np=43', 
    #r'$T_s=0,1$',
    #r'$T_s=0,2$',
    r'$N_c=25$',
    r'$N_p=25$',
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
    #r'$N=25$',
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


def animate_single_with_predictions(folder_index, save_path="single_pred_animation.gif"):
    """
    Erstellt ein GIF, in dem bei jedem Frame:
      - der Ist-Pfad bis zu diesem Zeitschritt
      - die Vorhersage-Trajektorie für diesen Zeitschritt
    dargestellt werden.
    """
    # Validierung
    if not (0 <= folder_index < len(folders)):
        raise IndexError(f"folder_index muss zwischen 0 und {len(folders)-1} liegen.")
    folder = folders[folder_index]
    label  = labels[folder_index]

    # Daten einlesen
    df_act = pd.read_csv(os.path.join(folder, actual_path_file))
    df_pred = pd.read_csv(os.path.join(folder, predictions_file))

    # Zeit-Spalte vorbereiten
    if 't' not in df_act.columns and 'time_step' not in df_act.columns:
        df_act = df_act.reset_index().rename(columns={'index': 't'})
    else:
        if 'time_step' in df_act.columns:
            df_act = df_act.rename(columns={'time_step': 't'})

    if 'trajectory_id' not in df_pred.columns:
        raise KeyError("Die Predictions-Datei benötigt eine Spalte 'trajectory_id'.")

    n_frames = int(df_act['t'].max()) + 1

    fig, ax = plt.subplots(figsize=(7.29, 3), dpi=150)
    ax.set_xlabel(r'$x$ in $\mathrm{m}$')
    ax.set_ylabel(r'$y$ in $\mathrm{m}$')
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')

    all_x = pd.concat([df_act['x'], df_pred['x']])
    all_y = pd.concat([df_act['y'], df_pred['y']])
    ax.set_xlim(all_x.min(), all_x.max())
    ax.set_ylim(all_y.min(), all_y.max())

    ox, oy = obstacle['obsXrl'], 0.0
    ow, oh = obstacle['obslength'], obstacle['obsYrl']
    ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
    ax.add_patch(Rectangle((ox-safezone, oy-safezone),
                           ow+2*safezone, oh+2*safezone,
                           fill=False, linestyle='--', edgecolor='red'))

    line_act, = ax.plot([], [], color=my_palette[0], lw=2, label='Fahrtverlauf')
    line_pred, = ax.plot([], [], color=my_palette[1], ls='--', lw=1.5, alpha=0.8, label='Vorhersage')

    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=2, fontsize='small')

    ax.margins(0)

    def init():
        line_act.set_data([], [])
        line_pred.set_data([], [])
        return line_act, line_pred

    def update(frame):
        df_a = df_act[df_act['t'] <= frame]
        line_act.set_data(df_a['x'], df_a['y'])

        df_p = df_pred[df_pred['trajectory_id'] == frame]
        if not df_p.empty:
            line_pred.set_data(df_p['x'], df_p['y'])
        else:
            line_pred.set_data([], [])

        return line_act, line_pred

    anim = FuncAnimation(
        fig, update,
        frames=n_frames,
        init_func=init,
        interval=100,
        blit=True
    )

    writer = PillowWriter(fps=5)
    anim.save(save_path, writer=writer, dpi=150)
    plt.close(fig)
    print(f"GIF gespeichert als: {save_path}")


if __name__ == "__main__":
    # GIF für die erste Konfiguration erstellen
    animate_single_with_predictions(folder_index=1,
                                    save_path="N_p=5.gif")
    