import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter
from cycler import cycler

# Verzeichnisse
folders = [
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryMPC',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.05_V_ref=0.2',
]

# Dateinamen
actual_path_file = 'mpc_data_actual_path.csv'
reference_file = 'mpc_data_reference.csv'

# Lookahead-Distanz (m)
lookahead_dist = 0.05

# Obstacle-Parameter
obstacle = {
    'obsXrl': 1.0,      # x-Koordinate (links unten)
    'obsYrl': 0.25,     # y-Halbhöhe
    'obslength': 0.75   # Breite in x-Richtung
}
safezone = 0.1

# Farbenpalette
my_palette = [
    "#009D81", "#FFC857", "#9E9A00", "#ff627e",
    "#0057FF", "#709c5b", "#99D8CD", "#CCEBE6",
]
plt.rcParams['axes.prop_cycle'] = cycler(color=my_palette)


def prepare_reference(path):
    df_ref = pd.read_csv(os.path.join(path, reference_file))
    if 't' not in df_ref.columns:
        df_ref = df_ref.reset_index().rename(columns={'index': 't'})
    coords = df_ref[['x', 'y']].values
    deltas = coords[1:] - coords[:-1]
    dist = np.hypot(deltas[:, 0], deltas[:, 1])
    s = np.concatenate([[0], np.cumsum(dist)])
    df_ref['s'] = s
    return df_ref


def animate_with_trajectoryMPC(folder_index, save_path="combined_animation_lookahead_l=0.05.gif"):
    # Daten laden
    ref_folder = folders[0]
    df_ref = prepare_reference(ref_folder)

    cfg_folder = folders[folder_index]
    df_act = pd.read_csv(os.path.join(cfg_folder, actual_path_file))
    if 't' not in df_act.columns:
        df_act = df_act.reset_index().rename(columns={'index': 't'})

    all_ts = sorted(df_act['t'].unique())
    n_frames = len(all_ts)

    # Plot-Setup
    fig, ax = plt.subplots(figsize=(7.29, 3))
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    ax.plot(df_ref['x'], df_ref['y'], '-', lw=2, color=my_palette[3], label='Referenztrajektorie')
    all_points = np.vstack([df_ref[['x', 'y']].values, df_act[['x', 'y']].values])
    ax.set_xlim(all_points[:, 0].min(), all_points[:, 0].max())
    ax.set_ylim(all_points[:, 1].min(), 0.5)  # y-Achse bis 0.5 begrenzen

    line_act, = ax.plot([], [], '-', lw=2, color=my_palette[0], label='Ist-Pfad')
    look_pt, = ax.plot([], [], 'o', ms=6, color=my_palette[1], label=f'Lookahead {lookahead_dist} m')

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


    # Legende oberhalb des Plots platzieren
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=3, fontsize='small')

    def init():
        line_act.set_data([], [])
        look_pt.set_data([], [])
        return line_act, look_pt

    def update(i):
        t = all_ts[i]
        row_act = df_act[df_act['t'] == t].iloc[0]
        x0, y0 = row_act['x'], row_act['y']
        sub_act = df_act[df_act['t'] <= t]
        line_act.set_data(sub_act['x'], sub_act['y'])

        dists = np.hypot(df_ref['x'] - x0, df_ref['y'] - y0)
        idx0 = dists.argmin()
        s0 = df_ref.at[idx0, 's']

        s_target = s0 + lookahead_dist
        s_array = df_ref['s'].values
        idx_l = np.searchsorted(s_array, s_target, side='left')
        idx_l = min(idx_l, len(s_array) - 1)

        x_l = df_ref.iloc[idx_l]['x']
        y_l = df_ref.iloc[idx_l]['y']
        look_pt.set_data([x_l], [y_l])

        # Keine Überschrift
        return line_act, look_pt

    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init,
                         interval=200, blit=True)
    writer = PillowWriter(fps=5)
    anim.save(save_path, writer=writer)
    plt.close(fig)
    print(f"GIF gespeichert als: {save_path}")


def main():
    animate_with_trajectoryMPC(folder_index=1)


if __name__ == '__main__':
    main()
