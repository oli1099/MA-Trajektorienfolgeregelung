import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter
from cycler import cycler

folders = [
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryMPC',
    '/home/oli/Desktop/Oliver/Uni/MA/NewData/MPCTrajectory_N=15_Q=100_T=0.1_T=23',
]

actual_path_file = 'mpc_data_actual_path.csv'
predictions_file = 'mpc_data_predictions.csv'

obstacle = {'obsXrl': 1.0, 'obsYrl': 0.25, 'obslength': 0.75}
safezone = 0.1

my_palette = ["#009D81", "#FFC857", "#9E9A00", "#ff627e", "#0057FF"]
plt.rcParams['axes.prop_cycle'] = cycler(color=my_palette)

def animate_with_trajectoryMPC(folder_index, save_path="combined_animation.gif"):
    ref_folder = folders[0]
    df_ref = pd.read_csv(os.path.join(ref_folder, actual_path_file))
    if 't' not in df_ref.columns:
        df_ref = df_ref.reset_index().rename(columns={'index': 't'})

    cfg_folder = folders[folder_index]
    df_act = pd.read_csv(os.path.join(cfg_folder, actual_path_file))
    df_pred = pd.read_csv(os.path.join(cfg_folder, predictions_file))

    for df in (df_act, df_pred):
        if 't' not in df.columns:
            df.reset_index(inplace=True)
            df.rename(columns={'index': 't'}, inplace=True)

    all_ts = sorted(df_act['t'].unique())
    n_frames = len(all_ts)

    fig, ax = plt.subplots(figsize=(7.29, 3))
    ax.set_xlabel(r'$x$ [m]')
    ax.set_ylabel(r'$y$ [m]')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

    all_x = pd.concat([df_ref['x'], df_act['x'], df_pred['x']])
    all_y = pd.concat([df_ref['y'], df_act['y'], df_pred['y']])
    ax.set_xlim(all_x.min(), all_x.max())
    ax.set_ylim(all_y.min(), all_y.max())

    ox, oy = obstacle['obsXrl'], 0.0
    ow, oh = obstacle['obslength'], obstacle['obsYrl']
    ax.add_patch(Rectangle((ox, oy), ow, oh, color='gray', alpha=0.5))
    ax.add_patch(Rectangle((ox-safezone, oy-safezone), ow+2*safezone, oh+2*safezone,
                           fill=False, ls='--', ec='red'))

    ax.plot(df_ref['x'], df_ref['y'], '-', lw=2, color=my_palette[3], label='Referenztrajektorie')

    line_act, = ax.plot([], [], '-', lw=2, color=my_palette[0], label='Ist-Pfad')
    line_pred, = ax.plot([], [], '--', lw=2, color=my_palette[1], label='Prädiktion')

    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.02), ncol=3, fontsize='small')

    def init():
        line_act.set_data([], [])
        line_pred.set_data([], [])
        return line_act, line_pred

    def update(i):
        t = all_ts[i]
        sub_act = df_act[df_act['t'] <= t]
        line_act.set_data(sub_act['x'], sub_act['y'])

        sub_pred = df_pred[df_pred['t'] == t]
        if not sub_pred.empty:
            line_pred.set_data(sub_pred['x'], sub_pred['y'])
        else:
            line_pred.set_data([], [])

        return line_act, line_pred

    anim = FuncAnimation(fig, update, frames=n_frames, init_func=init, interval=200, blit=True)
    writer = PillowWriter(fps=5)
    anim.save(save_path, writer=writer)
    plt.close(fig)
    print(f"GIF gespeichert als: {save_path}")

if __name__ == '__main__':
    animate_with_trajectoryMPC(folder_index=1)
