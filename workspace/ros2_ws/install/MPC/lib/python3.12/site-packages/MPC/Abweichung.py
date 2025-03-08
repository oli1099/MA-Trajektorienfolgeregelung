import pandas as pd
import numpy as np

# --- 1) CSVs einlesen ---
# passe hier die Dateinamen an
file_soll = '/home/oli/Desktop/Oliver/Uni/MA/NewData/Trajectory/mpc_data_actual_path.csv'   # sollte Spalten x, y, yaw (optional) und t enthalten
file_ist  = '/home/oli/Desktop/Oliver/Uni/MA/NewData/TrajectoryTracking_L=0.1_V_ref=0.1/mpc_data_actual_path.csv'    # sollte Spalten x, y und t enthalten

df_soll = pd.read_csv(file_soll)
df_ist  = pd.read_csv(file_ist)

# --- 2) nach Zeit sortieren (wichtig für die Interpolation) ---
df_soll = df_soll.sort_values('t').reset_index(drop=True)
df_ist  = df_ist.sort_values('t').reset_index(drop=True)

# Die Ist-Zeiten
t_ist = df_ist['t'].values

# --- 3) Soll-Daten auf die Ist-Zeitpunkte interpolieren ---
# np.interp erwartet aufsteigende x-Werte (hier df_soll['t'])
x_soll_i = np.interp(t_ist, df_soll['t'], df_soll['x'])
y_soll_i = np.interp(t_ist, df_soll['t'], df_soll['y'])

# Falls ihr auch yaw vergleichen wollt und im Ist zwei yaw-Spalte habt,
# könnt ihr analog interpolieren:
# yaw_soll_i = np.interp(t_ist, df_soll['t'], df_soll['yaw'])
# yaw_ist    = df_ist['yaw'].values
# yaw_err    = np.abs(np.arctan2(np.sin(yaw_ist-yaw_soll_i), np.cos(yaw_ist-yaw_soll_i)))

# --- 4) Fehlervektoren und euklidischer Positionsfehler ---
dx = df_ist['x'].values - x_soll_i
dy = df_ist['y'].values - y_soll_i
pos_error = np.linalg.norm(np.vstack((dx, dy)), axis=0)

# --- 5) Kennzahlen berechnen ---
mae_pos  = np.max(pos_error)                 # Mean Absolute Error
rmse_pos = np.sqrt(np.mean(pos_error**2))     # Root Mean Square Error

print(f"MAE Position:  {mae_pos:.6f} [m]")
print(f"RMSE Position: {rmse_pos:.6f} [m]")

# --- 6) Fehler pro Zeitstempel als CSV ausgeben (optional) ---
df_err = pd.DataFrame({
    't':        t_ist,
    'err_pos':  pos_error,
    # 'err_yaw':  yaw_err,      # wenn yaw verwendet wird
})
df_err.to_csv('traj_errors.csv', index=False)
print("Einzelne Fehler wurden in 'traj_errors.csv' gespeichert.")
