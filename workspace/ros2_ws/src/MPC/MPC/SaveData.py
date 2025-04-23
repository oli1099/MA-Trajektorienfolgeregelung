import numpy as np
import pandas as pd

class SaveData:
    def __init__(self,
                 predictions_list=None,    # Liste von np.ndarray (nx × Np+1)
                 actual_path=None,         # Liste von (x,y)-Tuples
                 actual_u=None,            # Liste von [u1,u2,u3,u4]
                 actual_theta=None,        # Liste von θ-Werten (gemessener Yaw)
                 predicted_theta_list=None # Liste von np.ndarray (1 × Np+1) für prädizierte θ
                ):
        self.predictions_list = predictions_list or []
        self.actual_path = actual_path or []
        self.actual_u = actual_u or []
        self.actual_theta = actual_theta or []
        self.predicted_theta_list = predicted_theta_list or []

    def save_predictions(self, basename):
        """Schreibt <basename>_predictions.csv"""
        rows = []
        for traj_id, pred in enumerate(self.predictions_list):
            xs, ys = pred[0, :], pred[1, :]
            for x, y in zip(xs, ys):
                rows.append({'trajectory_id': traj_id, 'x': float(x), 'y': float(y)})
        fname = f"{basename}_predictions.csv"
        pd.DataFrame(rows).to_csv(fname, index=False)
        print(f"-> Predictions: {fname}")

    def save_actual_path(self, basename):
        """Schreibt <basename>_actual_path.csv"""
        df = pd.DataFrame(self.actual_path, columns=['x', 'y'])
        fname = f"{basename}_actual_path.csv"
        df.to_csv(fname, index=False)
        print(f"-> Actual path: {fname}")

    def save_control_inputs(self, basename):
        """Schreibt <basename>_control_inputs.csv"""
        df = pd.DataFrame(self.actual_u, columns=['u1', 'u2', 'u3', 'u4'])
        df.index.name = 'time_step'
        fname = f"{basename}_control_inputs.csv"
        df.to_csv(fname)
        print(f"-> Control inputs: {fname}")

    def save_actual_theta(self, basename):
        """Schreibt <basename>_actual_theta.csv" 
        """
        df = pd.DataFrame({'theta': self.actual_theta})
        df.index.name = 'time_step'
        fname = f"{basename}_actual_theta.csv"
        df.to_csv(fname)
        print(f"-> Actual theta: {fname}")

    def save_predicted_theta(self, basename):
        """Schreibt <basename>_predicted_theta.csv"""
        rows = []
        for traj_id, theta_pred in enumerate(self.predicted_theta_list):
            for t_idx, theta in enumerate(theta_pred):
                rows.append({'trajectory_id': traj_id, 'time_step': t_idx, 'theta': float(theta)})
        fname = f"{basename}_predicted_theta.csv"
        pd.DataFrame(rows).to_csv(fname, index=False)
        print(f"-> Predicted theta: {fname}")

    def save_all(self, basename):
        """Alle Dateien auf Basis von `basename` erzeugen."""
        self.save_predictions(basename)
        self.save_actual_path(basename)
        self.save_control_inputs(basename)
        self.save_actual_theta(basename)
        self.save_predicted_theta(basename)
