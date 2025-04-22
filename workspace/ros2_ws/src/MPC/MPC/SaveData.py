import numpy as np
import pandas as pd
import argparse

class SaveData:
    def __init__(self,
                 predictions_list=None,  # Liste von np.ndarray (2×N)
                 actual_path=None,       # Liste von (x,y)-Tuples
                 actual_u=None):         # Liste von [u1,u2,u3,u4]
        self.predictions_list = predictions_list or []
        self.actual_path = actual_path or []
        self.actual_u = actual_u or []

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

    def save_all(self, basename):
        """Alle drei Dateien auf Basis von `basename` erzeugen."""
        self.save_predictions(basename)
        self.save_actual_path(basename)
        self.save_control_inputs(basename)
