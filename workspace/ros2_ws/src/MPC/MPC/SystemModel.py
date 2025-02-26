import numpy as np
import casadi as ca
from scipy.linalg import expm
from scipy.signal import cont2discrete


class DynamicModel:
        def __init__(self,  m=1.2, #Masse des Roboters
                 I=0.0074385,      # Trägheitsmoment (um z Achse) in kgm²
                 lx=0.0735,       # Abstand in x-Richtung vom Schwerpunkt zu den Rädern (m)
                 ly=0.1005,       # Abstand in x-Richtung vom Schwerpunkt zu den Rädern (m)
                 r=0.0325,         # Radius der Räder
                 k=10.0,       # Verstärkungsfaktor (Umwandlung Geschwindigkeitsfehler -> Kraft)
                 Ts=0.1):          # Abtastzeit
                
            
            self.m = m
            self.I = I
            self.lx = lx
            self.ly = ly
            self.r = r
            self.k = k
            self.Ts = Ts
            
            # Kontinuierliches Modell aufstellen
            self.Ac, self.Bc , self.Cc, self.Dc = self.continuous_model()
            
            # Diskretisierung
            self.A_d, self.B_d = self.c2d(self.Ac, self.Bc,self.Cc, self.Dc, Ts)
            
            # Dimensions
            self.nx = self.Ac.shape[0]
            self.nu = self.Bc.shape[1]

        def continuous_model (self):
                  
                #Sytemmatrix
                Ac = np.array([
                        [0, 0, 0, 1,        0,      0],
                        [0, 0, 0, 0,        1,      0],
                        [0, 0, 0, 0,        0,      1],
                        [0, 0, 0, -2*self.k/self.m,   0,      0],
                        [0, 0, 0, 0,   -2*self.k/self.m, 2*self.k*self.ly/self.m],
                        [0, 0, 0, 0,   2*self.k*self.ly/self.I,        - 2*self.k*(np.square(self.lx)+np.square(self.ly))/self.I]])
                
                Bc = np.array([
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [self.k*self.r/(self.m*np.sqrt(2)), self.k*self.r/(self.m*np.sqrt(2)),  self.k*self.r/(self.m*np.sqrt(2)),  self.k*self.r/(self.m*np.sqrt(2))],
                        [self.k*self.r/(self.m*np.sqrt(2)), -self.k*self.r/(self.m*np.sqrt(2)), self.k*self.r/(self.m*np.sqrt(2)), -self.k*self.r/(self.m*np.sqrt(2))],
                        [(self.k*self.r*(self.lx-self.ly))/(self.I*np.sqrt(2)), (self.k*self.r*(self.ly-self.lx))/(self.I*np.sqrt(2)), 
                        (-self.k*self.r*(self.lx+self.ly))/(self.I*np.sqrt(2)), self.k*self.r/(self.I*np.sqrt(2))*(self.lx+self.ly)]
                    ])
                Cc = np.array([
                        [1, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0],
                        [0, 0, 1, 0, 0, 0]               
                    ])
                Dc = np.zeros((3,4))



                return Ac, Bc, Cc, Dc
            
        def c2d(self,Ac,Bc,Cc,Dc,Ts):
                 sys_d = cont2discrete((Ac,Bc,Cc,Dc),Ts,method='zoh')
                 Ad, Bd, Cd, Dd, dt = sys_d

                 return Ad, Bd

        def get_dimensions(self):
               return self.nx, self.nu