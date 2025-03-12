

import numpy as np
import casadi as ca
from scipy.linalg import expm, block_diag
import matplotlib.pyplot as plt

class QP:
    def __init__(self, A_d, B_d, Q, R, QN, N, nx, nu, Ts,solver_opts=None):
        self.A_d = A_d
        self.B_d = B_d
        self.Q = Q
        self.R = R
        self.QN = QN
        self.N = N
        self.nx = nx
        self.nu = nu
        self.Ts = Ts


        #Hindernisse definieren (kreisförmig)

        self.x_obs = 1
        self.y_obs = 0.5
        self.r_obs = 0.5

        
    
        if solver_opts  is None:
            solver_opts = {"print_time":0}
        
        #Erstellen der Optimierungsvariablen
        self.zdim = (N+1)*nx +N*nu
        Z = ca.SX.sym('Z',self.zdim)

        # Anfangs- und Zielzustand P = [x0; x_ref]
        P = ca.SX.sym('P',nx*2)
        x_0 = P[0:nx]
        x_ref = P[nx:]

        #Inititalisieren der Kostenfunktion und der Anfangsbedingung
        cost = 0
        g = []

        #Anfangsbedingung setzen x_i,0 = x_current
        g.append(Z[0:nx]-x_0)

        #Kostenfunktion aufbauen
        for k in range(N):
            #extrahiere xk, x_k+1 ,u_k
            xk = Z[k*nx:(k+1)*nx]
            x_next = Z[(k+1)*nx:(k+2)*nx]
            uk = Z[(N+1)*nx +k*nu:(N+1)*nx + (k+1)*nu]
            
            cost += (xk-x_ref).T @ self.Q @ (xk - x_ref) + uk.T @ self.R @ uk
            g.append(x_next - (self.A_d @ xk + self.B_d @ uk))
        
        
        #Terminalkosten
        xN = Z[N*nx : (N+1)*nx]
        cost += (xN - x_ref).T @ self.QN @ (xN - x_ref)

        #Hindernis constraints hinzufügen (x-x_obs)² + (y-y_obs)² >= r²

        for k in range(N+1):
            xk = Z[k*nx:(k+1)*nx]
            obs = (xk[0]-self.x_obs)**2 + (xk[1]-self.y_obs)**2 - (self.r_obs)**2
            g.append(obs)
        
        #Nebenbedingungen

        g = ca.vertcat(*g) #Aufsplitten der Liste und als spaltenvektor deklarieren

        #Dimension der Equality constraints festlegen
        n_dynamics = (N+1)*self.nx
        n_obs = (N+1)

        lbg_dyn = np.zeros(n_dynamics)
        ubg_dyn = np.zeros(n_dynamics)

        lbg_obs = np.zeros(n_obs)
        ubg_obs = np.inf*np.ones(n_obs)

        self.lbg = np.concatenate((lbg_dyn, lbg_obs))
        self.ubg = np.concatenate((ubg_dyn, ubg_obs))

        #Equality Constraints rechte Seite von  x_k+1 - Ad*x - Bd*u = 0 und (x-x_obs)² + (y-y_obs)² - r² >= 0
        #self.lbg = np.zeros(g.size1())
        #self.ubg = np.zeros(g.size1())
        
        #Inequality Constraints bestimmen für u < |1| und x muss die map dann sein

        #Standardgrenzen erstellen 
        lbz = -np.inf * np.ones(self.zdim) 
        ubz = np.inf*np.ones(self.zdim)

        #Zustandsbegrenzung
        # evetl funktion sich xmin etc ziehen
       
        #Eingangsbegrenzung
        for k in range(N):
            lbz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = -10
            ubz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = 10
        
        self.lbz = np.array(lbz).flatten()
        self.ubz = np.array(ubz).flatten()

        #Definition des quadratischen Problems 

        nlp = {'f': cost, 'x':Z,'g': g, 'p':P} 
        self.solver = ca.nlpsol('solver','ipopt',nlp,solver_opts)

        #Warmstart immer der vorherige Zustand

        #self.z0 = np.zeros(self.zdim)

    def solveMPC(self,x_current, x_ref,z0):
        P_val = np.concatenate([x_current,x_ref])
        
        '''# 1) Kopien der globalen Bounds anlegen
        lbz_mod = self.lbz.copy()
        ubz_mod = self.ubz.copy() 
        # 2) Region per if-Abfrage bestimmen
        # Beispiel: Zwei Teilbereiche
        if x_current[0] < 2:
            x_min, x_max = 0.0, 5.0
            y_min, y_max = 0.0, 2.0
        elif x_current[0]  >= 2 and x_current[0] <3: 
            x_min, x_max = 2.0, 5.0
            y_min, y_max = 1, 2.0
        else:
            x_min, x_max = 3.0, 5.0
            y_min, y_max = 0, 2.0

        # 3) Für alle Zeitschritte k=0..N diese Bounds anwenden
        #    Annahme: x = Z[k*nx+0], y = Z[k*nx+1]
        for k in range(self.N+1):
            lbz_mod[k*self.nx + 0] = x_min
            ubz_mod[k*self.nx + 0] = x_max

            lbz_mod[k*self.nx + 1] = y_min
            ubz_mod[k*self.nx + 1] = y_max

        # 4) MPC mit den aktualisierten Bounds lösen
        P_val = np.concatenate([x_current, x_ref])
        sol = self.solver(
            x0 = z0,
            p = P_val,
            lbx = lbz_mod,
            ubx = ubz_mod,
            lbg = self.lbg,
            ubg = self.ubg
        )'''
        sol = self.solver(x0 = z0,p=P_val,lbx=self.lbz, ubx= self.ubz,lbg = self.lbg, ubg= self.ubg)
        z_opt =sol['x'].full().flatten()

        #Extrahiere X und U
        x_opt = np.zeros((self.nx,self.N+1))
        u_opt = np.zeros((self.nu,self.N))

        for k in range(self.N+1):
            x_opt[:,k] = z_opt[k*self.nx: (k+1)*self.nx]
        for k in range (self.N):
            u_opt[:,k] = z_opt[(self.N+1)*self.nx + k*self.nu:(self.N+1)*self.nx + (k+1)*self.nu]

        return x_opt, u_opt
    '''def getRegionBounds(x_current):
        # x_current[0] = x-Position
        if x_current[0] < 2:
            return (0,2, 0,2)
        elif x_current[0]  >= 2 and x_current[0] <3:
            return (2,3, 1,2)  # Beispiel: Schmaler Korridor
        else:
            return (3,5, 0,2)'''

        
'''if __name__ == "__main__":
    if __name__ == "__main__":
        nx = 3
        nu = 2
        N = 5
        Ts = 0.1
        A_d = np.eye(nx)
        B_d = 0.1 * np.ones((nx, nu))
        Q = np.eye(nx)
        R = 0.1 * np.eye(nu)
        QN = 2 * np.eye(nx)
        
        mpc = QP(A_d, B_d, Q, R, QN, N, nx, nu, Ts)
        x0 = np.array([0.0, 0.0, 0.0])
        x_ref = np.array([4.5,0, 0.0])  # Referenz, die hinter dem Hindernis liegt
        # Initialer Warmstart: Einfach Nullvektor (oder Propagation via A_d/B_d)
        z0 = np.zeros(mpc.zdim)
        
        x_opt, u_opt = mpc.solveMPC(x0, x_ref, z0)
        print("Optimale Zustände:\n", x_opt)
        print("Optimale Eingänge:\n", u_opt)
        # Plotten der Trajektorie:
        plt.plot(x_opt[0, :], x_opt[1, :], 'bo-')
        circle = plt.Circle((mpc.x_obs, mpc.y_obs), mpc.r_obs, color='r', fill=False, linestyle='--', label="Hindernis")
        plt.gca().add_artist(circle)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title("MPC-Trajektorie mit kreisförmigem Hindernis")
        plt.legend()
        plt.show()'''









