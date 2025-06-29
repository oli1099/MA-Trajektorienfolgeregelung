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

        if solver_opts  is None:
            solver_opts = {"print_time":0}
        
        #Erstellen der Optimierungsvariablen
        self.zdim = (N+1)*nx +N*nu
        Z = ca.SX.sym('Z',self.zdim)

        # Anfangs- + referenz für jeden schritt
        P = ca.SX.sym('P',nx +nx*(N+1))
        x_0 = P[0:nx]
        
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
            x_ref_k = P[nx +k*nx : nx +(k+1)*nx] # zeitvariante Referenz
            
            cost += (xk-x_ref_k).T @ self.Q @ (xk - x_ref_k) + uk.T @ self.R @ uk
            g.append(x_next - (self.A_d @ xk + self.B_d @ uk))
        
        
        #Terminalkosten
        xN = Z[N*nx : (N+1)*nx]
        x_ref_N = P[nx + N*nx : nx +(N+1)*nx]
        cost += (xN - x_ref_N).T @ self.QN @ (xN - x_ref_N)

        #Nebenbedingungen

        g = ca.vertcat(*g) #Aufsplitten der Liste und als spaltenvektor deklarieren

        #Equality Constraints rechte Seite von  x_k+1 - Ad*x - Bd*u = 0
        self.lbg = np.zeros(g.size1())
        self.ubg = np.zeros(g.size1())
        
        #Inequality Constraints bestimmen für u < |1| und x muss die map dann sein

        #Standardgrenzen erstellen 
        lbz = -np.inf * np.ones(self.zdim) 
        ubz = np.inf*np.ones(self.zdim)

        #Zustandsbegrenzung

        for k in range(N +1):
            # X und Ywird begrenzt auf 0 bis 6
            lbz[k*self.nx + 0] = 0
            ubz[k*self.nx + 0] = 4       
            lbz[k*self.nx + 1] = -0.05
            ubz[k*self.nx + 1] = 0.5

        #Eingangsbegrenzung
        for k in range(N):
            lbz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = -5
            ubz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = 5
        
        self.lbz = np.array(lbz).flatten()
        self.ubz = ubz

        #Definition des quadratischen Problems 

        qp = {'f': cost, 'x':Z,'g': g, 'p':P} 
        self.solver = ca.qpsol('solver','qpoases',qp,solver_opts)

        #Warmstart immer der vorherige Zustand

    def solveMPC(self,x_current, x_ref,z0):
    
        P_val = []
        P_val.extend(x_current)  # dimension nx
        for k in range(self.N+1):
            P_val.extend(x_ref[:,k])  # dimension nx
        P_val = np.array(P_val)

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
        