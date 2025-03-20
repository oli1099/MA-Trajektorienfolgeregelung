

import numpy as np
import casadi as ca
from scipy.linalg import expm, block_diag
import matplotlib.pyplot as plt

class QP:
    def __init__(self, A_d, B_d, Q, R, QN,Penalty, N, nx, nu, Ts,solver_opts=None):
        self.A_d = A_d
        self.B_d = B_d
        self.Q = Q
        self.R = R
        self.QN = QN
        self.Penalty = Penalty
        self.N = N
        self.nx = nx
        self.nu = nu
        self.Ts = Ts

        #H = 0.5*block_diag([Q]*N+[QN],[R]*N)
    
        if solver_opts  is None:
            solver_opts = {"print_time":0}
        
        #Erstellen der Optimierungsvariablen (x0,x1,...,xN,u0,u1,...,uN und slack)
        self.zdim = (N+1)*nx +N*nu + self.N+1
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
            slack = Z[(N+1)*nx + N*nu + k]
                  
            cost += (xk-x_ref).T @ self.Q @ (xk - x_ref) + uk.T @ self.R @ uk + slack.T @ self.Penalty @ slack
            g.append(x_next - (self.A_d @ xk + self.B_d @ uk))
            
            
        #Terminalkosten
        xN = Z[N*nx : (N+1)*nx]
        cost += (xN - x_ref).T @ self.QN @ (xN - x_ref)

        default_ymin =0
        #Soft Constraints (ymin -s-y<=0)
        for k in range(N+1):
            xk = Z[k*nx:(k+1)*nx]
            slack = Z[(N+1)*nx + N*nu + k] #Slack Variable
            g.append(default_ymin - xk[1] - slack) #ymin - y - s <= 0

    
        #Nebenbedingungen

        g = ca.vertcat(*g) #Aufsplitten der Liste und als spaltenvektor deklarieren

        #Für die Equality Constrains g == 0 und für die Inequality Constraints g <= 0
        n_dynamics = (N+1)*self.nx
        n_slack = N+1

        lbg_dyn = np.zeros(n_dynamics)
        ubg_dyn = np.zeros(n_dynamics)

        lbg_slack = -np.inf*np.ones(n_slack)
        ubg_slack = np.zeros(n_slack)

        self.lbg = np.concatenate((lbg_dyn, lbg_slack))
        self.ubg = np.concatenate((ubg_dyn, ubg_slack))



        #Equality Constraints rechte Seite von  x_k+1 - Ad*x - Bd*u = 0
        #self.lbg = np.zeros(g.size1())
        #self.ubg = np.zeros(g.size1())
        
        #Inequality Constraints bestimmen für u < |1| und x muss die map dann sein

        #Standardgrenzen erstellen 
        lbz = -np.inf * np.ones(self.zdim) 
        ubz = np.inf*np.ones(self.zdim)

        #Zustandsbegrenzung
        # evetl funktion sich xmin etc ziehen

        for k in range(N +1):
            # X wird begrenzt auf 0 bis 6
            lbz[k*self.nx + 0] = 0
            ubz[k*self.nx + 0] = 6            
       
        #Eingangsbegrenzung
        for k in range(N):
            lbz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = -10
            ubz[(N+1)*nx+k*nu:(N+1)*nx +(k+1)*nu] = 10
        #Slack Variable Begrenzung ( darf nicht negativ werden)
        for k in range(N+1):
            lbz[(N+1)*nx + N*nu + k] = 0
            ubz[(N+1)*nx + N*nu + k] = 1e20
            # Obere schranke bleibt unendlich
        
        
        self.lbz = np.array(lbz).flatten()
        self.ubz = np.array(ubz).flatten()

        #Definition des quadratischen Problems 

        qp = {'f': cost, 'x':Z,'g': g, 'p':P} 
        self.solver = ca.qpsol('solver','qpoases',qp,solver_opts)

        #Warmstart immer der vorherige Zustand

        #self.z0 = np.zeros(self.zdim)

    def solveMPC(self,x_current, x_ref,z0):
        P_val = np.concatenate([x_current,x_ref])
        
        lbz_mod = self.lbz.copy()
        ubz_mod = self.ubz.copy()

        

        for k in range(self.N+1):
        # Der x-Wert des Warmstarts  für Zeitschritt k
            x_pred = z0[k*self.nx + 0]
            
            # Standard: Straßenbegrenzung 
            y_min = 0.0
            y_max = 4.0
            
            # Wenn der prädizierte x-Wert im Hindernisbereich liegt, setze y_min auf einen sicheren Wert
            '''if  x_pred >= 1 and x_pred <= 2.0:
                y_min = 1.1 

            else:
                y_min = 0.0
            
            lbz_mod[k*self.nx + 1] = y_min
            ubz_mod[k*self.nx + 1] = y_max

            print(f"Zeitschritt {k}: x in [{lbz_mod[k*self.nx + 0]}, {ubz_mod[k*self.nx + 0]}], "
            f"y in [{lbz_mod[k*self.nx + 1]}, {ubz_mod[k*self.nx + 1]}]")
            print(f"Prädizierter x-Wert: {x_pred}")'''

        sol = self.solver(x0 = z0,p=P_val,lbx=lbz_mod, ubx= ubz_mod,lbg = self.lbg, ubg= self.ubg)    
        
        #sol = self.solver(x0 = z0,p=P_val,lbx=self.lbz, ubx= self.ubz,lbg = self.lbg, ubg= self.ubg)
        z_opt =sol['x'].full().flatten()

        #Extrahiere X und U
        x_opt = np.zeros((self.nx,self.N+1))
        u_opt = np.zeros((self.nu,self.N))
        slack_opt = np.zeros(self.N+1)

        for k in range(self.N+1):
            x_opt[:,k] = z_opt[k*self.nx: (k+1)*self.nx]
            slack_opt[k] = z_opt[(self.N+1)*self.nx + self.N*self.nu + k]
        for k in range (self.N):
            u_opt[:,k] = z_opt[(self.N+1)*self.nx + k*self.nu:(self.N+1)*self.nx + (k+1)*self.nu]

        return x_opt, u_opt , slack_opt
   
        
'''if __name__ == "__main__":
    # KLEINER TEST

    # Beispielsystem
    nx = 3
    nu = 2
    N  = 5
    Ts = 0.1
    
    A_d = np.eye(nx)
    B_d = np.ones((nx, nu))*0.1
    
    Q  = np.eye(nx)*1.0
    R  = np.eye(nu)*0.1
    QN = np.eye(nx)*2.0
    
    for i in range(N):
        mpc = QP(A_d, B_d, Q, R, QN, N, nx, nu, Ts)
    
        x0 = np.array([0.0, 0.0, 0.0])
        x_ref = np.array([2.0, 3.0, 0.0])
    
        U_opt, X_opt = mpc.solveMPC(x0,x_ref, mpc.z0)
        print("U_opt:\n", U_opt)
        print("X_opt:\n", X_opt)'''










