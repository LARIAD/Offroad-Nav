import numpy as np

class I_PID:
    def __init__(self, Kp, Ki, Kd, dt, order=2, alpha=1, W=10):

        self.order = order
        self.alpha = alpha
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.error = 0.0
        self.sigma = np.arange(W) * dt
        self.L = W * dt

        # Vecteurs FIFO
        self.consignes = np.zeros(W)
        self.mesures = np.zeros(W)
        self.controls = np.zeros(W)
        
        # Erreurs précédentes et cumulatives
        self.previous_error = 0.0
        self.integral = 0.0

        # use limits
        self.use_lim=False
        self.use_antiwindup=False
        
        #limite min max
        self.lim_max = 3.1415/2
        self.lim_min = -3.1415/2
        self.windupMax = 1.0

    def get_phi(self, y, u):
        
        if (self.order == 1):
            phi2int = (-6 / self.L**3) * ((self.L - 2 * self.sigma) * self.mesures + self.alpha * self.sigma * (self.L - self.sigma) * self.control)
            phi = np.trapz(phi2int) * self.dt
            return phi
        elif (self.order == 2):
            phi2int_0 = (60 / self.L**5) * (self.L**2 + 6 * self.sigma**2 - 6 * self.L * self.sigma) * self.mesures
            phi2int_1 = (-30 * self.alpha / self.L**5) * (self.L - self.sigma)**2 * self.sigma**2 * self.controls
            phi = np.trapz(phi2int_0) * self.dt + np.trapz(phi2int_1) * self.dt
            return phi

    def FIFO(self, x, val):
            x = np.roll(x, -1)
            x[-1] = val
            return x

    def update(self, consigne, mesure, control):
        
        self.error = consigne - mesure
        
        self.consignes = self.FIFO(self.consignes, consigne)
        self.mesures = self.FIFO(self.mesures, mesure)
        self.controls = self.FIFO(self.controls, control)
        
        P = self.Kp * self.error
        
        self.integral += self.error * self.dt
        I = self.Ki * self.integral

        if (self.use_antiwindup & (self.windupMax != 0)):
            if I > self.windupMax:
                I = self.windupMax
            elif I < -self.windupMax:
                I = -self.windupMax
        
        derivative = (self.error - self.previous_error) / self.dt
        D = self.Kd * derivative
        
        derivative_consigne1 = np.diff(self.consignes) / self.dt
        derivative_consigne2 = np.diff(self.consignes, n=2) / self.dt**2

        if (self.order == 1):
            u_corrected = -self.get_phi(self.mesures, self.controls) + derivative_consigne1[-1] + P + I + D
            u_corrected = u_corrected / self.alpha
            
        elif (self.order == 2):
            u_corrected = - self.get_phi(self.mesures, self.controls) + derivative_consigne2[-1] + P + I + D
            u_corrected = u_corrected / self.alpha
    
        output = u_corrected
        
        if self.use_lim:
            if output<self.lim_min:
                output=self.lim_min
            if output>self.lim_max:
                output=self.lim_max
            # Sauvegarder l'erreur actuelle pour le calcul futur du terme dérivé
            self.previous_error = self.error

        return output
    
    def update_w_param(self,W):
        self.sigma = np.arange(W) * self.dt
        self.L = W * self.dt

        # Vecteurs FIFO
        self.consignes = np.zeros(W)
        self.mesures = np.zeros(W)
        self.controls = np.zeros(W)
        
    def reset_error(self):
        self.error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
