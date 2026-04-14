class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        # Gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.dt = dt

        self.error = 0.0
        
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

    
    def update(self, consigne, mesure):
        # Calcul de l'erreur
        self.error = consigne - mesure
        
        # Terme proportionnel
        P = self.Kp * self.error

        # Terme intégral
        self.integral += self.error * self.dt
        I = self.Ki * self.integral

        if (self.use_antiwindup & (self.windupMax != 0)):
            if I > self.windupMax:
                I = self.windupMax
            elif I < -self.windupMax:
                I = -self.windupMax

        # Terme dérivé
        derivative = (self.error - self.previous_error) / self.dt
        D = self.Kd * derivative
        
        # Mise à jour de la sortie du PID
        output = consigne + P + I + D
        
        if self.use_lim:
            if output<self.lim_min:
                output=self.lim_min
            if output>self.lim_max:
                output=self.lim_max
            # Sauvegarder l'erreur actuelle pour le calcul futur du terme dérivé
            self.previous_error = self.error

        return output
    
    def reset_error(self):
        self.error = 0.0
        self.previous_error = 0.0
        self.integral=0.0
