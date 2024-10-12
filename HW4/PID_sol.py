class PID:
    # constructor
    def __init__(self, Kp, Ki, Kd, N, Ts, umax, umin, Kt):
        self.istate = 0
        self.dstate = 0
        self.error_prev = 0   
        self.actuator_error_prev = 0  # holds previous value of (v-u)

        # set the PID parameters
        self.Kp, self.Ki, self.Kd, self.N, self.Kt = Kp, Ki, Kd, N, Kt
        self.Ts = Ts
        self.umin, self.umax = umin, umax


    def update(self, setpoint, measurement):
        error = setpoint - measurement

        # update the integral state, apply antiwindup as needed
        integrator_input = self.Ki*error + self.Kt*(self.actuator_error_prev)
        self.istate += self.Ts*integrator_input
        
        # update the derivative state
        self.dstate = 1/self.N*(error - self.error_prev - (self.Ts-self.N)*self.dstate)
        self.error_prev = error
 
        # Calculate command using PID equation
        command = self.Kp*error + self.istate + self.Kd*self.dstate

        # Saturate command
        if command > self.umax:
            output = self.umax
        elif command < self.umin:
            output = self.umin
        else:
            output = command

        # Store previous saturated command error for anti windup
        self.actuator_error_prev = output-command

        return output
