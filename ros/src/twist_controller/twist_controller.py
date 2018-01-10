
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

KP_VEL = 0.4
KI_VEL = 0.3
KD_VEL = 0.3

KP_ANG = 0.1
KI_ANG = 0.2
KD_ANG = 0.3

class Controller(object):
    def __init__():
        self.pid_vel = PID(KP_VEL, KI_VEL, KD_VEL, -1.0, 1.0)
        self.pid_ang = PID(KP_ANG, KI_ANG, KD_ANG, -1.0, 1.0)
        pass

    def control(self, target_linear_v, target_angular_v, current_linear_v, dbw_status, sample_time):
        # TODO: Change the arg, kwarg list to suit your needs
        if dbw_status == True:
            self.pid_vel.reset()
            self.pid_ang.reset()
            return 0., 0., 0.

        error_linear_v = target_linear_v.x - current_linear_v.x
        value = self.pid_vel.step(error_linear_v, sample_time)
        # Return throttle, brake, steer
        if value > 0.0
            throttle = value

        error_angular_v = target_angular_v.z 

        return throttle, 0., 0.
