import numpy as np
from transforms3d.euler import euler2mat

class StanceController:
    def __init__(self, config):
        self.config = config

    def raibert_touchdown_location(
            self, leg_index, command
    ):
        delta_p_2d = (
                self.config.alpha
                * self.config.stance_ticks
                * self.config.dt
                * command.horizontal_velocity
        )
        delta_p = np.array([delta_p_2d[0], delta_p_2d[1], 0])
        theta = (
                self.config.beta
                * self.config.stance_ticks
                * self.config.dt
                * command.yaw_rate
        )
        R = euler2mat(0, 0, theta)
        return R @ self.config.default_stance[:, leg_index] + delta_p

    def foot_up_point(
            self, leg_index, command
    ):
        fup = (
                (self.config.alpha - 1.0)
                * self.config.stance_ticks
                * self.config.dt
                * command.horizontal_velocity[0]
                + self.config.default_stance[0, leg_index]
        )

        return fup

    def position_delta(self, leg_index, state, command):
        """Calculate the difference between the next desired body location and the current body location
        
        Parameters
        ----------
        z_measured : float
            Z coordinate of the feet relative to the body.
        stance_params : StanceParams
            Stance parameters object.
        movement_reference : MovementReference
            Movement reference object.
        gait_params : GaitParams
            Gait parameters object.

        Returns
        -------
        (Numpy array (3), Numpy array (3, 3))
            (Position increment, rotation matrix increment)
        """
        z = state.foot_locations[2, leg_index]
        v_xy = np.array(
            [
                -command.horizontal_velocity[0],
                -command.horizontal_velocity[1],
                1.0
                / self.config.z_time_constant
                * (state.height - z),
            ]
        )
        delta_p = v_xy * self.config.dt
        delta_R = euler2mat(0, 0, -command.yaw_rate * self.config.dt)
        return delta_p, delta_R

    # TODO: put current foot location into state
    def next_foot_location(self, leg_index, state, command):
        foot_location = state.foot_locations[:, leg_index]
        (delta_p, delta_R) = self.position_delta(leg_index, state, command)
        incremented_location = delta_R @ foot_location + delta_p

        # Override Z
        stance_x = incremented_location[0]
        if stance_x < self.foot_up_point(leg_index, command):
            z = command.height
        else:
            l1 = self.config.LEG_L1
            l2 = self.config.LEG_L2
            touchdown_location = self.raibert_touchdown_location(leg_index, command)
            h = self.config.default_stance[0, leg_index]  # Center of ellipse
            rtl_x = touchdown_location[0]
            foot_up_angle = np.arcsin(-0.5)
            foot_down_angle = np.pi - foot_up_angle
            a = (rtl_x - h)/np.cos(foot_down_angle)
            b = 2.8 * ((l1 + l2 / 2.0) - np.sqrt(l1 * l1 + l2 * l2))
            k = command.height + b / 2.0  # improved ellipse center

            if a == 0:
                theta = 3.0 * np.pi / 2.0
            else:
                theta = np.pi + np.arccos((h - stance_x)/a)

            z = k + b * np.sin(theta)

        incremented_location[2] = z
        # if leg_index == 0: print("stance_x, l1, l2, rtl_x, a, b, k, fu, fd, theta, z: %0.4f, %0.4f, %0.4f, %0.4f,%0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\r" % (
        #     stance_x, l1, l2, rtl_x, a, b, k, foot_up_angle, foot_down_angle, theta, z))

        return incremented_location
