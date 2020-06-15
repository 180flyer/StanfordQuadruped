import numpy as np
from transforms3d.euler import euler2mat


class SwingController:
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

    # def swing_height(self, swing_phase, triangular=True):
    #     if triangular:
    #         if swing_phase < 0.5:
    #             swing_height_ = swing_phase / 0.5 * self.config.z_clearance
    #         else:
    #             swing_height_ = self.config.z_clearance * (1 - (swing_phase - 0.5) / 0.5)
    #     return swing_height_


    def next_foot_location(
            self,
            swing_subphase_ticks,
            leg_index,
            state,
            command,
    ):
        l1 = self.config.LEG_L1
        l2 = self.config.LEG_L2
        touchdown_location = self.raibert_touchdown_location(leg_index, command)
        h = self.config.default_stance[0, leg_index]  # Center of ellipse
        rtl_x = touchdown_location[0]
        foot_up_angle = np.arcsin(-0.5)
        foot_down_angle = np.pi - foot_up_angle
        a = (h - rtl_x)/np.cos(foot_down_angle)
        b = 2.2 * ((l1 + l2 / 2.0) - np.sqrt(l1 * l1 + l2 * l2))
        k = command.height + b / 2.0  # improved ellipse center

        swing_angle_range = foot_down_angle - foot_up_angle
        swing_angle_per_tick = swing_angle_range / self.config.swing_ticks

        theta = (swing_subphase_ticks + 1) * swing_angle_per_tick + foot_up_angle
        new_x = h - a * np.cos(theta)  # Note the minus here to align with the positive x axis on the bot
        new_z = k + b * np.sin(theta)
        # print("l1, l2, b, k, fu, fd, sapt, theta: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f\r" % (
        #    l1, l2, b, k, foot_up_angle, foot_down_angle, swing_angle_per_tick, theta))
        # if leg_index == 0: print("h, rtl_x, a: %0.4f %0.4f %0.4f\r" % (h, rtl_x, a))

        swing_prop = swing_subphase_ticks / self.config.swing_ticks
        foot_location_y = state.foot_locations[1, leg_index]
        time_left = self.config.dt * self.config.swing_ticks * (1.0 - swing_prop)
        v = (touchdown_location[1] - foot_location_y) / time_left
        new_y = foot_location_y + v * self.config.dt
        return_vector = np.array([new_x, new_y, new_z])

        # if leg_index == 0: print(
        #     "leg: %d, subphase_tick: %d, return_vector: %0.4f, %0.4f, %0.4f\r" % (leg_index, swing_subphase_ticks, return_vector[0], return_vector[1], return_vector[2]))
        return return_vector
