from enum import Enum
import math
import numpy


class SCurveProfile:
    class Strategy(Enum):
        S_CURVE_STRATEGY = 1
        TRAPEZOIDAL_STRATEGY = 2
        STEP_STRATEGY = 3
        AUTO_STRATEGY = 4

    class Segment:
        pos, vel, acc, jerk, dt = 0, 0, 0, 0, 0

        def __str__(self):
            return str(self.pos) + "," + str(self.vel) + "," + str(self.acc) + "," + str(self.jerk)

        def __init__(self, pos=0, vel=0, acc=0, jerk=0, dt=0):
            self.pos = pos
            self.vel = vel
            self.acc = acc
            self.jerk = jerk
            self.dt = dt

    class Config:
        dt = 0
        max_vel = 0
        max_acc = 0
        max_jerk = 0

        def __init__(self, dt, max_vel, max_acc, max_jerk):
            self.dt = dt
            self.max_vel = max_vel
            self.max_acc = max_acc
            self.max_jerk = max_jerk

    def choose_strategy(self, start_vel, goal_vel, max_vel):
        if start_vel == goal_vel and start_vel == max_vel:
            strategy = self.Strategy.STEP_STRATEGY
        elif start_vel == goal_vel and start_vel == 0:
            strategy = self.Strategy.S_CURVE_STRATEGY
        else:
            strategy = self.Strategy.TRAPEZOIDAL_STRATEGY
        return strategy

    def generate(self, config: Config, start_vel: float, goal_pos: float, goal_vel: float):
        strategy = self.choose_strategy(start_vel, goal_vel, config.max_vel)
        traj = []
        if strategy is self.Strategy.STEP_STRATEGY:
            impulse = (goal_pos / config.max_vel) / config.dt
            time = int(math.floor(impulse))
            traj = self.second_order_filter(1, 1, config.dt, config.max_vel, config.max_vel, impulse, time,
                                            self.IntegrationMethod.TRAPEZOIDAL)
        elif strategy is self.Strategy.TRAPEZOIDAL_STRATEGY:
            start_discount = .5 * pow(start_vel, 2)/config.max_acc
            end_discount = .5 * pow(goal_vel, 2)/config.max_acc
            adjusted_max_vel = min(config.max_vel, math.sqrt(config.max_acc * goal_pos - start_discount - end_discount))
            t_ramp_up = (adjusted_max_vel - start_vel) / config.max_acc
            x_ramp_up = start_vel * t_ramp_up + .5*config.max_acc * pow(t_ramp_up, 2)
            t_ramp_down = (adjusted_max_vel - goal_vel)/ config.max_acc
            x_ramp_down = adjusted_max_vel * t_ramp_down - .5 * config.max_acc* pow(t_ramp_down, 2)
            x_cruise = goal_pos - x_ramp_down - x_ramp_up
            time = int((t_ramp_up + t_ramp_down + x_cruise/adjusted_max_vel)/config.dt + .5)
            f1_length = int(math.ceil((adjusted_max_vel / config.max_acc) / config.dt))
            impulse = (goal_pos / adjusted_max_vel)/config.dt - start_vel / config.max_acc / config.dt \
                + start_discount + end_discount
            traj = self.second_order_filter(f1_length, 1, config.dt, start_vel, adjusted_max_vel, impulse, time,
                                            self.IntegrationMethod.TRAPEZOIDAL)
        elif strategy is self.Strategy.S_CURVE_STRATEGY:
            adjusted_max_vel = min(config.max_vel, (-config.max_acc * config.max_acc +
                                   math.sqrt(pow(config.max_acc, 4) + 4 * pow(config.max_jerk, 2)
                                             * config.max_acc * goal_pos)) / (2*config.max_jerk))
            f1_length = int(math.ceil((adjusted_max_vel / config.max_acc) / config.dt))
            f2_length = int(math.ceil((config.max_acc/config.max_jerk)/config.dt))
            impulse = (goal_pos / adjusted_max_vel) / config.dt
            time = int(math.ceil(f1_length + f2_length + impulse))
            traj = self.second_order_filter(f1_length, f2_length, config.dt, 0, adjusted_max_vel,
                                            impulse, time, self.IntegrationMethod.TRAPEZOIDAL)
        else:
            return None

        return traj

    class IntegrationMethod(Enum):
        RECTANGULAR = 0
        TRAPEZOIDAL = 1

    def second_order_filter(self, f1_length, f2_length, dt, start_vel, max_vel, total_impulse, length,
                            integration_method: IntegrationMethod):
        if length < 0:
            return None

        traj = numpy.empty(length + 1, dtype=self.Config)
        last = self.Segment(0, start_vel, 0, 0, dt)
        f1 = numpy.empty(length + 1)
        f1[0] = ((start_vel / max_vel) * f1_length)
        f2 = 0
        for i in range(0, length):
            traj[i] = self.Segment()
            input = min(total_impulse, 1)
            if input < 1:
                input -= 1
                total_impulse = 0
            else:
                total_impulse -= input
            f1_last = 0
            if i > 0:
                f1_last = f1[i - 1]
            else:
                f1_last = f1[0]
            f1[i] = max(0.0, min(f1_length, f1_last + input))

            f2 = 0
            for j in range(0, f2_length):
                if i - j < 0:
                    break
                f2 += f1[i - j]
            f2 = f2 / f1_length
            traj[i].vel = f2/f2_length * max_vel
            if integration_method is self.IntegrationMethod.RECTANGULAR:
                traj[i].pos = traj[i].vel * dt + last.pos
            elif integration_method is self.IntegrationMethod.TRAPEZOIDAL:
                # print(traj[i].vel)
                traj[i].pos = (last.vel + traj[i].vel) / 2.0 * dt + last.pos
            traj[i].acc = (traj[i].vel - last.vel)/dt
            traj[i].jerk = (traj[i].acc - last.acc)/dt
            traj[i].dt = dt
            last = traj[i]
        return traj


# 			} else if (integration == TrapezoidalIntegration) {
# 				traj.segments_[i].pos = (last.vel
# 						+ traj.segments_[i].vel) / 2.0 * dt + last.pos;
# 			}
# 			traj.segments_[i].x = traj.segments_[i].pos;
# 			traj.segments_[i].y = 0;
#
# 			// Acceleration and jerk are the differences in velocity and
# 			// acceleration, respectively.
# 			traj.segments_[i].acc = (traj.segments_[i].vel - last.vel) / dt;
# 			traj.segments_[i].jerk = (traj.segments_[i].acc - last.acc) / dt;
# 			traj.segments_[i].dt = dt;
#
# 			last = traj.segments_[i];
# 		}
#
# 		return traj;
# 	}
