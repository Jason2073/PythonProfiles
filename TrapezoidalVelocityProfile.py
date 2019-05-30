import math


class TrapezoidalVelocityProfile:
    desired_position = 0.0
    current_position = 0.0
    current_velocity = 0.0
    current_acceleration = 0.0
    max_velocity = 0.0
    max_acceleration = 0.0
    starting_Position = 0.0
    dt = 0.0
    time_to_accelerate = 0.0
    time_at_max_vel = 0.0
    total_time = 0.0
    current_time = 0.0
    relative_desired_position = 0.0
    point = []

    def __init__(self, starting_pos: float, desired_pos: float, max_vel: float, max_acc: float, dt: float):
        self.point
        self.starting_Position = starting_pos
        self.max_velocity = max_vel
        self.max_acceleration = max_acc
        self.dt = dt
        self.desired_position = desired_pos
        self.relative_desired_position = desired_pos - starting_pos
        self.time_to_accelerate = max_vel / max_acc
        dx_while_accelerating = .5 * self.time_to_accelerate * max_vel
        if starting_pos > desired_pos:
            dx_at_max_vel = self.relative_desired_position + 2 * dx_while_accelerating
        else:
            dx_at_max_vel = self.relative_desired_position - 2 * dx_while_accelerating

        self.time_at_max_vel = abs(dx_at_max_vel / max_vel)
        self.total_time = self.time_at_max_vel + 2 * self.time_to_accelerate
        self.current_position = starting_pos
        if dx_while_accelerating > .5 * abs(self.relative_desired_position):
            self.time_to_accelerate = math.sqrt(abs(self.relative_desired_position) / max_acc)
            self.time_at_max_vel = 0
            self.total_time = 2 * self.time_to_accelerate
        if starting_pos > desired_pos:
            self.max_acceleration = -max_acc
            self.max_velocity = -max_vel

    def calculate_next_point(self):
        if self.current_time < self.time_to_accelerate:
            self.current_acceleration = self.max_acceleration
            self.current_velocity += self.current_acceleration * self.dt
            if abs(self.current_velocity) > abs(self.max_velocity):
                self.current_velocity = self.max_velocity
                self.current_position += self.current_velocity * self.dt + \
                                         .5 * self.current_acceleration * pow(self.dt, 2)
        elif (self.current_time >= self.time_to_accelerate) and \
                (self.current_time <= self.total_time - self.time_to_accelerate):
            self.current_acceleration = 0
            self.current_velocity = self.max_velocity
            self.current_position += self.current_velocity * self.dt
        else:
            self.current_acceleration = -self.max_acceleration
            self.current_velocity += self.current_acceleration * self.dt
            self.current_position += self.current_velocity * self.dt + .5 * self.current_acceleration * pow(self.dt, 2)

        self.point = [self.current_position, self.current_velocity, self.current_acceleration, self.dt,
                      self.current_time]
        self.current_time += self.dt
        if self.current_time > self.total_time:
            self.current_position = self.desired_position
            self.current_velocity = 0
            self.current_acceleration = 0
            self.point = [self.desired_position, 0, 0, self.dt, self.current_time]
        return self.point

    def is_finished(self):
        return self.current_time > self.total_time or self.current_time > 30

    def position(self):
        return self.current_position

    def velocity(self):
        return self.current_velocity

    def acceleration(self):
        return self.current_acceleration
