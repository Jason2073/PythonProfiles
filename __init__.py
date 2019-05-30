import TrapezoidalVelocityProfile as Tmp
import PoofsProfile as p
import numpy
import csv
import fileinput

profile = Tmp.TrapezoidalVelocityProfile(0, 100, 20, 80, .01)
poof = p.SCurveProfile()
traj = numpy.empty(1070, dtype=p.SCurveProfile.Segment)
traj = poof.generate(p.SCurveProfile.Config(.01, 120, 400, 8000), 0, 48, 0)
data = ""
# for(seg in traj):
time = 0
for i in range(0, traj.size-1):
    data += str(time) + "," + str(traj[i]) + "\n"
    time += .01
    # data.append(str(traj[i].pos) + "," + str(traj[i].vel) + "," + str(traj[i].acc) + "," + str(traj[i].jerk))
# while not profile.is_finished():
#     print(profile.position())
#     data.append(
#         [profile.current_time, profile.current_position, profile.current_velocity, profile.current_acceleration])
#     profile.calculate_next_point()
with open('pythonProfiles.csv', 'w', newline='') as file:
    # for point in data:
        # line = str(point[0]) + "," + str(point[1]) + "," + str(point[2]) + "," + str(point[3]) + "\n"
    file.write(data)
    file.flush()
