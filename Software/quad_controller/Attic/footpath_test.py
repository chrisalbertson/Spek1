import footpath

fp = footpath.Footpath()
rot_rate = 3.14159 / 8.0

fp.set_velocity(0.4, 0.2, rot_rate)

foot = fp.xyz(0.05)
print(foot)
foot = fp.xyz(0.95)
print(foot)
