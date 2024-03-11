import scripts.main as dubins
import math

# minimum turning radius
R = 1.0

# initial position
xi = 0.0
yi = 0.0
qi = (xi, yi)

# final position
xf = 5.0
yf = 0.0
qf = (xf, yf)

# initial heading interval
al = math.pi/6
ah = math.pi/5
a = (al, ah)

# final heading interval
bl = math.pi/2
bh = math.pi
b = (bl, bh)

# solve dubins interval problem
interval = dubins.DubinsInterval(R, qi, qf, a, b)
interval.results()
interval.draw_path()