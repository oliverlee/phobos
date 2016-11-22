#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Run this in ipython so showing the plot will not be closed after script
# terminates

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# Set event list, polynomial order, and tcf here.
# Events and tcf should be obtained from gdb where events is m_events and tcf is
# the penultimate element in m_T.
events = sorted([
#(2098183326, 999),
#(2098704741, 998),
#(2099377778, 999),
#(2106592372, 998),
#(2107462033, 997),
#(2108285010, 998),
#(2110745533, 997),
#(2112368672, 998),
#(2114811569, 999),
#(2114813308, 998)

#(3708851300, -2548),
#(3710531300, -2548),
#(3712211300, -2548),
#(3713891296, -2548),
#(3715571296, -2548),
#(3717251296, -2548),
#(3718931296, -2548),
#(3703811300, -2548),
#(3705491300, -2548),
#(3707171300, -2548)
#tcf = 1.87497497

#(1728730706, -571),
#(1730405339, -571),
#(1732085339, -571),
#(1733765339, -571),
#(1735462197, -571),
#(1728056289, -572),
#(1728132647, -571),
#(1728228985, -572),
#(1728405447, -571),
#(1728728142, -572),

(3137651615, -977),
(3138747156, -978),
(3138839909, -977),
(3138894043, -976),
(3140084071, -977),
(3140115043, -976),
(3141010425, -977),
(3141051449, -976),
(3141412563, -977),
(3141438081, -976),
(3142775311, -977),
(3142795483, -976),
(3142948805, -977),
(3143013620, -976),
(3134294914, -977),
(3135971615, -977),

    ])
tcf = 1.25
polyorder = 2
##############################################################################

def timestamp_matrix(polynomial_order, time_vector):
    A = np.ones((len(time_vector), polynomial_order + 1))
    A[:, -2] = np.array(time_vector)
    for i in reversed(range(0, A.shape[1] - 2)):
        A[:, i] = A[:, i + 1] * A[:, -2]
    return A

print("encoder events: {0}".format(len(events)))
print("polynomial order {0}:\n".format(polyorder))

t, x = zip(*events)
t = np.array(t)
x = np.array(x)

t = t - t[0] # redefine zero time
alpha = 1/(t[-1] - t[0])
t = alpha*t # scale time values
print("alpha = {0}".format(alpha))

A = timestamp_matrix(polyorder, t)
B = np.array(x)
print("time stamp matrix A =\n{0}".format(A))
print("position vector B = {0}".format(B))

P = np.linalg.lstsq(A, B)[0]
print("polynomial coefficients P = {0}".format(P))

#tc = tcf/alpha + events[0][0]
#print("tc = {0}".format(tc))
T = np.ones(polyorder + 1)
for i in reversed(range(0, T.shape[0] - 1)):
    T[i] = tcf * T[i + 1]

print("time vector T = {0}".format(T))
print("position = {0}".format(np.dot(P, T)))

# get non-ugly colors
colors = sns.color_palette("Paired")
sns.set_style('darkgrid')

fig, ax = plt.subplots(2, sharex=True)
# print encoder events
ax[0].plot(t, x, 'o', color=colors[5])
x0, x1, y0, y1 = plt.axis()

# plot best fit line
t_fit = np.linspace(0, tcf, 100)
p_fit = np.poly1d(P)
pd_fit = np.polyder(p_fit)
y_fit = p_fit(t_fit)
ax[0].plot(t_fit, y_fit, color=colors[4], linewidth=3)
ax[1].plot(t_fit, pd_fit(t_fit), color=colors[4], linewidth=3)

# plot estimated position
ycf = y_fit[-1]
ecf = ycf - x[-1]
if abs(ecf) > 1:
    ycf = x[-1] + np.sign(ecf)
ax[0].plot(tcf, ycf, 'o', color=colors[3])

# recalculate polynomial with estimated position
t2 = np.append(t, tcf)
x2 = np.append(x, ycf)
A2 = timestamp_matrix(polyorder, t2)
B2 = np.array(x2)
P2 = np.linalg.lstsq(A2, B2)[0]
p2_fit = np.poly1d(P2)
p2d_fit = np.polyder(p2_fit)

ax[0].plot(t_fit, p2_fit(t_fit), color=colors[2])
ax[1].plot(t_fit, p2d_fit(t_fit), color=colors[2])

# fit axes
x0 = 0
x1 = tcf
y1 = max(max(y_fit), max(x))
y0 = min(min(y_fit), min(x))
x_plot_margin = 0.1 * (x1 - x0)
y_plot_margin = max(0.1 * (y1 - y0), 0.1)
ax[0].set_xlim([x0 - x_plot_margin, x1 + x_plot_margin])
ax[0].set_ylim([y0 - y_plot_margin, y1 + y_plot_margin])

ax[0].legend(['encoder events', 'best fit line',
              'estimated position', 'recalculated best fit line'],
             loc=0)
ax[0].set_ylabel('encoder position')
ax[1].set_xlabel('normalized time')
ax[1].set_ylabel('encoder velocity')
fig.suptitle('higher-order encoder time-stamping estimate, polynomial order'
        ' {0}\n{1}, {2}, {3}'.format(polyorder, ycf, pd_fit[-1], p2d_fit[-1]))

ax[0].ticklabel_format(useOffset=False, style='plain')
ax[1].ticklabel_format(useOffset=False, style='plain')
plt.show(block=False)
