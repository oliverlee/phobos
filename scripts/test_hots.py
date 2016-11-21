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
events = [
(2098183326, 999),
(2098704741, 998),
(2099377778, 999),
(2106592372, 998),
(2107462033, 997),
(2108285010, 998),
(2110745533, 997),
(2112368672, 998),
(2114811569, 999),
(2114813308, 998)
    ]
tcf = 1.87497497
polyorder = 3

##############################################################################

eventlength = len(events)
print("encoder events: {0}".format(eventlength))
print("polynomial order {0}:\n".format(polyorder))

t, x = zip(*events)
t = np.array(t)
x = np.array(x)

t = t - t[0] # redefine zero time
alpha = 1/(t[-1] - t[0])
t = alpha*t # scale time values
print("alpha = {0}".format(alpha))

A = np.ones((eventlength, polyorder))
A[:, -2] = np.array(t)
for i in reversed(range(0, A.shape[1] - 2)):
    A[:, i] = A[:, i + 1] * A[:, -2]
B = np.array(x)
print("time stamp matrix A =\n{0}".format(A))
print("position vector B = {0}".format(B))

P = np.linalg.lstsq(A, B)[0]
print("polynomial coefficients P = {0}".format(P))

#tc = tcf/alpha + events[0][0]
#print("tc = {0}".format(tc))
T = np.ones(polyorder)
for i in reversed(range(0, T.shape[0] - 1)):
    T[i] = tcf * T[i + 1]

print("time vector T = {0}".format(T))
print("position = {0}".format(np.dot(P, T)))

# get non-ugly colors
colors = sns.color_palette("Paired")
sns.set_style('darkgrid')

fig, ax = plt.subplots()
# print encoder events
ax.plot(t, x, 'o', color=colors[5])
x0, x1, y0, y1 = plt.axis()

# plot best fit line
t_fit = np.linspace(0, tcf, 100)
p_fit = np.poly1d(P)
y_fit = p_fit(t_fit)
ax.plot(t_fit, y_fit, color=colors[4])

# plot estimated position
ax.plot(tcf, y_fit[-1], 'o', color=colors[3])

# fit axes
x1 = tcf
y1 = max(y1, max(y_fit))
y0 = min(y0, min(y_fit))
x_plot_margin = 0.1 * (x1 - x0)
y_plot_margin = 0.1 * (y1 - y0)
plt.axis((x0 - x_plot_margin,
          x1 + x_plot_margin,
          y0 - y_plot_margin,
          y1 + y_plot_margin))

plt.legend(['encoder events', 'best fit line', 'estimated position'], loc=0)
ax.set_xlabel('normalized time')
ax.set_ylabel('encoder position')
fig.suptitle('higher-order encoder time-stamping estimate')
plt.show(block=False)
