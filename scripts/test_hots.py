import numpy as np


events = [
    (1162704874, -5547),
    (1179727586, -5548),
    (1209562198, -5547),
    (1224960594, -5548),
    ]

t, x = zip(*events)
t = np.array(t)
x = np.array(x)

t = t - t[0] # redefine zero time
alpha = 1/t[-1]
t = alpha*t # scale time values

A = np.ones((4, 4))
A[:, -2] = np.array(t)
for i in reversed(range(0, A.shape[1] - 2)):
    A[:, i] = A[:, i + 1] * A[:, -2]
B = np.array(x)
print(A)
print(B)

P = np.linalg.lstsq(A, B)[0]
print(P)

tc = alpha*(events[-1][0] + 1000)
print(tc)
T = np.ones(4)
for i in reversed(range(0, T.shape[0] - 1)):
    T[i] = tc * T[i + 1]

print(T)
print(np.dot(P, T))
