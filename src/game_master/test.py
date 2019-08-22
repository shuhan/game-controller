import numpy as np

def getPoints():
    return np.array([320, 230]), np.array([320, 240])

def getError(pointOne, pointTwo):
    return pointOne - pointTwo

p1, p2 = getPoints()
e = getError(p1, p2)
sign = e/abs(e)

print(p1)
print(p2)
print(e)
print(sign)
print(np.linalg.norm(e))