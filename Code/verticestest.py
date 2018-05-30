import numpy as np

def get_vertices(height,width):
    A = np.zeros((4, 2))
    B = np.zeros((4, 1))
    radius = [0.5*height, 0.5*width,
              0.5*height, 0.5*width]
    dth = 0.5*np.pi
    for l in range(4):
        A[l, :] = np.array([np.sin(l*dth), np.cos(l*dth)])
        B[l] = radius[l]
    vertices = np.zeros((2, 4))
    for l in range(4):
        a = np.vstack((A[l, :], A[(l+1) % 4, :]))
        b = np.vstack((B[l], B[(l+1) % 4]))
        vertices[:, l] = np.linalg.solve(a, b).ravel()
    return vertices

print(get_vertices(4,2))