import numpy as np

# a = np.array([[1,2],[3,4]])
# b = np.array([[5,6],[7,8]])
# print(np.vstack((a,b)))
# print(a.shape)
# a = np.array([[1,2,3,4],[2,5,7,6],[7,4,5,6]])
a = np.array([[],[],[]])
b = np.hstack((a,np.array([2,8,7])))
print(b)
# b = np.hstack((a, np.array([[2],[8]])))
# print(a.shape)
# print(a)
# a = a[np.newaxis,:]
# print(a.shape)
# print(a)
# print(a[0][2])
# a[0][2]=10
# print(a[0][2])
# map_grid = np.full((50, 50), int(0), dtype=np.int8)
# print(map_grid)