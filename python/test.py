import numpy as np
from numba import int32, float32    # import the types
from numba.experimental import jitclass
from numba import jit


x = np.arange(100).reshape(10, 10)

@jit(nopython=True) # Set "nopython" mode for best performance, equivalent to @njit
def go_fast(a): # Function is compiled to machine code when called the first time
    trace = 0.0
    for i in range(a.shape[0]):   # Numba likes loops
        trace += np.tanh(a[i, i]) # Numba likes NumPy functions
    return a + trace              # Numba likes NumPy broadcasting

print(go_fast(x))

class classname(object):
    """
    
    """
    def __init__(self,value):
        """
        docstring
        """
        print(go_fast(value))

name = classname(x)
# spec = [
#     ('value', int32),               # a simple scalar field
#     ('array', float32[:]),          # an array field
# ]

# @jitclass(spec)
# class Bag(object):
#     def __init__(self, value):
#         self.value = value
#         self.array = np.zeros(value, dtype=np.float32)
#         go_fast(self.array)

#     @property
#     def size(self):
#         return self.array.size

#     def increment(self, val):
#         for i in range(self.size):
#             self.array[i] += val
#         return self.array

#     # @staticmethod
#     def add(x, y):
#         return x + y

# n = 21
# mybag = Bag(n)


# for j in range(self._roi_col_range[0], self._roi_col_range[1]):
#         cur_col_sum = 0
#         for i in range(self._roi_row_range[0], self._roi_row_range[1]):
#             if gray[i, j] > self._carrying_ignor_thre or self._pipet_img[i, j] == 255:
#                 continue
#             cur_col_sum += ((255 -gray[i, j]) >> 5)**3
#         cur_total_sum += cur_col_sum
#         if cur_col_sum != 0:
#             if last_col_sum == 0:
#                 cur_key = j
#         else:
#             if cur_total_sum != 0:
#                 col_intensity[cur_key] = cur_total_sum
#                 cur_total_sum = 0
#         last_col_sum = cur_col_sum
#         return True