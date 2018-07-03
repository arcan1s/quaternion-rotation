#from __future__ import annotations
import math


#simple vector representation
class Vector(object):
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y, self.z + other.z)

    def __len__(self):
        return math.sqrt(self.scalar_multiplication(self))

    def __mul__(self, num: float):
        return Vector(self.x * num, self.y * num, self.z * num)

    def __neg__(self):
        return self.__mul__(-1)

    def __repr__(self) -> str:
        return "Vector({}, {}, {})".format(self.x, self.y, self.z)

    def __sub__(self, other):
        return self.__add__(-other)

    def __truediv__(self, num: float):
        return self.__mul__(1.0 / num)

    def scalar_multiplication(self, other) -> float:
        return (self.x * other.x + self.y * other.y + self.z * other.z)

    def vector_multiplication(self, other):
        return Vector(self.y * other.z - self.z * other.y,
                      self.z * other.x - self.x * other.z ,
                      self.x * other.y - self.y * other.x)


# quaternion representation
class Quaternion(object):
    def __init__(self, w: float, x: float, y: float, z: float):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Quaternion(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)

    def __len__(self) -> float:
        return math.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __mul__(self, num: float):
        return Quaternion(self.w * num, self.x * num, self.y * num, self.z * num)

    def __neg__(self):
        return self.__mul__(-1)

    def __repr__(self) -> str:
        return "Quaternion({}, {}, {}, {})".format(self.w, self.x, self.y, self.z)

    def __sub__(self, other):
        return self.__add__(-other)

    def __truediv__(self, num: float):
        return self.__mul__(1.0 / num)

    @property
    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    @property
    def inverse(self):
        return self.conjugate / self.scalar_multiplication(self)

    @property
    def vector(self):
        return Vector(self.x, self.y, self.z)

    def norm(self):
        return self / self.__len__()

    def quaternion_multiplication(self, other):
        return Quaternion(self.scalar_multiplication(other),
                          other.w * self.x + other.x * self.w - other.y * self.z + other.z * self.y,
                          other.w * self.y + other.x * self.z + other.y * self.w - other.z * self.x,
                          other.w * self.z - other.x * self.y + other.y * self.x + other.z * self.w)

    def rotate(self, vector):
        # q * s * q-1 = q * s * q' = (w + v) (0 + s) (w - v)
        return vector * (self.w ** 2 - self.vector.scalar_multiplication(self.vector)) + \
            self.vector * vector.scalar_multiplication(self.vector) * 2 + \
            self.vector.vector_multiplication(vector) * self.w * 2

    def scalar_multiplication(self, other):
        return self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
