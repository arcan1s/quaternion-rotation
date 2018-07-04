import random

from src.quaternion_rotation import quaternion_rotation


source = None
product = None
quaternion = None


def coordinate_from_random():
    return random.random() * (-1) ** random.randint(0, 1)


def quaternion_from_random():
    return quaternion_rotation.Quaternion.from_vector(random.random(), vector_from_random()).norm()


def vector_from_random():
    return quaternion_rotation.Vector(coordinate_from_random(), coordinate_from_random(), coordinate_from_random())


def test_prepare():
    global source
    source = vector_from_random()
    global quaternion
    quaternion = quaternion_from_random()
    global product
    product = quaternion.rotate(source)


def test_rotation():
    # get another quaternion
    result = quaternion_rotation.Quaternion.from_rotation(source, product)
    # calcualte <>
    new_product = result.rotate(source)
    new_result = quaternion_rotation.Quaternion.from_rotation(source, new_product)
    assert str(result) == str(new_result)
