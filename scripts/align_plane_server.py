
from turtle import position
import geometry_msgs.msg
import numpy
import rospy
import math
import tf 

from align_plane_to_points.srv import *

def handle_align_plane(req):

    point1 = req.point1
    point2 = req.point2
    point3 = req.point3

    # calculate vector from pose 1 to 2 (and its magnitude)
    vector_1_to_2 = calculate_vector(point1, point2)
    normalized_vector_1_to_2 = normalize(vector_1_to_2)

    width = vector_magnitude(vector_1_to_2)

    # calculate vector from pose 1 to 3
    vector_1_to_3 = calculate_vector(point1, point3)

    # calculate perpendicular vector to both
    vector_perpendicular = cross_product(vector_1_to_2, vector_1_to_3)

    normalized_vector_perpendicular = normalize(vector_perpendicular)

    # caluclate the vector 1 to 3 with no 1 to 2 component
    vector_forward = cross_product(vector_perpendicular, vector_1_to_2)

    height = vector_magnitude(vector_forward) / (width ** 2)

    normalized_vector_forward = normalize(vector_forward)

    # calculate the rotation matrix

    R = calculate_rotation_matrix(
        normalized_vector_1_to_2, normalized_vector_forward, normalized_vector_perpendicular)

    #convert to quaternion
    T = numpy.matrix([[R.item(0, 0), R.item(0, 1), R.item(0, 2), 0],
                      [R.item(1, 0), R.item(1, 1), R.item(1, 2), 0],
                      [R.item(2, 0), R.item(2, 1), R.item(2, 2), 0],
                      [0, 0, 0, 1]])

    q = tf.transformations.quaternion_from_matrix(T)

    #send it back
    response = AlignPlaneResponse()

    response.page_height = height
    response.page_width = width
    response.alignedpose.position = req.point1

    response.alignedpose.orientation.x = q[0]
    response.alignedpose.orientation.y = q[1]
    response.alignedpose.orientation.z = q[2]
    response.alignedpose.orientation.w = q[3]
 
    return response

def calculate_vector(from_position, to_position):
    vector = geometry_msgs.msg.Vector3()

    vector.x = to_position.x - from_position.x
    vector.y = to_position.y - from_position.y
    vector.z = to_position.z - from_position.z

    return vector


def vector_magnitude(vector):

    return math.sqrt(vector.x**2 + vector.y**2 + vector.z**2)


def cross_product(u, v):
    cross = geometry_msgs.msg.Vector3()
    cross.x = u.y*v.z-v.y*u.z
    cross.y = v.x*u.z-u.x*v.z
    cross.z = u.x*v.y-u.y*v.x
    return cross


def normalize(input_vector):
    mag = (input_vector.x**2+input_vector.y**2+input_vector.z**2)**0.5
    unit_vector = geometry_msgs.msg.Vector3()
    unit_vector.x = input_vector.x/mag
    unit_vector.y = input_vector.y/mag
    unit_vector.z = input_vector.z/mag

    return unit_vector

# See https://math.stackexchange.com/questions/1125203/finding-rotation-axis-and-angle-to-align-two-3d-vector-bases?rq=1


def calculate_rotation_matrix(new_x_unit_vector, new_y_unit_vector, new_z_unit_vector):

    # vector basis for new orientation
    d = numpy.matrix(
        [new_x_unit_vector.x, new_x_unit_vector.y, new_x_unit_vector.z])
    e = numpy.matrix(
        [new_y_unit_vector.x, new_y_unit_vector.y, new_y_unit_vector.z])
    f = numpy.matrix(
        [new_z_unit_vector.x, new_z_unit_vector.y, new_z_unit_vector.z])

    # vector basis for default orientation
    a = numpy.matrix('1 0 0')
    b = numpy.matrix('0 1 0')
    c = numpy.matrix('0 0 1')

    # Dyads
    d_a = numpy.matrix([[d.item(0)*a.item(0), d.item(0)*a.item(1), d.item(0)*a.item(2)],
                        [d.item(1)*a.item(0), d.item(1) *
                         a.item(1), d.item(1)*a.item(2)],
                        [d.item(2)*a.item(0), d.item(2)*a.item(1), d.item(2)*a.item(2)]])

    e_b = numpy.matrix([[e.item(0)*b.item(0), e.item(0)*b.item(1), e.item(0)*b.item(2)],
                        [e.item(1)*b.item(0), e.item(1) *
                         b.item(1), e.item(1)*b.item(2)],
                        [e.item(2)*b.item(0), e.item(2)*b.item(1), e.item(2)*b.item(2)]])

    f_c = numpy.matrix([[f.item(0)*c.item(0), f.item(0)*c.item(1), f.item(0)*c.item(2)],
                        [f.item(1)*c.item(0), f.item(1) *
                         c.item(1), f.item(1)*c.item(2)],
                        [f.item(2)*c.item(0), f.item(2)*c.item(1), f.item(2)*c.item(2)]])

    # The rotation matrix corresponding with rotation from default orientation to the new one
    R = d_a + e_b + f_c

    return R


if __name__ == '__main__':
    rospy.init_node('align_plane_server')
    service = rospy.Service("align_plane", AlignPlane, handle_align_plane)
    rospy.spin()
