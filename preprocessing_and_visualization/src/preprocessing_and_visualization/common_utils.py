import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

def invert_transformation_matrix(transformation_matrix):
    transformation_matrix_inverse = np.eye(4)
    transformation_matrix_inverse[0:3, 0:3] = transformation_matrix[0:3, 0:3].T
    transformation_matrix_inverse[0:3, 3] = -transformation_matrix_inverse[0:3, 0:3].dot(transformation_matrix[0:3, 3])

    return transformation_matrix_inverse

def transformation_matrix_to_transform_stamped(transformation_matrix, frame_id_parent, frame_id_child, timestamp=None):
    message_transform_stamped = TransformStamped()

    if timestamp is not None:
        message_transform_stamped.header.stamp = timestamp
    message_transform_stamped.header.frame_id = frame_id_parent
    message_transform_stamped.child_frame_id = frame_id_child

    message_transform_stamped.transform.translation.x = transformation_matrix[0, 3]
    message_transform_stamped.transform.translation.y = transformation_matrix[1, 3]
    message_transform_stamped.transform.translation.z = transformation_matrix[2, 3]

    quaternion = Rotation.from_matrix(transformation_matrix[0:3, 0:3]).as_quat()
    message_transform_stamped.transform.rotation.x = quaternion[0]
    message_transform_stamped.transform.rotation.y = quaternion[1]
    message_transform_stamped.transform.rotation.z = quaternion[2]
    message_transform_stamped.transform.rotation.w = quaternion[3]

    return message_transform_stamped
