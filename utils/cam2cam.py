import numpy as np
from scipy.spatial.transform import Rotation as R

# Transformation from base_link_gt to left_cam
left_translation = np.array([0.0, 0.05, 0.0])
left_rotation = R.from_quat([0.5, -0.5, 0.5, -0.5]).as_matrix()
tf_left_cam = np.eye(4)
tf_left_cam[:3, :3] = left_rotation
tf_left_cam[:3, 3] = left_translation

# Transformation from base_link_gt to right_cam
right_translation = np.array([0.0, -0.05, 0.0])
right_rotation = R.from_quat([0.5, -0.5, 0.5, -0.5]).as_matrix()
tf_right_cam = np.eye(4)
tf_right_cam[:3, :3] = right_rotation
tf_right_cam[:3, 3] = right_translation

# l_T_b
tf_left_cam_inv = np.linalg.inv(tf_left_cam)

# l_T_r = l_T_b * b_T_r
relative_tf = np.matmul(tf_left_cam_inv,tf_right_cam)

print("Relative transformation matrix:")
print(relative_tf)

# Relative transformation matrix:
# [[1.  0.  0.  0.1]
#  [0.  1.  0.  0. ]
#  [0.  0.  1.  0. ]
#  [0.  0.  0.  1. ]]

