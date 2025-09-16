"""
=====================
Transformation Editor
=====================

The transformation editor can be used to manipulate transformations.
"""
from pytransform3d.transform_manager import TransformManager
from pytransform3d.editor import TransformEditor
from pytransform3d.transformations import transform_from
from pytransform3d.rotations import matrix_from_quaternion

import numpy as np

tm = TransformManager()

tm.add_transform(
    "zedx_left", "zedx_imu",
    transform_from(
        [[-0.00191266, 0.00297417, 0.99999375],
        [-0.99997317, -0.00707693, -0.00189157],
        [0.00707126, -0.99997054, 0.00298762]],
        [0.0121479, 0.03453897, -0.00107773]
    )
)

tm.add_transform(
    "zedx_right", "zedx_imu",
    transform_from(
        [[0.00136316, -0.00423679, 0.9999901],
        [-0.99997644, -0.00673315, 0.00133461],
        [0.00672743, -0.99996836, -0.00424587]],
        [0.01192727, -0.08537704, -0.00026116]
    )
)

# baseline T_1_0: 0 from 1 (left from right)
# for tm.add_transform(target, reference), the syntax is (0,1)
# 0: left
# 1: right
# q: [-0.00360552 -0.00163802 0.00017089 0.99999214] +- [0.00026055 0.00038025 0.00003525]
# t: [-0.11991837 0.0000082 0.00038414] +- [0.00002527 0.00002471 0.00007031]

tm.add_transform(
    "zedone_left", "zedx_left",
    transform_from(
        matrix_from_quaternion([-0.9219977, -0.01501683, 0.37441235, -0.09751987]),
        [-0.1531745, 0.01447088, 0.02441717]
    )
)
# baseline T_1_0:
# 1:zedone_left
# 0:zedx_left
# q: [-0.01501683 0.37441235 -0.09751987 0.9219977 ] +- [0.00054625 0.00059622 0.00022102]
# t: [-0.1531745 0.01447088 0.02441717] +- [0.0000695 0.00004896 0.00016089]

tm.add_transform(
    "zedone_right", "zedx_right",
    transform_from(
        matrix_from_quaternion([-0.93346243, -0.00447565, -0.3471676, 0.09001399]),
        [0.15394875, 0.01472174, 0.0235663]
    )
)
# baseline T_1_0:
# 1:zedone_right
# 0:zedx_right
# q: [-0.00447565 -0.3471676 0.09001399 0.93346243] +- [0.00056638 0.00068279 0.00028607]
# t: [0.15394875 0.01472174 0.0235663 ] +- [0.00007894 0.0000504 0.00022365]

# matrix_from_quaternion : (w, x, y, z)
tm.add_transform(
    "realsense", "zedx_right",
    transform_from(
        matrix_from_quaternion([0.00020612, -0.00351331, -0.12572679, -0.99205866]),
        [0.15557029, -0.19402682, -0.09975974]
    )
)
# q: [-0.00351331 -0.12572679 -0.99205866 0.00020612] +- [0.00029184 0.00045661 0.00009059]
# t: [ 0.15557029 -0.19402682 -0.09975974] +- [0.00008603 0.00008472 0.00021323]

tm.add_transform(
    "realsense", "xsense",
    transform_from(
        [[-0.00699928, -0.00169143, 0.99997407],
        [ 0.99997386, 0.00179997, 0.00700233],
        [-0.00181176, 0.99999695, 0.00167879]],
        [0.07524504, 0.00864593, 0.05117368]
    )
)

te = TransformEditor(tm, "xsense", s=0.1)

# Set print options and print in the specified format
np.set_printoptions(suppress=True, precision=8)

print("-----------------------")
a = "zedx_left"
b = "zedx_imu"
print(a, "to", b)
print(te.transform_manager.get_transform(a, b))

print("-----------------------")
a = "zedx_right"
b = "zedx_imu"
print(a, "to", b)
print(te.transform_manager.get_transform(a, b))

print("-----------------------")
a = "zedx_right"
b = "zedx_left"
print(a, "to", b)
print(te.transform_manager.get_transform(a, b))

te.show()
