from pytransform3d.plot_utils import plot_basis
from pytransform3d.transformations import plot_transform
import matplotlib.pyplot as plt

# Transformation matrices manually defined
T_c1_c2 = np.array([
    [0.9999946, 0.00035359, 0.00327479, 0.11991645],
    [-0.00032997, 0.99997395, -0.00721154, -0.000045],
    [-0.00327724, 0.00721042, 0.99996865, -0.00077719],
    [0, 0, 0, 1]
])

T_b_c1 = np.array([
    [-0.00191266, 0.00297417, 0.99999375, 0.0121479],
    [-0.99997317, -0.00707693, -0.00189157, 0.03453897],
    [0.00707126, -0.99997054, 0.00298762, -0.00107773],
    [0, 0, 0, 1]
])

T_b_c3 = np.array([
    [0.6925006, -0.04205233, 0.72019064, 0.03690092],
    [-0.7030963, 0.18421265, 0.6868197, 0.18756075],
    [-0.1615506, -0.9819864, 0.09800053, -0.01655837],
    [0, 0, 0, 1]
])

T_b_c4 = np.array([
    [-0.64722383, -0.05807829, 0.7600844, 0.03564082],
    [-0.74248224, -0.17784895, -0.6458249, -0.23938982],
    [0.17268862, -0.9823424, 0.07198599, -0.01404682],
    [0, 0, 0, 1]
])

# Set up a 3D plot using matplotlib
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-0.5, 0.5])
ax.set_ylim([-0.5, 0.5])
ax.set_zlim([-0.5, 0.5])

# Plot world frame
plot_basis(ax, label="World")

# Add transformations to the plot
plot_transform(ax, A2B=T_b_c1, s=0.1, name="Camera1 (Body)")
plot_transform(ax, A2B=T_b_c3, s=0.1, name="Camera3 (Body)")
plot_transform(ax, A2B=T_b_c4, s=0.1, name="Camera4 (Body)")
plot_transform(ax, A2B=T_c1_c2, s=0.1, name="Camera1 to Camera2")

# Set labels and title
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("3D Visualization of Transformations (pytransform3d)")

# Save the visualization
pytransform_manual_visualization_file = "/mnt/data/transformation_visualization_pytransform3d_manual.png"
plt.savefig(pytransform_manual_visualization_file)
plt.show()
