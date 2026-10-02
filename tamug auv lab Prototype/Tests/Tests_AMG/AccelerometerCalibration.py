import numpy as np
import time
import board
import busio
import adafruit_lsm9ds1
import matplotlib.pyplot as plt

fig = plt.figure()
plot3d = fig.add_subplot(111, projection='3d')
N = 200

# Step 1: Collect data
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(busio.I2C(board.SCL, board.SDA))
accel_data = np.zeros((N, 3))

print("Rotate the device slowly in all directions for ~20 seconds...")
for i in range(N):
    ax, ay, az = sensor.acceleration
    accel_data[i] = [ax, ay, az]
    time.sleep(0.1)
    print(f'{i}/{N}')

# Step 2: Normalize data to unit magnitude
#magnitudes = np.linalg.norm(accel_data, axis=1, keepdims=True)
#accel_data /= magnitudes

# Step 3: Fit ellipsoid
x, y, z = accel_data[:, 0], accel_data[:, 1], accel_data[:, 2]
D = np.column_stack([
    x**2, y**2, z**2,
    2*x*y, 2*x*z, 2*y*z,
    2*x, 2*y, 2*z,
    np.ones_like(x)
])
S = D.T @ D
_, _, V = np.linalg.svd(S)
params = V[-1]

# Step 4: Extract ellipsoid parameters
A, B, C, Dxy, Dxz, Dyz, Gx, Gy, Gz, J = params
M = np.array([
    [A, Dxy, Dxz],
    [Dxy, B, Dyz],
    [Dxz, Dyz, C]
])
v = np.array([Gx, Gy, Gz])
bias = -np.linalg.inv(M) @ v

# Step 5: Regularize and compute scale matrix
M += np.eye(3) * 1e-6  # regularization
eigvals, eigvecs = np.linalg.eigh(M)
eigvals = np.clip(eigvals, 1e-8, None)
scale_matrix = eigvecs @ np.diag(1.0 / np.sqrt(eigvals)) @ eigvecs.T

# Step 6: Save to CSV
with open("AccelerometerCalibration.csv", "w") as f:
    f.write("bias_x,bias_y,bias_z\n")
    f.write(f"{bias[0]},{bias[1]},{bias[2]}\n")
    f.write("scale_matrix_row_0\n")
    f.write(",".join(map(str, scale_matrix[0])) + "\n")
    f.write("scale_matrix_row_1\n")
    f.write(",".join(map(str, scale_matrix[1])) + "\n")
    f.write("scale_matrix_row_2\n")
    f.write(",".join(map(str, scale_matrix[2])) + "\n")

print("? Calibration complete.")
print("Bias vector:", bias)
print("Scale/misalignment matrix:\n", scale_matrix)
print("Saved to AccelerometerCalibration.csv")

plot3d.scatter(x, y, z, color='r')
plt.show()
#ax.scatter(DC[0], DC[1], DC[2], color='g')
