import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# 1. Figure out where this script lives:
script_dir = os.path.dirname(os.path.abspath(__file__))
print("Script directory:", script_dir)

# 2. Build the absolute path to your CSV
csv_path = os.path.join(script_dir, '..', 'homework', 'hw3', 'q1.csv')
csv_path = os.path.normpath(csv_path)   # clean up any “..” or extra slashes
print("Looking for CSV at:", csv_path)

# 3. (Optional) verify it exists before you try to read it:
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"CSV not found at {csv_path}")

# 4. Load it!
df = pd.read_csv(csv_path)
print(df['x'])

# plt.figure()
plt.plot(df['t'], df['norm'], label='velocity')
# plt.plot(df['t'], df['ydot'], label='y velocity')
# plt.plot(df['t'], df['zdot'], label='z velocity')
# plt.plot(df['t'], df['q2'], label='Joint 3')
# plt.plot(df['t'], df['q3'], label='Joint 4')
# plt.plot(df['t'], df['q4'], label='Joint 5')
# plt.plot(df['t'], df['q5'], label='Joint 6')
# plt.plot(df['t'], df['q6'], label='Joint 7')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.title('End effector velocity')

plt.axhline(0.1,    color='blue', linestyle='--', linewidth=1, label='Part b Vmax')
plt.axhline(-0.1,    color='blue', linestyle='--', linewidth=1, label='Part b Vmin')
# plt.axhline(-170*np.pi/180,color='blue', linestyle='--', linewidth=1, label='y = -170 deg')

# plt.axhline(210*np.pi/180, color='orange', linestyle='--', linewidth=1, label='y = 210 deg')
# plt.axhline(0, color='orange', linestyle='--', linewidth=1, label='y = 0 deg')

plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(df['t'], df['x'], label='x')
plt.plot(df['t'], df['y'], label='y')
plt.plot(df['t'], df['z'], label='z')
plt.plot(df['t'], df['xd'], label='xd', color='r')
plt.plot(df['t'], df['yd'], label='yd', color='r')
plt.plot(df['t'], df['zd'], label='zd', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('End effector trajectory')

# plt.axhline(0.6,    color='blue', linestyle='--', linewidth=1, label='x = 0.6')
# plt.axhline(0.3,    color='blue', linestyle='--', linewidth=1, label='y = 0.3')
# plt.axhline(0.5,    color='blue', linestyle='--', linewidth=1, label='z = 0.5')

# print(df['t'][1])
# plt.figure()
# plt.plot(df['t'], df['t1'], label='t1')
# plt.plot(df['t'], df['t2'], label='t2')
# plt.plot(df['t'], df['t3'], label='t3')
# plt.plot(df['t'], df['t4'], label='t4')
# plt.plot(df['t'], df['t5'], label='t5')
# plt.plot(df['t'], df['t6'], label='t6')
# plt.plot(df['t'], df['t7'], label='t6')
# plt.xlabel('Time (s)')
# plt.ylabel('Torque')
# plt.title('Null Space Control Torques - Field')

plt.grid(True)
plt.legend()
plt.show()
