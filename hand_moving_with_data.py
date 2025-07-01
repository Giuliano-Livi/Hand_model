import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from functions_for_model import from_q_to_rotation, print_axis, pos_line, pos_point, angle_between_vectors, get_htm_mp, from_final_pos_to_joints
import math
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time
import rosbag

# Inizializza le liste
hand_positions = []
hand_orientations = []
marker_positions = [[] for _ in range(5)]  # 5 marker
last_marker_positions = [None] * 5  # Per tracciare l'ultima posizione valida

# Carica il file .bag
bag = rosbag.Bag('bagfiles/max_extention.bag')

for topic, msg, t in bag.read_messages():
    if topic == "/tf":
        for transform in msg.transforms:
            if transform.child_frame_id == "hand":
                pos = transform.transform.translation
                rot = transform.transform.rotation
                hand_positions.append([pos.x, pos.y, pos.z])
                hand_orientations.append([rot.x, rot.y, rot.z, rot.w])

    elif topic == "/vicon/unlabeled_markers":
        current_marker_positions = [None] * 5
        for i, marker in enumerate(msg.markers):
            if i < 5:
                pos = marker.pose.position
                current_marker_positions[i] = [pos.x, pos.y, pos.z]
                last_marker_positions[i] = current_marker_positions[i]  # aggiorna ultima posizione valida

        # Per ogni marker: se il valore corrente è None, usa il precedente
        for i in range(5):
            if current_marker_positions[i] is None:
                if last_marker_positions[i] is not None:
                    marker_positions[i].append(last_marker_positions[i])
                else:
                    marker_positions[i].append([np.nan, np.nan, np.nan])  # se nessuna posizione disponibile
            else:
                marker_positions[i].append(current_marker_positions[i])
bag.close()


# Salvataggio in file .npy
np.save("npy_files/hand_positions.npy", np.array(hand_positions))
np.save("npy_files/hand_orientations.npy", np.array(hand_orientations))
for i in range(5):
    np.save(f"npy_files/marker_{i}_positions.npy", np.array(marker_positions[i]))


# Carica i file .npy
pos_hand = np.load("npy_files/hand_positions.npy").tolist()
rot_hand = np.load("npy_files/hand_orientations.npy").tolist()
pos_f1 = np.load("npy_files/marker_0_positions.npy").tolist()
pos_f2 = np.load("npy_files/marker_1_positions.npy").tolist()
pos_f3 = np.load("npy_files/marker_2_positions.npy").tolist()
pos_f4 = np.load("npy_files/marker_3_positions.npy").tolist()
pos_f5 = np.load("npy_files/marker_4_positions.npy").tolist()


min_length = min(len(pos_hand), len(pos_f1), len(pos_f2), len(pos_f3), len(pos_f4), len(pos_f5))

plt.ion()

# Set up the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_zlim(-0.2, 0.2)

# Definition of the points and lines of the hand structure
#name     = ax.plot([x],[y],[z], 'color/shape', size)
origin, = ax.plot([], [], [], 'bo', markersize=5)
point_h1, = ax.plot([], [], [], 'bo', markersize=5)
point_h2, = ax.plot([], [], [], 'bo', markersize=5)
point_h3, = ax.plot([], [], [], 'bo', markersize=5)
point_h4, = ax.plot([], [], [], 'bo', markersize=5)
point_h5, = ax.plot([], [], [], 'bo', markersize=5)
point_h6, = ax.plot([], [], [], 'bo', markersize=5)
line_h1, = ax.plot([], [], [], 'k-', linewidth=2)
line_h2, = ax.plot([], [], [], 'k-', linewidth=2)
line_h3, = ax.plot([], [], [], 'k-', linewidth=2)
line_h4, = ax.plot([], [], [], 'k-', linewidth=2)
line_h5, = ax.plot([], [], [], 'k-', linewidth=2)
line_h6, = ax.plot([], [], [], 'k-', linewidth=2)

# Definition of the points and the lines of the fingers
marker_f1, = ax.plot([], [], [], 'go', markersize=5)
marker_f2, = ax.plot([], [], [], 'mo', markersize=5)
marker_f3, = ax.plot([], [], [], 'co', markersize=5)
marker_f4, = ax.plot([], [], [], 'ro', markersize=5)
marker_f5, = ax.plot([], [], [], 'yo', markersize=5)
line_f11, = ax.plot([], [], [], 'k-', linewidth=2)
line_f12, = ax.plot([], [], [], 'k-', linewidth=2)
line_f13, = ax.plot([], [], [], 'k-', linewidth=2)
line_f21, = ax.plot([], [], [], 'k-', linewidth=2)
line_f22, = ax.plot([], [], [], 'k-', linewidth=2)
line_f23, = ax.plot([], [], [], 'k-', linewidth=2)
line_f31, = ax.plot([], [], [], 'k-', linewidth=2)
line_f32, = ax.plot([], [], [], 'k-', linewidth=2)
line_f33, = ax.plot([], [], [], 'k-', linewidth=2)
line_f41, = ax.plot([], [], [], 'k-', linewidth=2)
line_f42, = ax.plot([], [], [], 'k-', linewidth=2)
line_f43, = ax.plot([], [], [], 'k-', linewidth=2)
line_f51, = ax.plot([], [], [], 'k-', linewidth=2)
line_f52, = ax.plot([], [], [], 'k-', linewidth=2)
line_f53, = ax.plot([], [], [], 'k-', linewidth=2)

joint1p_f1, = ax.plot([], [], [], 'go', markersize=5)
joint2p_f1, = ax.plot([], [], [], 'go', markersize=5)
joint1p_f2, = ax.plot([], [], [], 'yo', markersize=5)
joint2p_f2, = ax.plot([], [], [], 'yo', markersize=5)
joint1p_f3, = ax.plot([], [], [], 'mo', markersize=5)
joint2p_f3, = ax.plot([], [], [], 'mo', markersize=5)
joint1p_f4, = ax.plot([], [], [], 'ro', markersize=5)
joint2p_f4, = ax.plot([], [], [], 'ro', markersize=5)
joint1p_f5, = ax.plot([], [], [], 'co', markersize=5)
joint2p_f5, = ax.plot([], [], [], 'co', markersize=5)

# Definition of the length of the finger joints
f1_length = 0.11
f1_l1, f1_l2, f1_l3 = f1_length*0.5, f1_length*0.34, f1_length*0.26
f2_l1, f2_l2, f2_l3 = 0.067, 0.057, 0.042
f3_l1, f3_l2, f3_l3 = 0.073, 0.06, 0.046
f4_l1, f4_l2, f4_l3 = 0.067, 0.063, 0.04
f5_l1, f5_l2, f5_l3 = 0.06, 0.073, 0.055

phi_1 = -20
phi_2 = -40
phi_3 = -40
phi_4 = -60
phi_5 = -65

threshold = -1

f1_rel_prev = np.array([0.15954817, -0.04850752, -0.01941226, 1])
f2_rel_prev = np.array([0.14718147, 0.12169835, -0.04258014, 1])
f3_rel_prev = np.array([0.02415149, 0.13794686, -0.09176365, 1])
f4_rel_prev = np.array([0.18140122, 0.06914709, -0.01804172, 1])
f5_rel_prev = np.array([1.82499501e-01, 9.96591903e-04, -1.92990169e-02, 1])
m_nulla = np.array([[1,0,0],[0,1,0],[0,0,1]])


for i in range(1):
    pos_point(origin, 0, 0, 0)
    pos_point(point_h1, -0.025, 0.05, -0.025)
    pos_point(point_h2, -0.025, -0.035, -0.025)
    pos_point(point_h3, 0.05, -0.035, -0.025)
    pos_point(point_h4, 0.052, -0.01, -0.025)
    pos_point(point_h5, 0.055, 0.02, -0.025)
    pos_point(point_h6, 0.052, 0.05, -0.025)
    pos_line(line_h1, point_h1, point_h2)
    pos_line(line_h2, point_h2, point_h3)
    pos_line(line_h3, point_h3, point_h4)
    pos_line(line_h4, point_h4, point_h5)
    pos_line(line_h5, point_h5, point_h6)
    pos_line(line_h6, point_h6, point_h1)

    # Update finger marker positions
    pos_point(marker_f1, f1_rel_prev[0], f1_rel_prev[1], f1_rel_prev[2])
    pos_point(marker_f2, f2_rel_prev[0], f2_rel_prev[1], f2_rel_prev[2])
    pos_point(marker_f3, f3_rel_prev[0], f3_rel_prev[1], f3_rel_prev[2])
    pos_point(marker_f4, f4_rel_prev[0], f4_rel_prev[1], f4_rel_prev[2])
    pos_point(marker_f5, f5_rel_prev[0], f5_rel_prev[1], f5_rel_prev[2])


    # Creation of the joint model finger 1
    target_1 = np.array([f1_rel_prev[0], f1_rel_prev[1], -0.01])
    T1 = get_htm_mp(point_h3, target_1)

    joint1_f1, joint2_f1 = from_final_pos_to_joints(T1, f1_rel_prev, f1_l1, f1_l2, f1_l3, phi_1)
    pos_point(joint1p_f1, joint1_f1[0], joint1_f1[1], joint1_f1[2])
    pos_point(joint2p_f1, joint2_f1[0], joint2_f1[1], joint2_f1[2])
    pos_line(line_f11, point_h3, joint1p_f1)
    pos_line(line_f12, joint1p_f1, joint2p_f1)
    pos_line(line_f13, joint2p_f1, marker_f1)

    # Creation of the joint model finger 2
    target_2 = np.array([f5_rel_prev[0], f5_rel_prev[1], -0.01])
    T2 = get_htm_mp(point_h4, target_2)
    joint1_f2, joint2_f2 = from_final_pos_to_joints(T2, f5_rel_prev, f2_l1, f2_l2, f2_l3, phi_2)
    pos_point(joint1p_f2, joint1_f2[0], joint1_f2[1], joint1_f2[2])
    pos_point(joint2p_f2, joint2_f2[0], joint2_f2[1], joint2_f2[2])
    pos_line(line_f21, point_h4, joint1p_f2)
    pos_line(line_f22, joint1p_f2, joint2p_f2)
    pos_line(line_f23, joint2p_f2, marker_f5)

    # Creation of the joint model finger 3
    target_3 = np.array([f4_rel_prev[0], f4_rel_prev[1], -0.01])
    T3 = get_htm_mp(point_h5, target_3)
    joint1_f3, joint2_f3 = from_final_pos_to_joints(T3, f4_rel_prev, f3_l1, f3_l2, f3_l3, phi_3)
    pos_point(joint1p_f3, joint1_f3[0], joint1_f3[1], joint1_f3[2])
    pos_point(joint2p_f3, joint2_f3[0], joint2_f3[1], joint2_f3[2])
    pos_line(line_f31, point_h5, joint1p_f3)
    pos_line(line_f32, joint1p_f3, joint2p_f3)
    pos_line(line_f33, joint2p_f3, marker_f4)

    # Creation of the joint model finger 4
    target_4 = np.array([f2_rel_prev[0], f2_rel_prev[1], -0.01])
    T4 = get_htm_mp(point_h6, target_4)
    joint1_f4, joint2_f4 = from_final_pos_to_joints(T4, f2_rel_prev, f4_l1, f4_l2, f4_l3, phi_4)
    pos_point(joint1p_f4, joint1_f4[0], joint1_f4[1], joint1_f4[2])
    pos_point(joint2p_f4, joint2_f4[0], joint2_f4[1], joint2_f4[2])
    pos_line(line_f41, point_h6, joint1p_f4)
    pos_line(line_f42, joint1p_f4, joint2p_f4)
    pos_line(line_f43, joint2p_f4, marker_f2)

    # Creation of the joint model finger 5
    target_5 = np.array([f3_rel_prev[0], f3_rel_prev[1], -0.01])
    T5 = get_htm_mp(point_h1, target_5)
    joint1_f5, joint2_f5 = from_final_pos_to_joints(T5, f3_rel_prev, f5_l1, f5_l2, f5_l3, phi_5)
    pos_point(joint1p_f5, joint1_f5[0], joint1_f5[1], joint1_f5[2])
    pos_point(joint2p_f5, joint2_f5[0], joint2_f5[1], joint2_f5[2])
    pos_line(line_f51, point_h1, joint1p_f5)
    pos_line(line_f52, joint1p_f5, joint2p_f5)
    pos_line(line_f53, joint2p_f5, marker_f3)
    
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.05)


# Animation loop
for i in range(min_length):

    # Rotation matrix calculation
    rotation_matrix = from_q_to_rotation(rot_hand[i])
    if(rotation_matrix == m_nulla).all():
        continue
    
    # Absolute position of the hand and fingers
    x1, y1, z1 = pos_hand[i]
    x_f1, y_f1, z_f1 = pos_f1[i]
    x_f2, y_f2, z_f2 = pos_f2[i]
    x_f3, y_f3, z_f3 = pos_f3[i]
    x_f4, y_f4, z_f4 = pos_f4[i]
    x_f5, y_f5, z_f5 = pos_f5[i]

    # Homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = np.array([x1, y1, z1])

    f1 = np.linalg.inv(T) @ np.array([x_f1, y_f1, z_f1, 1])
    f2 = np.linalg.inv(T) @ np.array([x_f2, y_f2, z_f2, 1])
    f3 = np.linalg.inv(T) @ np.array([x_f3, y_f3, z_f3, 1])
    f4 = np.linalg.inv(T) @ np.array([x_f4, y_f4, z_f4, 1])
    f5 = np.linalg.inv(T) @ np.array([x_f5, y_f5, z_f5, 1])

    # Algoritmo di Hungarian
    P_prev = np.array([f1_rel_prev, f2_rel_prev, f3_rel_prev, f4_rel_prev, f5_rel_prev])
    P_new = np.array([f1, f2, f3, f4, f5])
    cost_matrix = cdist(P_prev, P_new) 
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    f1_rel = P_new[col_ind[0]]
    f2_rel = P_new[col_ind[1]]
    f3_rel = P_new[col_ind[2]]
    f4_rel = P_new[col_ind[3]]
    f5_rel = P_new[col_ind[4]]

    # Creazione mano fissa
    pos_point(origin, 0, 0, 0)
    pos_point(point_h1, -0.025, 0.05, -0.025)
    pos_point(point_h2, -0.025, -0.035, -0.025)
    pos_point(point_h3, 0.05, -0.035, -0.025)
    pos_point(point_h4, 0.052, -0.01, -0.025)
    pos_point(point_h5, 0.055, 0.02, -0.025)
    pos_point(point_h6, 0.052, 0.05, -0.025)
    pos_line(line_h1, point_h1, point_h2)
    pos_line(line_h2, point_h2, point_h3)
    pos_line(line_h3, point_h3, point_h4)
    pos_line(line_h4, point_h4, point_h5)
    pos_line(line_h5, point_h5, point_h6)
    pos_line(line_h6, point_h6, point_h1)

    # Update finger marker positions
    pos_point(marker_f1, f1_rel[0], f1_rel[1], f1_rel[2])
    pos_point(marker_f4, f2_rel[0], f2_rel[1], f2_rel[2])
    pos_point(marker_f3, f3_rel[0], f3_rel[1], f3_rel[2])
    pos_point(marker_f2, f4_rel[0], f4_rel[1], f4_rel[2])
    pos_point(marker_f5, f5_rel[0], f5_rel[1], f5_rel[2])

    # Animation of finger joints
    # Finger 1
    target_1 = np.array([f1_rel[0], f1_rel[1], 0])
    T1 = get_htm_mp(point_h3, target_1, [0.15954817, -0.025, -0.01])
    print(np.linalg.inv(T1) @ np.array([f1_rel[0], f1_rel[1], f1_rel[2], 1]))
    phi_1_prev = phi_1
    success = False
    for delta in range(0, -31, -1):
        phi_1 = phi_1_prev + delta
        try:
            joint1_f1, joint2_f1 = from_final_pos_to_joints(T1, f1_rel, f1_l1, f1_l2, f1_l3, phi_1, threshold)
            success = True
            break
        except Exception as e:
            success = False
    if not success:
        for delta in range(1, 31):
            phi_1 = phi_1_prev + delta
            try:
                joint1_f1, joint2_f1 = from_final_pos_to_joints(T1, f1_rel, f1_l1, f1_l2, f1_l3, phi_1, threshold)
                success = True
                break
            except Exception as e:
                success = False
    if (success==True):
        pos_point(joint1p_f1, joint1_f1[0], joint1_f1[1], joint1_f1[2])
        pos_point(joint2p_f1, joint2_f1[0], joint2_f1[1], joint2_f1[2])
        pos_line(line_f11, point_h3, joint1p_f1)
        pos_line(line_f12, joint1p_f1, joint2p_f1)
        pos_line(line_f13, joint2p_f1, marker_f1)
    else:
        pos_point(marker_f1, f1_rel[0], f1_rel[1], f1_rel[2])

    # Finger 2
    target_2 = np.array([f5_rel[0], f5_rel[1], 0])
    T2 = get_htm_mp(point_h4, target_2, [1.82499501e-01, 0.015, -0.01])
    phi_2_prev = phi_2
    success = False
    for delta in range(0, -31, -1):
        phi_2 = phi_2_prev + delta
        try:
            joint1_f2, joint2_f2 = from_final_pos_to_joints(T2, f5_rel, f2_l1, f2_l2, f2_l3, phi_2, threshold)
            success = True
            break
        except Exception as e:
            success = False
    if not success:
        for delta in range(1, 31):
            phi_2 = phi_2_prev + delta
            try:
                joint1_f2, joint2_f2 = from_final_pos_to_joints(T2, f5_rel, f2_l1, f2_l2, f2_l3, phi_2, threshold)
                success = True
                break
            except Exception as e:
                success = False
    if (success==True):
        pos_point(joint1p_f2, joint1_f2[0], joint1_f2[1], joint1_f2[2])
        pos_point(joint2p_f2, joint2_f2[0], joint2_f2[1], joint2_f2[2])
        pos_line(line_f21, point_h4, joint1p_f2)
        pos_line(line_f22, joint1p_f2, joint2p_f2)
        pos_line(line_f23, joint2p_f2, marker_f5)
    else:
        pos_point(marker_f5, f5_rel[0], f5_rel[1], f5_rel[2])

    # Finger 3
    target_3 = np.array([f4_rel[0], f4_rel[1], 0])
    T3 = get_htm_mp(point_h5, target_3, [0.18140122, 0.03, -0.01])
    phi_3_prev = phi_3
    success = False
    for delta in range(0, -31, -1):
        phi_3 = phi_3_prev + delta
        try:
            joint1_f3, joint2_f3 = from_final_pos_to_joints(T3, f4_rel, f3_l1, f3_l2, f3_l3, phi_3, threshold)
            success = True
            break
        except Exception as e:
            success = False
    if not success:
        for delta in range(1, 31):
            phi_3 = phi_3_prev + delta
            try:
                joint1_f3, joint2_f3 = from_final_pos_to_joints(T3, f4_rel, f3_l1, f3_l2, f3_l3, phi_3, threshold)
                success = True
                break
            except Exception as e:
                success = False
    if (success==True):
        pos_point(joint1p_f3, joint1_f3[0], joint1_f3[1], joint1_f3[2])
        pos_point(joint2p_f3, joint2_f3[0], joint2_f3[1], joint2_f3[2])
        pos_line(line_f31, point_h5, joint1p_f3)
        pos_line(line_f32, joint1p_f3, joint2p_f3)
        pos_line(line_f33, joint2p_f3, marker_f2)
    else:
        pos_point(marker_f2, f4_rel[0], f4_rel[1], f4_rel[2])

    # Finger 4
    target_4 = np.array([f2_rel[0], f2_rel[1], 0])
    T4 = get_htm_mp(point_h6, target_4, [0.14718147, 0.053, -0.01])
    phi_4_prev = phi_4
    success = False
    for delta in range(0, -31, -1):
        phi_4 = phi_4_prev + delta
        try:
            joint1_f4, joint2_f4 = from_final_pos_to_joints(T4, f2_rel, f4_l1, f4_l2, f4_l3, phi_4, threshold)
            success = True
            break
        except Exception as e:
            success = False
    if not success:
        for delta in range(1, 31):
            phi_4 = phi_4_prev + delta
            try:
                joint1_f4, joint2_f4 = from_final_pos_to_joints(T4, f2_rel, f4_l1, f4_l2, f4_l3, phi_4, threshold)
                success = True
                break
            except Exception as e:
                success = False
    if (success==True):
        pos_point(joint1p_f4, joint1_f4[0], joint1_f4[1], joint1_f4[2])
        pos_point(joint2p_f4, joint2_f4[0], joint2_f4[1], joint2_f4[2])
        pos_line(line_f41, point_h6, joint1p_f4)
        pos_line(line_f42, joint1p_f4, joint2p_f4)
        pos_line(line_f43, joint2p_f4, marker_f4)
    else:
        pos_point(marker_f4, f2_rel[0], f2_rel[1], f2_rel[2])

    # Finger 5
    target_5 = np.array([f3_rel[0], f3_rel[1], 0])
    T5 = get_htm_mp(point_h1, target_5, [0.02415149, 0.13794686, -0.01])
    phi_5_prev = phi_5
    success = False
    for delta in range(0, -31, -1):
        phi_5 = phi_5_prev + delta
        try:
            joint1_f5, joint2_f5 = from_final_pos_to_joints(T5, f3_rel, f5_l1, f5_l2, f5_l3, phi_5, threshold)
            success = True
            break
        except Exception as e:
            success = False
    if not success:
        for delta in range(1, 31):
            phi_5 = phi_5_prev + delta
            try:
                joint1_f5, joint2_f5 = from_final_pos_to_joints(T5, f3_rel, f5_l1, f5_l2, f5_l3, phi_5, threshold)
                success = True
                break
            except Exception as e:
                success = False
    if (success==True):
        pos_point(joint1p_f5, joint1_f5[0], joint1_f5[1], joint1_f5[2])
        pos_point(joint2p_f5, joint2_f5[0], joint2_f5[1], joint2_f5[2])
        pos_line(line_f51, point_h1, joint1p_f5)
        pos_line(line_f52, joint1p_f5, joint2p_f5)
        pos_line(line_f53, joint2p_f5, marker_f3)
    else:
        pos_point(marker_f3, f3_rel[0], f3_rel[1], f3_rel[2])

    f1_rel_prev = f1_rel
    f2_rel_prev = f2_rel
    f3_rel_prev = f3_rel
    f4_rel_prev = f4_rel
    f5_rel_prev = f5_rel

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.001)

plt.ioff()
plt.show()