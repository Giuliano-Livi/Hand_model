import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from functions_for_model import from_q_to_rotation, pos_line, pos_point, angle_between_vectors, normalize_angle
import math
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time
import rosbag

# Select the dataset you want to work on
chosen_dataset = "power_grasp1.bag"


#-----------------------------------------------------------------------
# Load EMG data from a specific bag file
# Initialize list to store EMG data
emg_data_specimen = []
timestamps_specimen = []
selected_topic = "/emg"  # Change this to the topic you want to read

# Choose which bag file to load for specimen and analysis
bag_path_specimen = "bag_emg/" + chosen_dataset     # <-- Change here to use a different file
#bag_path_test = power

# Open the bag and extract EMG values from messages
with rosbag.Bag(bag_path_specimen, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[selected_topic]):
        try:
            for i in msg.emg:  # Read each value in the EMG array
                emg_data_specimen.append(i)
                timestamps_specimen.append(t.to_sec())
        except AttributeError as e:
            print("Message missing expected fields:", e)
            break

# Convert to numpy arrays
emg_data_specimen = np.array(emg_data_specimen)
timestamps_specimen = np.array(timestamps_specimen)


# Data Processing - Reshape raw EMG vector into (16 x N) matrix format
# The bag file streams data as a flat list; this section reformats it into 16 channels
selector = 0
raw_emg = np.empty((16, 0))  # Initialize empty matrix with 16 rows (channels)

for i in range(int(len(emg_data_specimen)/16)):
    temp = emg_data_specimen[selector:selector+16]            # Extract 16 consecutive samples
    new_column = np.array(temp).reshape(16, 1)                # Convert to column format
    raw_emg = np.hstack((raw_emg, new_column))                # Append column to EMG matrix
    selector += 16                                            # Move to next block
    sample_number =  i
# Print shape information of extracted data
print(sample_number)



# Initialize lists to store hand and marker positions and orientation (only of the hand)
hand_positions = []
hand_orientations = []
marker_positions = [[] for _ in range(5)]  # 5 marker
last_marker_positions = [None] * 5         # To track last valid positions of markers (to use in case we lost a marker for a frame)
timestamp = []                             # To track the time of each frame

# Load the bag file containing Vicon data
bag = rosbag.Bag('bag_vicon/' + chosen_dataset)

# Extract hand and marker positions from the bag file
for topic, msg, t in bag.read_messages():
    if topic == "/tf":                                          # /tf topic contains the hand positions and orientations
        for transform in msg.transforms:
            if transform.child_frame_id == "hand":
                pos = transform.transform.translation
                rot = transform.transform.rotation
                hand_positions.append([pos.x, pos.y, pos.z])
                hand_orientations.append([rot.x, rot.y, rot.z, rot.w])
                timestamp.append(t.to_sec())

    elif topic == "/vicon/unlabeled_markers":                  # /vicon/unlabeled_markers topic contains the marker positions
        current_marker_positions = [None] * 5
        for i, marker in enumerate(msg.markers):               # Loop to update the last known positions of the markers
            if i < 5:
                pos = marker.pose.position
                current_marker_positions[i] = [pos.x, pos.y, pos.z]
                last_marker_positions[i] = current_marker_positions[i]  

        for i in range(5):                                     # Loop to append the current marker positions to the marker_positions list
            if current_marker_positions[i] is None:            # If the marker is not found in the current frame, use the last known position
                if last_marker_positions[i] is not None:
                    marker_positions[i].append(last_marker_positions[i])
                else:
                    marker_positions[i].append([np.nan, np.nan, np.nan])  # If no last position is available, append NaN
            else:
                marker_positions[i].append(current_marker_positions[i])
bag.close()

# Save the extracted data to .npy files
np.save("npy_files/hand_positions.npy", np.array(hand_positions))
np.save("npy_files/hand_orientations.npy", np.array(hand_orientations))
for i in range(5):
    np.save(f"npy_files/marker_{i}_positions.npy", np.array(marker_positions[i]))


# Calculate the frequency and duration of vicon data
timestamp = np.array(timestamp)
timestamp_int = len(timestamp)
duration = timestamp[-1] - timestamp[0]
print(f"Total duration: {duration:.5f} seconds")
frequency = timestamp_int / duration
print(f"Frequency: {frequency:.2f} Hz")

# Print the number of samples found for hand and markers
print("Hand:", len(hand_positions), "samples")
for i, positions in enumerate(marker_positions):
    print(f"Marker {i}: {len(positions)} samples")


# Load the saved data from .npy files into lists
pos_hand = np.load("npy_files/hand_positions.npy").tolist()
rot_hand = np.load("npy_files/hand_orientations.npy").tolist()
pos_f1 = np.load("npy_files/marker_0_positions.npy").tolist()
pos_f2 = np.load("npy_files/marker_1_positions.npy").tolist()
pos_f3 = np.load("npy_files/marker_2_positions.npy").tolist()
pos_f4 = np.load("npy_files/marker_3_positions.npy").tolist()
pos_f5 = np.load("npy_files/marker_4_positions.npy").tolist()

min_length = min(len(pos_hand), len(rot_hand), len(pos_f1), len(pos_f2), len(pos_f3), len(pos_f4), len(pos_f5))

# Find the number of times you have to use the same vicon data to match the EMG data length
n_of_times = round(sample_number/len(pos_hand))
print("Number of times to repeat vicon data:", n_of_times)


# Set up the 3D plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.2, 0.2)
ax.set_ylim(-0.2, 0.2)
ax.set_zlim(-0.2, 0.2)

# Definition of the points and lines of the hand structure
#name     = ax.plot([x],[y],[z], 'color/shape', size)
point_h1, = ax.plot([], [], [], 'ko', markersize=3)
point_h2, = ax.plot([], [], [], 'ko', markersize=3)
point_h3, = ax.plot([], [], [], 'ko', markersize=3)
point_h4, = ax.plot([], [], [], 'ko', markersize=3)
point_h5, = ax.plot([], [], [], 'ko', markersize=3)
point_h6, = ax.plot([], [], [], 'ko', markersize=3)
line_h1, = ax.plot([], [], [], 'k-', linewidth=3)
line_h2, = ax.plot([], [], [], 'k-', linewidth=3)
line_h3, = ax.plot([], [], [], 'k-', linewidth=3)
line_h4, = ax.plot([], [], [], 'k-', linewidth=3)
line_h5, = ax.plot([], [], [], 'k-', linewidth=3)
line_h6, = ax.plot([], [], [], 'k-', linewidth=3)

# Definition of fingers length
little_length = 0.085
ring_length = 0.1
middle_length = 0.13
index_length =  0.1
thumb_length = 0.14


# Little definition
little_chain = Chain(name='ring', links=[
    URDFLink(
        name="joint1",
        translation_vector=[0.083, -0.05, -0.025],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        bounds=(np.pi/3, np.pi)
    ),
    URDFLink(
        name="joint2",
        translation_vector=[0, 0, little_length*0.5],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint3",
        translation_vector=[0, 0, little_length*0.34],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint4",
        translation_vector=[0, 0, little_length*0.26],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    )
])

# Ring definition
ring_chain = Chain(name='ring', links=[
    URDFLink(
        name="joint1",
        translation_vector=[0.09, -0.01, -0.025],
        orientation=[0, 0, 0],
        rotation=[-0.2425, 0.9701, 0],
        bounds=(np.pi/3, np.pi)
    ),
    URDFLink(
        name="joint2",
        translation_vector=[0, 0, ring_length*0.5],
        orientation=[0, 0, 0],
        rotation=[-0.2425, 0.9701, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint3",
        translation_vector=[0, 0, ring_length*0.34],
        orientation=[0, 0, 0],
        rotation=[-0.2425, 0.9701, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint4",
        translation_vector=[0, 0, ring_length*0.26],
        orientation=[0, 0, 0],
        rotation=[-0.2425, 0.9701, 0]
    )
])

# Midlle definition
middle_chain = Chain(name='middle', links=[
    URDFLink(
        name="joint1",
        translation_vector=[0.09, 0.03, -0.025],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(np.pi/3, np.pi)
    ),
    URDFLink(
        name="joint2",
        translation_vector=[0, 0, middle_length*0.5],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint3",
        translation_vector=[0, 0, middle_length*0.34],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint4",
        translation_vector=[0, 0, middle_length*0.26],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0]
    )
])

# Index definition
index_chain = Chain(name='index', links=[
    URDFLink(
        name="joint1",
        translation_vector=[0.07, 0.07, -0.025],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(np.pi/3, np.pi)
    ),
    URDFLink(
        name="joint2",
        translation_vector=[0, 0, index_length*0.5],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint3",
        translation_vector=[0, 0, index_length*0.34],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint4",
        translation_vector=[0, 0, index_length*0.26],
        orientation=[0, 0, 0],
        rotation=[-0.36765889, 0.92996072, 0]
    )
])

# Thumb definition
thumb_chain = Chain(name='thumb', links=[
    URDFLink(
        name="joint1",
        translation_vector=[-0.01, 0.01, -0.025],
        orientation=[0, 0, 0],
        rotation=[-0.68558295, -0.68558295, -0.24485105],
        bounds=(np.pi/3, np.pi)
    ),
    URDFLink(
        name="joint2",
        translation_vector=[0, 0, thumb_length*0.34],
        orientation=[0, 0, 0],
        rotation=[-0.68558295, -0.68558295, -0.24485105],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint3",
        translation_vector=[0, 0, thumb_length*0.34],
        orientation=[0, 0, 0],
        rotation=[-0.68558295, -0.68558295, -0.24485105],
        bounds=(0, np.pi/2)
    ),
    URDFLink(
        name="joint4",
        translation_vector=[0, 0, thumb_length*0.26],
        orientation=[0, 0, 0],
        rotation=[-0.68558295, -0.68558295, -0.24485105]
    )
])


# Definition of reference hand points
pos_point(point_h1, -0.01, 0.01, -0.025)
pos_point(point_h2, -0.01, -0.06, -0.025)
pos_point(point_h3, 0.083, -0.05, -0.025)
pos_point(point_h4, 0.09, -0.01, -0.025)
pos_point(point_h5, 0.09, 0.03, -0.025)
pos_point(point_h6, 0.07, 0.07, -0.025)
pos_line(line_h1, point_h1, point_h2)
pos_line(line_h2, point_h2, point_h3)
pos_line(line_h3, point_h3, point_h4)
pos_line(line_h4, point_h4, point_h5)
pos_line(line_h5, point_h5, point_h6)
pos_line(line_h6, point_h6, point_h1)

# Definition of initial pose
f1_rel_prev = np.array([0.13868391, -0.05139445,  0.00857016, 1])
f2_rel_prev = np.array([0.11976245,  0.13734637, -0.00763299, 1])
f3_rel_prev = np.array([-0.0369033, 0.15533912, -0.02849389, 1])
f4_rel_prev = np.array([0.16185893, 0.08565602, 0.008876, 1])
f5_rel_prev = np.array([0.16463813, 0.03239017, 0.01738579, 1])
m_nulla = np.array([[1,0,0],[0,1,0],[0,0,1]])

# Create the sigma matrix for vicon data
sigma = []

# Initialization loop
for i in range(1):
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(0.05)

# Initialize chain artists, used to clear from the previous iteration chains representation
finger_line_little = None
finger_scatter_little = None
finger_line_ring = None
finger_scatter_ring = None
finger_line_middle = None
finger_scatter_middle = None
finger_line_index = None
finger_scatter_index = None
finger_line_thumb = None
finger_scatter_thumb = None


# Animation loop
for i in range(min_length):

    # Rotation matrix calculation to pass from the world frame to the hand frame
    rotation_matrix = from_q_to_rotation(rot_hand[i])
    if(rotation_matrix == m_nulla).all():
        continue
    
    # Loading of the current position of the hand and fingers wrt the world frame
    x1, y1, z1 = pos_hand[i]
    x_f1, y_f1, z_f1 = pos_f1[i]
    x_f2, y_f2, z_f2 = pos_f2[i]
    x_f3, y_f3, z_f3 = pos_f3[i]
    x_f4, y_f4, z_f4 = pos_f4[i]
    x_f5, y_f5, z_f5 = pos_f5[i]

    # Homogeneous transformation matrix to pass from the world frame to the hand frame
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = np.array([x1, y1, z1])

    # Calculate the relative position of the fingers in the hand frame
    f1 = np.linalg.inv(T) @ np.array([x_f1, y_f1, z_f1, 1])
    f2 = np.linalg.inv(T) @ np.array([x_f2, y_f2, z_f2, 1])
    f3 = np.linalg.inv(T) @ np.array([x_f3, y_f3, z_f3, 1])
    f4 = np.linalg.inv(T) @ np.array([x_f4, y_f4, z_f4, 1])
    f5 = np.linalg.inv(T) @ np.array([x_f5, y_f5, z_f5, 1])

    # Hungarian algorithm to match the current finger positions with the previous ones
    # This is done to avoid the problem of the fingers jumping from one position to another
    P_prev = np.array([f1_rel_prev, f2_rel_prev, f3_rel_prev, f4_rel_prev, f5_rel_prev])
    P_new = np.array([f1, f2, f3, f4, f5])
    cost_matrix = cdist(P_prev, P_new) 
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    f1_rel = P_new[col_ind[0]]
    f2_rel = P_new[col_ind[1]]
    f3_rel = P_new[col_ind[2]]
    f4_rel = P_new[col_ind[3]]
    f5_rel = P_new[col_ind[4]]

    # Compute inverse kinematics
    little_position = [f1_rel[0], f1_rel[1], f1_rel[2]]
    ring_position = [f5_rel[0], f5_rel[1], f5_rel[2]]
    middle_position = [f4_rel[0], f4_rel[1], f4_rel[2]]
    index_position = [f2_rel[0], f2_rel[1], f2_rel[2]]
    thumb_position = [f3_rel[0], f3_rel[1], f3_rel[2]]
    little_solution = little_chain.inverse_kinematics(little_position)
    ring_solution = ring_chain.inverse_kinematics(ring_position)
    middle_solution = ring_chain.inverse_kinematics(middle_position)
    index_solution = index_chain.inverse_kinematics(index_position)
    thumb_solution = thumb_chain.inverse_kinematics(thumb_position)

    # Remove the previous representation of the fingers
    if finger_line_little is not None:
        finger_line_little.remove()
        finger_scatter_little.remove()
        finger_line_ring.remove()
        finger_scatter_ring.remove()
        finger_line_middle.remove()
        finger_scatter_middle.remove()
        finger_line_index.remove()
        finger_scatter_index.remove()
        finger_line_thumb.remove()
        finger_scatter_thumb.remove()
    
    # Compute forward kinematics for all links to obtain the joint positions
    transformation_matrixes_little = little_chain.forward_kinematics(little_solution, full_kinematics=True)
    nodes_little = [tm[:3, 3] for tm in transformation_matrixes_little]  # Extract translation vectors
    transformation_matrixes_ring = ring_chain.forward_kinematics(ring_solution, full_kinematics=True)
    nodes_ring = [tm[:3, 3] for tm in transformation_matrixes_ring]  # Extract translation vectors
    transformation_matrixes_middle = middle_chain.forward_kinematics(middle_solution, full_kinematics=True)
    nodes_middle = [tm[:3, 3] for tm in transformation_matrixes_middle]  # Extract translation vectors
    transformation_matrixes_index = index_chain.forward_kinematics(index_solution, full_kinematics=True)
    nodes_index = [tm[:3, 3] for tm in transformation_matrixes_index]  # Extract translation vectors
    transformation_matrixes_thumb = thumb_chain.forward_kinematics(thumb_solution, full_kinematics=True)
    nodes_thumb = [tm[:3, 3] for tm in transformation_matrixes_thumb]  # Extract translation vectors
    
    # Plot new chain
    finger_line_little, = ax.plot([node[0] for node in nodes_little], [node[1] for node in nodes_little], [node[2] for node in nodes_little], color='black', linewidth=3)
    finger_scatter_little = ax.scatter([node[0] for node in nodes_little], [node[1] for node in nodes_little], [node[2] for node in nodes_little], color='black', s=5)
    finger_line_ring, = ax.plot([node[0] for node in nodes_ring], [node[1] for node in nodes_ring], [node[2] for node in nodes_ring], color='black', linewidth=3)
    finger_scatter_ring = ax.scatter([node[0] for node in nodes_ring], [node[1] for node in nodes_ring], [node[2] for node in nodes_ring], color='black', s=5)
    finger_line_middle, = ax.plot([node[0] for node in nodes_middle], [node[1] for node in nodes_middle], [node[2] for node in nodes_middle], color='black', linewidth=3)
    finger_scatter_middle = ax.scatter([node[0] for node in nodes_middle], [node[1] for node in nodes_middle], [node[2] for node in nodes_middle], color='black', s=5)
    finger_line_index, = ax.plot([node[0] for node in nodes_index], [node[1] for node in nodes_index], [node[2] for node in nodes_index], color='black', linewidth=3)
    finger_scatter_index = ax.scatter([node[0] for node in nodes_index], [node[1] for node in nodes_index], [node[2] for node in nodes_index], color='black', s=5)
    finger_line_thumb, = ax.plot([node[0] for node in nodes_thumb], [node[1] for node in nodes_thumb], [node[2] for node in nodes_thumb], color='black', linewidth=3)
    finger_scatter_thumb = ax.scatter([node[0] for node in nodes_thumb], [node[1] for node in nodes_thumb], [node[2] for node in nodes_thumb], color='black', s=5)


    # Get angle of closure of little
    x_h3, y_h3, z_h3 = point_h3.get_data_3d()
    little_joint_coord = np.array(nodes_little[2])
    h3_coord = np.array([x_h3[0], y_h3[0], z_h3[0]])
    knocle1_axis = [x_h3[0] + 0.1, y_h3[0], z_h3[0]] - h3_coord
    knocle1_axis = knocle1_axis / np.linalg.norm(knocle1_axis)
    little_axis = little_joint_coord - h3_coord
    little_axis = little_axis / np.linalg.norm(little_axis)
    angle_little = angle_between_vectors(knocle1_axis, little_axis)

    # Get angle of closure of ring
    x_h4, y_h4, z_h4 = point_h4.get_data_3d()
    ring_joint_coord = np.array(nodes_ring[2])
    h4_coord = np.array([x_h4[0], y_h4[0], z_h4[0]])
    knocle2_axis = [0.16272325, 0.00817894, z_h4[0]] - h4_coord
    knocle2_axis = knocle2_axis / np.linalg.norm(knocle2_axis)
    ring_axis = ring_joint_coord - h4_coord
    ring_axis = ring_axis / np.linalg.norm(ring_axis)
    angle_ring = angle_between_vectors(knocle2_axis, ring_axis)
    
    # Get angle of closure of middle
    x_h5, y_h5, z_h5 = point_h5.get_data_3d()
    middle_joint_coord = np.array(nodes_middle[2])
    h5_coord = np.array([x_h5[0], y_h5[0], z_h5[0]])
    knocle3_axis = [0.17139323, 0.06217872, z_h5[0]] - h5_coord
    knocle3_axis = knocle3_axis / np.linalg.norm(knocle3_axis)
    middle_axis = middle_joint_coord - h5_coord
    middle_axis = middle_axis / np.linalg.norm(middle_axis)
    angle_middle = angle_between_vectors(knocle3_axis, middle_axis)

    # Get angle of closure of index
    x_h6, y_h6, z_h6 = point_h6.get_data_3d()
    index_joint_coord = np.array(nodes_index[2])
    h6_coord = np.array([x_h6[0], y_h6[0], z_h6[0]])
    knocle4_axis = [0.12973476, 0.09361607, z_h6[0]] - h6_coord
    knocle4_axis = knocle4_axis / np.linalg.norm(knocle4_axis)
    index_axis = index_joint_coord - h6_coord
    index_axis = index_axis / np.linalg.norm(index_axis)
    angle_index = angle_between_vectors(knocle4_axis, index_axis)


    # Get angle of closure of thumb
    x_h1, y_h1, z_h1 = point_h1.get_data_3d()
    thumb_joint_coord = np.array(nodes_thumb[2])
    h1_coord = np.array([x_h1[0], y_h1[0], z_h1[0]])
    knocle5_axis = [-0.03948546, 0.08916224, z_h1[0]] - h1_coord
    knocle5_axis = knocle5_axis / np.linalg.norm(knocle5_axis)
    thumb_axis = thumb_joint_coord - h1_coord
    thumb_axis = thumb_axis / np.linalg.norm(thumb_axis)
    angle_thumb = angle_between_vectors(knocle5_axis, thumb_axis)

    # Here all the angles are normalized to express the sigma value of closure of the hand
    sigma_value = normalize_angle(angle_little, angle_ring, angle_middle, angle_index, angle_thumb)
    for i in range(n_of_times):                 # Every value is added n_of_times to the sigma list, to match the EMG data length
        sigma.append(sigma_value)


    # Update the previous position of the fingers
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

# Ensure the sigma list has the same number of elements as the EMG data
sigma_len = len(sigma)
if sigma_len < sample_number:
    for i in range(sample_number - sigma_len):
        sigma.append(sigma_value)

if sigma_len > sample_number:
    sigma = sigma[:sample_number]

print(len(sigma), "sigma values generated")
print(sample_number, "samples in EMG data")

# Plot the sigma values
plt.figure(figsize=(10, 3))
plt.plot(sigma, label='Sigma', color='orange')
plt.legend()
plt.show()