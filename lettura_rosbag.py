import rosbag
import numpy as np

# Inizializza le liste
hand_positions = []
hand_orientations = []
marker_positions = [[] for _ in range(5)]  # 5 marker
last_marker_positions = [None] * 5  # Per tracciare l'ultima posizione valida

# Carica il file .bag
bag = rosbag.Bag('bagfiles/max_extensions.bag')

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

# Debug
print("Hand:", len(hand_positions), "samples")
for i, positions in enumerate(marker_positions):
    print(f"Marker {i}: {len(positions)} samples")
