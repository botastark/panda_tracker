from task_pose_sender import TaskPosePublisher, pose6_to_transform

publisher = TaskPosePublisher(
    destination_ip="127.0.0.1",
    destination_port=6501,
)

try:
    while True:
        # Values produced by the vision algorithm.
        pose6 = [
            x_T_S,
            y_T_S,
            z_T_S,
            roll_T_S,
            pitch_T_S,
            yaw_T_S,
        ]

        T_TS = pose6_to_transform(pose6)
        publisher.publish_matrix(T_TS)

finally:
    publisher.close()