# External model artifacts

This cleaned release intentionally excludes learned model weights.

Provide the following artifacts through launch arguments after confirming that
their data and redistribution rights permit publication:

- `bc_model_path` for the Behavioral Cloning steering checkpoint
- `traffic_light_model_path` for the traffic-light detector
- `yolo_model_path` for the pedestrian/vehicle detector

Do not commit weights, training data, rosbag files, or raw experiment outputs
to this repository by default.
