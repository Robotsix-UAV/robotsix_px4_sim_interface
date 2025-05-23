# Definition of ModelInfo
# Contains all information needed to spawn a model
string model_dir  # Path to model dir (relative to GZ_SIM_RESOURCE_PATH)
string model_name  # Unique name for the model in gazebo
int32 px4_parameters  # SYS_AUTOSTART parameter number for PX4
float64 x  # X position in meters
float64 y  # Y position in meters
float64 z  # Z position in meters
