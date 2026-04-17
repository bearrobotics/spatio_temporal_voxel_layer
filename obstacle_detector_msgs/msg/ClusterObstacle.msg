Header header
# Assigned IDs
uint32 cluster_id
uint32 tracker_id

# Tracking states
uint8 TRACKING_STATE_UNKNOWN = 0
uint8 TRACKING_STATE_REGISTRATION = 1
uint8 TRACKING_STATE_ACTIVE = 2
uint8 TRACKING_STATE_INACTIVE = 3
uint8 tracking_state

# Obstacle types
uint8 OBSTACLE_TYPE_UNKNOWN = 0
uint8 OBSTACLE_TYPE_STATIC = 1
uint8 OBSTACLE_TYPE_DYNAMIC = 2
uint8 OBSTACLE_TYPE_OCCLUDED = 3
uint8 obstacle_type

# Probability of being static object
float64 static_probability
# Point cloud of this cluster
sensor_msgs/PointCloud2 points

# Center of the points [m]
geometry_msgs/Point center
# Velocity of the cluster center point [m]
geometry_msgs/Vector3 velocity
# Shape of cluster
uint8 shape_type
# Polygon shape
geometry_msgs/Polygon polygon
# Extended polygons if pcl extender is enabled
geometry_msgs/Polygon[] extended_polygons
# If polygon is empty and radius > 0,then assume the shape is circle
float64 radius
# Tracker(KalmanFilter) state covariances for x, y and radius
float64[4] covariance_x
float64[4] covariance_y
float64[4] covariance_r

obstacle_detector/ModelInfo[] model_infos
