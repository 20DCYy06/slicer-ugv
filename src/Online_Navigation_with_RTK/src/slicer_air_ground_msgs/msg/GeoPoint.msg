# WGS84 degrees, horizontal base_link target; already compensated once.
float64 latitude
float64 longitude
# Missing height is explicit; the receiver applies its documented 2D convention.
bool has_altitude
float64 altitude_ellipsoid_m
# Optional final-goal yaw only, radians CCW from WGS84 ENU East.
bool has_yaw
float64 yaw
