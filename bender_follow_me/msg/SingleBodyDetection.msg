
int8    id      # detection id
float32 x       # detection position: x [m]
float32 y       # detection position: y [m]
float32 z       # detection position: z [m]
float32 heading # detection heading     [deg]
float32 vel     # detection speed       [m/s]

# This is for leg detector only! ...
# TODO: find another way to send this information
int8 leg_type   # LA, FS, SL

