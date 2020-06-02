pf_driver/PFHeader header

uint16 layer_index        # vertical layer index (0..3)
int32 layer_inclination   # vertical layer inclination [1/10000 degree]
uint64 timestamp_raw      # raw timestamp of first scan point in this packet in NTP time format
uint64 reserved1          # reserved - all bits zero for devices without switching I/Q
uint32 status_flags       # scan status flags (TODO: should this be a struct? or provide masking to extract
                                  # specific flags)
uint32 scan_frequency     # frequency of head rotation  (1/1000Hz)
uint16 num_points_scan    # number of scan points (samples) within complete scan
uint16 num_points_packet  # total number of scan points within this packet
uint16 first_index        # index of first scan point within this packet
int32 first_angle         # absolute angle of first scan point within this packet  (1/10000°)
int32 angular_increment   # delta between two succeding scan points (1/10000°) CCW rotation: +ve, CW
                                  # rotation: -ve
uint32 reserved2          # reserved - all bits zero for devices without switching I/Q
uint32 reserved3          # reserved - all bits zero for devices without switching I/Q
uint64 reserved4          # reserved - all bits zero for devices without switching I/Q
uint64 reserved5          # reserved - all bits zero for devices without switching I/Q