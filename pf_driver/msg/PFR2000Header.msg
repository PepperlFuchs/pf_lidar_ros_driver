pf_driver/PFHeader header

uint64 timestamp_raw      # raw timestamp of first scan point in this packet in NTP time format
uint64 timestamp_sync     # synchronized timestamp of first scan point in this packet in NTP time format
                                    #(currenty not available and and set to zero)
uint32 status_flags       # scan status flags (TODO: should this be a struct? or provide masking to extract
                                    # specific flags)
uint32 scan_frequency     # frequency of head rotation  (1/1000Hz)
uint16 num_points_scan    # number of scan points (samples) within complete scan
uint16 num_points_packet  # total number of scan points within this packet
uint16 first_index        # index of first scan point within this packet
int32 first_angle         # absolute angle of first scan point within this packet  (1/10000°)
int32 angular_increment   # delta between two succeding scan points (1/10000°) CCW rotation: +ve, CW
                                    # rotation: -ve
uint32 iq_input           # reserved - all bits zero for devices without switching I/Q
uint32 iq_overload        # reserved - all bits zero for devices without switching I/Q
uint64 iq_timestamp_raw   # raw timestamp for status of switching I/Q
uint64 iq_timestamp_sync  # synchronized timestamp for status of switching I/Q
#uint8[] padding           # 0-3 bytes padding to align header size to 32bit boundary (TODO: should I use