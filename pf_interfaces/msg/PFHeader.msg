uint16 magic           # magic byte (0xa25c) marking the beginning of a packet
uint16 packet_type     # scan data packet type (0x0041 - 'A' 0x0042 - 'B' 0x0043 - 'C')
uint32 packet_size     # overall packet size in bytes (header + payload)
uint16 header_size     # header size in bytes (offset to payload data)
uint16 scan_number     # sequence number for scan (incremented for every scan, starting with 0, overflows)
uint16 packet_number   # sequence number for packet (counting packets of a particular scan, starting with 1)