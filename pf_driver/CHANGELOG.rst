^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pf_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2022-05-03)
-------------------
* fix focal point to world link of r2300 `#72 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/72>`_ 
* fix xacro filename cases `#70 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/70>`_ 
* viz scanner model in rviz
* changed necessary for noetic `#68 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/68>`_
* added urdf for r2000 and r2300 `#66 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/66>`_
* fix deps for CI
* add urdf bringup launch file
* added urdf for r2300
* fix high CPU usage `#65 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/65>`_ from PepperlFuchs/fix_while_loop
* fix pointcloud flickering in rviz
* reset cloud when params change in between publish cycle
* reduce sleep time for writer thread as observed in `#63 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/63#issuecomment-889831408>`_
* wait for the next message from the device with a timeout
* fix layers accumulation and low frequency issue `#56 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/56>`_ from PepperlFuchs/fix_freq
* Contributors: Harsh Deshpande

1.1.1 (2021-02-18)
-------------------
* Update package.xml
* Contributors: Pepperl+Fuchs SE

1.1.0 (2021-02-09)
-------------------
* Update package.xml
* Merge pull request `#53 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/53>`_ from PepperlFuchs/fix-typo
  fix typo
* fix typo
* Merge pull request `#52 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/52>`_ from PepperlFuchs/fix-exec-name
  fix exec name
* changed default start_angle of R2300
* fix start_angle
* dummy
* fix start_angle
* check negative start_angle
* fix scan output rviz
* fix watchdog
* fix set scanconfig
* fix private namespace
* fix exec name
* Merge pull request `#50 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/50>`_ from wsaihopfsg/rviz_start_angle_max_num_points_scan
  Rviz start angle max num points scan
* updated PFSDP for R2000
* resolved merge conflicts
* Update pfsdp_2300.hpp
  To correct the bug for incorrect display when start_angle and max_num_points_scan is specified
* Update pfsdp_protocol.hpp
  To correct the bug in RViz display when start_angle & max_num_points_scan are specified
* Update scan_publisher.cpp
  Consider start_angle & max_num_points_scan for RViz
* Merge pull request `#49 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/49>`_ from wsaihopfsg/dyn_reconfig_start_angle_max_points_scan
  Dyn reconfig start angle max points scan
* Merge pull request `#51 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/51>`_ from PepperlFuchs/protocol_args
  Protocol args
* fix config not updated
* fix indentation
* Update pf_interface.cpp
  Communicates the new config to R2300 after dynamic reconfiguration
* Update pfsdp_protocol.hpp
  Communicates the new config to R2300 during dynamic reconfig
* fixed launch files
* handle optional args
* format check in CI
* Merge pull request `#46 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/46>`_ from wsaihopfsg/config_watchdog
  Update pfsdp_protocol.hpp
* Merge branch 'master' into config_watchdog
* Merge pull request `#40 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/40>`_ from PepperlFuchs/format_lint
  fixes for formatting and linting errors
* Merge branch 'master' into config_watchdog
* Update pfsdp_protocol.hpp
* fix catkin_lint
* fix roslint
* applied clang formatting
* Merge pull request `#39 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/39>`_ from PepperlFuchs/pcl_intensities
  add intensities to PCL fields
* Merge branch 'pcl_intensities' of https://github.com/PepperlFuchs/ROS_driver into pcl_intensities
* add intensities to PCL fields
* fix no return statement warning
* fix missing jsoncpp include
* Merge pull request `#31 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/31>`_ from PepperlFuchs/fix-dyn-params
  fix ip_mode and layer_on
* fix ip_mode and layer_on
* Merge pull request `#26 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/26>`_ from MilanoTechnicalGroupInc/r2000_fix
  R2000 fix
* Fix parsing TCP streams/packets
  UDP transport puts one PSDF packet per UDP packet; TCP jams all PSDF packets into a single stream which is often delivered in chunks that don't match the size of the actual TCP packets.
  Here, we make two changes to account for this:
  1. Restructure the parser to parse an arbitrary number of packets iteratively from its buffer (avoid multiple recursion, though it's probably fine).
  2. Restructure the writer to persist any unused bytes from the previous parse and prepend them to the next packet that comes in over the network.
* Validate buffer size based on packet header
  Original implementation compared the computed data size (based on sizes in the packet header) to the buffer size less the nominal size of the known packet header structure.  However, some (I guess older?) R2000s use a smaller packet header (60 bytes vs 76).  Trying to parse packets from these devices results in almost universal failure, the original condition ends up being "does the packet have at least 16 extra bytes at the end" to which the answer is usually "no" (except if the packet is part of a TCP stream and has the beginning of the next PFSDP packet immediately after it in the buffer).
  This code is still not ideal, as the first few data bytes are also parsed into the header in that case, so we'll end up with some header fields that might be invalid (with no indication to the user).  It also doesn't modify the outer check (in `include/pf_driver/pf/pf_parser.h`) that verifies that there's at least as much data as the expected header size, so in rare cases (packets with only a few points) we may drop (or delay) those packets; however that should be fairly uncommon (packet sizes are based on scan parameters and AFAICT tend to avoid nearly empty packets).
* Default to packet type C for R2000
  Basically no reason to use type A; we know how to parse all three types, A and C use the same space, and C gives us additional information.
  Don't alter the default for R2300, (which I believe means it uses type C1?)
  This also requires us to add the necessary arguments to `request_handle\_{tcp,udp}` in the PFSDP protocol header.
* Refactor HTTP get calls in request_handle\_{tcp,udp}
  - Refactor HTTP get call to allow passing std::map in addition to initializer list (initializer lists are great for quick use, but make constructing complicated queries difficult, since they are immutable)
  -> Expect this to come in handy if additional arguments to the handle request are added later (e.g. start angle, max number of points per scan, etc)
  - Properly construct query based on which arguments have been specified
* Parse R2000 packet type B properly
  Useful code was commented out for some reason.  Changing it slightly to
  match the other packet types and for more efficiency.
* Publish echo amplitudes as intensities in ROS LaserScan messages
  Except for R2000 packet type A, which doesn't include intensity
* Merge pull request `#29 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/29>`_ from PepperlFuchs/fix-dyn-params
  Fixed setting scan output config
* Merge branch 'master' into fix-dyn-params
* Merge pull request `#27 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/27>`_ from MilanoTechnicalGroupInc/param_fix
  Minor fixes for dynamic reconfigure on R2000
* Only set up one dynamic reconfigure server per node
  Otherwise only the later of the two is active (I think?) and we end up unable to use dynamic reconfigure on R2000.
  Also move the server setup to a more sensible place, now that this is
  getting more complicated.
* Fix a few R2000 config parameters
  several "value" fields weren't appropriately set (spaces, or longer names used instead), and one of the "Watchdog" enum values had a copy/paste error.
* Fixed setting scan output config
* Merge pull request `#24 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/24>`_ from MilanoTechnicalGroupInc/buildfix
  Fix clean build
* Fix clean build
  Missing dependencies caused fresh builds to fail because the message
  headers weren't generated until too late
* Merge pull request `#21 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/21>`_ from PepperlFuchs/cleanup
  Added dynamic reconf for R2000
* Added dynamic reconf for R2000
* Merge pull request `#20 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/20>`_ from PepperlFuchs/cleanup
  Cleanup
* fix protocol interface build
* fix dynamic reconf
* handle product versions
* Merge pull request `#19 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/19>`_ from PepperlFuchs/cleanup
  Cleanup & Fixed device handling
* handle product versions
* removed unused files & reshuffled files
* Merge pull request `#18 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/18>`_ from PepperlFuchs/curl
  fix flicker
* fix flicker
* Merge pull request `#15 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/15>`_ from PepperlFuchs/curl
  fix pointcloud
* fix pointcloud
* Merge pull request `#14 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/14>`_ from PepperlFuchs/curl
  Refactor
* laserscan to pointcloud
* added dynamic reconf
* publish R2300 header
  scan still not visible properly
* R2000 with data parsing
* revamped transport
* replaced cpprestsdk with curlpp
* scan publisher
* Initializes R2300
* complete pipeline for R2000 packet A
* publishes scans with full message
  TODO: display is not correct
* pipeline from TCP to publish header
* removed files
* simplified buf read
* Refactored code. Handles connection well
  TODO: parse data
* added lock-free queue
* moved files
* removed / moved files
* Merge pull request `#13 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/13>`_ from PepperlFuchs/protocol_classes
  Protocol classes
* message for param not found
* added new dynamic params
* Merge branch 'master' of https://github.com/PepperlFuchs/ROS_driver
* separate classes for R2000 and R2300
* install workspace
* Merge pull request `#5 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/5>`_ from MilanoTechnicalGroupInc/master
  Update r2300_allscans launch file to pass args
* Merge pull request `#11 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/11>`_ from PepperlFuchs/fix_compile
  Fix compile
* Merge branch 'master' into fix_compile
* Fixed gitk issues
* clang formatting
* Added ROS tests
* Imrpoved error handling for PFSDP
* Added gtests
  Currently only for HTTPInterface, need to extend it to other classes
* Added error checking for HTTP GET
* [WIP] added error handling for protocol
* Applied clang formatting
* [WIP] Small test to de-serialize packet header using ROS
* Merge pull request `#8 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/8>`_ from PepperlFuchs/merge_scans
  merges all laser scans into pointcloud
* Merge branch 'master' into merge_scans
* merges all laser scans into pointcloud
* Merge pull request `#7 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/7>`_ from PepperlFuchs/ros_serialization
  Ros serialization
* clang formatting
* Added ROS tests
* Imrpoved error handling for PFSDP
* Added gtests
  Currently only for HTTPInterface, need to extend it to other classes
* Added error checking for HTTP GET
* [WIP] added error handling for protocol
* Applied clang formatting
* [WIP] Small test to de-serialize packet header using ROS
* Merge pull request `#6 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/6>`_ from PepperlFuchs/dyn_recfg
  Dynamic reconfigure
* Merge branch 'master' into dyn_recfg
* Update r2300_allscans launch file to pass args
* Merge pull request `#4 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/4>`_ from PepperlFuchs/issue_ip
  Fixed hard-coded host address
* Fixed hard-coded host address
* Added dynamic reconfigure for scan_frequency parameter
* Fixed calculations for angle_min and angle_max
* Sets timestamp and angular_increment from ROS message
  Calculates time_increment from timestamp
* Invalid echoes set to NAN
* Merge pull request `#10 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/10>`_ from ipa320/rings
  Fixed angle_min and angle_max
* Fixed angle_min and angle_max
* Merge pull request `#9 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/9>`_ from ipa320/rings
  Added static_transform to visualize R2300 rings
* Starts only 1 publisher in case of R2000
* Added static transforms to viz all rings as per datasheet
* Merge pull request `#8 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/8>`_ from ipa320/rings
  Rings
* publishes rings with respective frame_id
* Visualizes data cleanly without crash
* Changed string to basic_string<u_char>
* Merge pull request `#7 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/7>`_ from ipa320/rings
  Publishes rings on respective topics
* Publishes rings on respective topics
* Merge pull request `#6 <https://github.com/PepperlFuchs/pf_lidar_ros_driver/issues/6>`_ from ipa320/generic
  Generic code for R2000 and R2300
* Generic code for R2000 and R2300
* Applied clang-format
* Added Apache 2.0 license
* Basic working code for R2300
* Contributors: Andres, Ben Kurtz, Benjamin Kurtz, Harsh Deshpande, Pepperl+Fuchs AG, Pepperl+Fuchs SE, wsaihopfsg
