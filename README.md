# ntrip_ros

## Overview
`ntrip_ros` is an **NTRIP client** that imports **RTCM streams** into a ROS topic for real-time kinematic (RTK) GPS correction.

This package was originally forked from [tilk/ntrip_ros](https://github.com/tilk/ntrip_ros) and has been improved for better reliability, handling of incomplete reads, and reconnection logic.

## Features
- **Handles RTCM streams** from an NTRIP caster and publishes them to a ROS topic.
- **Automatically detects and parses messages**, handling cases where CORS correction servers do not include `/n/r` characters.
- **Improved error handling**, preventing crashes from `IncompleteRead` exceptions and maintaining a stable connection.
- **Supports GPS GGA messages** for sending position data to the NTRIP caster.
- **Enhanced reconnection logic** to detect zero-length data, close, and reopen the stream automatically.

## Installation
### Dependencies
This package requires the following:
- ROS Melodic
- `mavros_msgs` (for RTCM messages)
- `sensor_msgs` (for GPS data)
- `rtcm_msgs` (if required, from [tilk/rtcm_msgs](https://github.com/tilk/rtcm_msgs))

### Build Instructions
Clone the repository into your catkin workspace:
```sh
cd ~/catkin_ws/src
git clone https://github.com/ikh-innovation/ntrip_ros.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage
### Launching the NTRIP Client
To start the NTRIP client, use:
```sh
roslaunch ntrip_ros ntrip_ros.launch
```

### Required Parameters
Set these parameters in the launch file or ROS parameter server:
```yaml
ntrip_server: "your_ntrip_server.com"
ntrip_user: "your_username"
ntrip_pass: "your_password"
ntrip_stream: "mountpoint"
nmea_gga: "$GPGGA,...."  # Replace with a valid GGA message
timeout: 3  # Timeout in seconds
```

### Generating a $GPGGA Message
If you need to generate a **$GPGGA message**, visit [NMEA Generator](https://www.nmeagen.org/), set a point near your location, click "Generate NMEA file," and copy the **$GPGGA** message into your launch file.

### Publishing RTCM Data
RTCM messages will be published on:
```sh
rostopic echo /rtcm
```

## Related Projects
- [ublox_f9p](https://github.com/ros-agriculture/ublox_f9p) – A ROS driver for the u-blox F9P GNSS receiver.
- [dayjaby/ntrip_ros](https://github.com/dayjaby/ntrip_ros) – An alternative NTRIP client for ROS.
