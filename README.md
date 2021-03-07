# ROS driver for ublox_snout ZED-F9P receiver

Just quick hardcode to publish some UBX messages to ROS.
Disabled configuration of the receiver via yaml. Reciever should be configured via u-center software.
Hardcoded to work as HPG Rover device.
NavRELPOSNED.msg updated to match u-blox 9 protocol version 27.1

## Options

zed-f9p.yaml (only for seting up device connection and published messages)

## Launch

roslaunch ublox_snout_gps ublox_snout_zed-f9p.launch

