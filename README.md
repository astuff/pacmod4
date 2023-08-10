# PACMod4 (Platform Actuation and Control MODule) ROS Driver #

[![CircleCI](https://circleci.com/gh/astuff/pacmod4/tree/ros1_master.svg?style=svg)](https://circleci.com/gh/astuff/pacmod3/tree/ros1_master)

This ROS node is designed to allow the user to control a vehicle with the PACMod drive-by-wire system.
The main purpose of the driver is to provide a common ROS API to PACMod devices regardless of vehicle type or specific PACMod version in use.

For access to the DBC file which defines the CAN interface for the PACMod, see the [pacmod_dbc](https://github.com/astuff/pacmod_dbc) repo.

## Installation 

Install pacmod4 using our debian repository:

```sh
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod4
```

## ROS API

The driver will automatically adapt the ROS API (published and subscribed topics) according to what data the PACMod system supports. 
Please consult the PACMod user manual you received with your vehicle in order to determine what types of data the PACMod system is able to send and receive.

### Launch Arguments

- **dbc_major_version**: This should be set to match the DBC version of your PACMod system. See the "Supported Vehicles" section at the bottom of this README for more details.
- **use_kvaser**: Set this to true if a Kvaser CAN device is being used with Kvaser canlib drivers to connect to the PACMod. Defaults to `false`.
- **kvaser_hardware_id**: The hardware id of the kvaser device, only applies if `use_kvaser` is true.
- **kvaser_circuit_id**: The circuit/channel id that the PACMod is plugged into on the kvaser device, only applies if `use_kvaser` is true.
- **use_socketcan**: Set this to true if Linux SocketCAN drivers are being used to connect to the PACMod. Defaults to `false`.
- **socketcan_device**: The device id of the SocketCAN channel the PACMod is plugged into, only applies if `use_socketcan` is true.
- **namespace**: The namespace of the PACMod driver, topics will be namespaced accordingly. Defaults to `pacmod`.