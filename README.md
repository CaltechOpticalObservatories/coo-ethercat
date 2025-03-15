# EtherCAT Soft Controller
# Table of Contents
1. [Introduction](#introduction)
2. [Quick Start](#quick-start)
3. [CiA 402 Specification](#cia-402-specification)
4. [Software Architecture](#software-architecture)
    - [Server](#server)
    - [High Level Soft Controller](#high-level-soft-controller)
    - [Hardware Abstraction Layer](#hardware-abstraction-layer)
5. [Notes](#notes)

# Introduction
In short, this repository allows you to control EtherCAT hardware from a desktop PC.

This software uses the CAN over EtherCAT (CoE) protocol to control CiA 402 compliant motion controller from a desktop PC. The control computer is acting as a soft master in this scheme and should have a dedicated NIC (Ethernet port with own hardware) attached via ethernet cable to an EtherCAT motion controller thus linking the soft master into the dedicated EtherCAT network. __Specifically, do not route the EtherCAT traffic through a network switch.__

# Quick Start
1. Update [settings](settings/masterSettings.yaml) with the network interface name of the NIC attached to the EtherCAT hardware.
2. Update [settings](settings/serverSettings.yaml) with a host IP and port that will be assigned to this software's listening socket.
3. Start the server with `python3 start.py`.
4. On a remote machine (can also be same machine for testing) use the same host IP and port to send motion requests to the workers. See [`src/server.py`](src/server.py) for a comment detailing the required message formats for various actions. To home all workers, send the byte output of: `struct.pack('<HB', 3, 0)`
5. The script `commands.py` can pack and send this information for you.
5. The script `run_hom.sh` is a simple example of how to send a homing request to the server using `commands.py` and `run_ppmpdo.sh` is an example of how to send a profile position mode request to the server. 


# CiA 402 Specification  
The CAN in Automation (CiA) group has released a technical specification for power drive systems that is designed to make communication robust and safe. The most important thing to know about this specification is that it ensures robust and safe operation by standardizing device states, network management states, and operating modes. Certain actions, for example movement and configuration, are only available in the correct combination of device state, network management state, and operating mode. [A CoE CiA 402 guide](https://caltech.sharepoint.com/:b:/r/sites/coo/LRIS-2/Shared%20Documents/LRIS-2%20-%20Subsystems%20%5BL3%5D/Maxon%20motion%20control/EPOS4%20Micro%2024%205%20EtherCAT%20Quick%20Start%20with%20Merged%20and%20Condensed%20Maxon%20Documentation.pdf?csf=1&web=1&e=m6LDQB) has been made specifically for maxon's EPOS Micro 24/5 EtherCAT worker hardware, which is worth a read to get more information about the CiA 402 specification.

# Software Architecture
## Server
Currently, the server is a simple socket that listens for incoming messages and performs certain actions based off of the requests within the messages. For more details on the specific data that needs to be sent to the socket to request certain actions, see [`src/server.py`](src/server.py).

## High Level Soft Controller
The high level controller breaks down many high level fundamental requests into sequences of CoE commands that will be performed by the hardware abstraction layer (HAL). This includes configuration tasks on startup. Additionally, it does a lot of 'book keeping' taks, such as tracking worker process data object assignments, fully automates device state changes, and handling the network management state at the current scope of the project (it's easy to break with the addition of new features). 

A current limitation of the soft high level controller is that PDO communication is only possible once all workers have reached identical states and modes. The reason for this is that many of the performable actions (homing, and moving) complete at unique times for each worker, and all workers must recieve PDO during each message. This means that creating a truly generalized PDO solution must handle time sensitive input, sequence sensitive input, and wait/continue logic unique to physical robotic systems. The difficulty of this task in comparison to the new capabilities was too low to justify for the FCS and CSU for which this repository was intially designed.

## Hardware Abstraction Layer
The hardware abstraction layer is responsible for sending actual information over the NIC to the workers. Currently, the only HAL setup for this repository is the [`pysoemHAL`](src/HAL/pysoem/ethercat_bus.py). It uses [pysoem](https://github.com/bnjmnp/pysoem), a cython wrapper for [SOEM](https://github.com/OpenEtherCATsociety/SOEM). Something to keep in mind is that these are designed to be *simple* soft EtherCAT controllers, and as a result it is difficult to directly change the CoE messages being sent, which likely prevents using CoE to its full potential. Future HALs should be made with the same function names and send the same CoE telegrams (this can be verified with wireshark). An additional consequence is that the high level soft controller and [`pysoemHAL`](src/HAL/pysoem/ethercat_bus.py) have very similar function names.

# Notes
- Software tested on [EPOS4 Micro 24/5 EtherCAT](https://www.maxongroup.com/maxon/view/product/654731)
- Most of the development time was spent understanding the CiA 402 specification and maxon documentation. The repo could benefit from more polish or a refactor.
- Currently the only CiA 402 supported operation modes are Homing Mode and Profile Position Mode (non-continuous)
