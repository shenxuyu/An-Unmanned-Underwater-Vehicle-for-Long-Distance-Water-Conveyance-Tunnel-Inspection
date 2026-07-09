# An Unmanned Underwater Vehicle for Long-Distance Water Conveyance Tunnel Inspection

This repository contains the source code associated with the manuscript **"An Unmanned Underwater Vehicle for Long-Distance Water Conveyance Tunnel Inspection"**.

The code is organized into two main parts:

1. **CMBS marker recognition** for marker-triggered absolute position correction.
2. **UUV heading planning and control**, including bilateral ranging-sonar heading planning, PID controllers, fuzzy-PID variants, depth control, thrust allocation, Gaussian-process residual modelling and RSGP-MPC heading control.

The code was developed for an experimental unmanned underwater vehicle (UUV) platform and was used to support the tunnel-inspection experiments described in the manuscript.

---

## Repository structure

```text
.
├── Code_for_marker_recognition/
│   ├── image_reciever.py
│   ├── inference.py
│   ├── Repeat_Timer.py
│   └── sonar_identify.py
│
├── auv_control_paper_code_modified/
│   └── auv_control(6.4)/
│       ├── auv_control.py
│       ├── FullCourseMPC.py
│       ├── FullCourseMPC_OP.py
│       ├── MPC_solver.py
│       ├── my_gp_class.py
│       ├── model.py
│       ├── course_pid.py
│       ├── course_pid_with_fuzzy.py
│       ├── full_depth_pid.py
│       ├── under_depth_pid.py
│       ├── under_depth_pid_with_fuzzy.py
│       ├── under_depth_backstep.py
│       ├── pitch_pid.py
│       ├── roll_pid.py
│       ├── fuzzy_ctl.py
│       ├── thruster_distribute.py
│       ├── Modbus.py
│       ├── data_process.py
│       ├── data_save.py
│       ├── Repeat_Timer.py
│       ├── Fr.npy
│       ├── r.npy
│       ├── Y.npy
│       └── test*.py
└── README.md
```

The folder name may differ slightly after downloading or unzipping the repository.

---

## 1. CMBS marker-recognition code

Folder:

```text
Code_for_marker_recognition/
```

This module implements marker recognition from circumferential multibeam sonar (CMBS) intensity frames. The marker-recognition result is used to trigger absolute position correction in the navigation filter.

### Main files

| File | Description |
|---|---|
| `image_reciever.py` | Receives sonar image frames through a TCP socket, decodes the binary frame format, converts the floating-point CMBS intensity frame to a pseudo-colour image, and stores recent frames in a queue. |
| `sonar_identify.py` | Runs periodic marker recognition using a trained ResNet-18 model and sends the recognition result to the upper computer through UDP. |
| `inference.py` | Standalone inference script for testing the trained marker-recognition model. |
| `Repeat_Timer.py` | Utility for repeated timed execution. |

### CMBS frame preprocessing

The preprocessing pipeline implemented in `image_reciever.py` and `sonar_identify.py` includes:

1. Receiving a square floating-point CMBS intensity frame.
2. Clipping saturated acoustic returns above `4.0 × 10^6`.
3. Min-max normalising the frame to `[0, 1]`.
4. Converting the frame to an 8-bit image.
5. Applying the JET pseudo-colour map.
6. Resizing the image to `480 × 480` pixels.
7. Converting from BGR to RGB.
8. Scaling the image to `[0, 1]` and shifting by `-0.5`.
9. Passing the resulting tensor to a ResNet-18 binary classifier.

### Notes

The marker-recognition scripts expect a custom model definition and trained weights. In the current code, these are referenced as:

```text
resnet18.py
Sonar_identify/trained_model/ACC_97.39.pth
1/trained_model/ACC_97.39.pth
```

If these files are not included in the repository, place the trained model definition and weight file at the paths expected by the scripts, or modify the paths before running.

The default network settings used by the marker-recognition code include:

```text
TCP image receiver: 127.0.0.1:8765
UDP local address: 192.168.1.5:6666
UDP target address: 192.168.1.5:6665
```

These addresses should be modified to match the local sonar computer and upper-computer configuration.

---

## 2. UUV heading-planning and control code

Folder:

```text
auv_control_paper_code_modified/auv_control(6.4)/
```

This module contains the main UUV control code used for heading planning, heading control, depth control, thrust allocation and hardware communication.

### Main files

| File | Description |
|---|---|
| `auv_control.py` | Main UUV control program. It manages UDP communication, sensor data reception, control-mode selection, heading/depth control, autonomous heading planning, thrust allocation and command transmission. |
| `FullCourseMPC.py` | RSGP-MPC heading controller used in the tunnel-inspection study. It combines a nominal yaw-rate model, Gaussian-process residual learning and MPC optimization. |
| `FullCourseMPC_OP.py` | Alternative or earlier MPC implementation. |
| `MPC_solver.py` | MPC solver construction utilities based on CasADi. |
| `my_gp_class.py` | Gaussian-process residual model. The GP uses a squared-exponential ARD kernel and exports a CasADi-compatible prediction function for MPC. |
| `model.py` | Dynamic model and model-analysis utilities. |
| `course_pid.py` | Heading PID controller. |
| `course_pid_with_fuzzy.py` | Fuzzy-PID heading controller. |
| `full_depth_pid.py` | Fully actuated depth PID controller. |
| `under_depth_pid.py` | Underactuated depth PID controller. |
| `under_depth_pid_with_fuzzy.py` | Fuzzy-PID variant of the underactuated depth controller. |
| `under_depth_backstep.py` | Backstepping-based underactuated depth controller. |
| `pitch_pid.py` | Pitch PID controller. |
| `roll_pid.py` | Roll PID controller. |
| `fuzzy_ctl.py` | Fuzzy-control membership functions and rule base. |
| `thruster_distribute.py` | Thruster allocation from desired force/moment commands to individual thruster speeds. |
| `Modbus.py` | Modbus encoding and decoding utilities. |
| `data_process.py` | Processing utilities for generating training data for GP residual learning. |
| `data_save.py` | Runtime data-saving utilities. |
| `Fr.npy`, `r.npy`, `Y.npy` | Data arrays used for GP-related testing or training. |

### Heading-planning and control workflow

The overall control workflow is:

1. Receive sensor data from the UUV navigation system, DVL, ranging sonars and other onboard modules.
2. Estimate the UUV deviation from the tunnel centreline using bilateral ranging-sonar measurements.
3. Compute the desired heading angle for near-centreline navigation.
4. Apply heading control using PID, fuzzy-PID, MPC or RSGP-MPC.
5. Allocate the desired force and moment commands to the thrusters.
6. Send actuator commands to the main control board.
7. Save experimental data for post-processing and controller evaluation.

### RSGP-MPC component

The RSGP-MPC implementation is mainly contained in:

```text
FullCourseMPC.py
my_gp_class.py
MPC_solver.py
```

The controller uses:

- a nominal yaw-rate dynamic model,
- a Gaussian-process residual model to learn model uncertainty,
- recursive or online GP-data updating,
- CasADi-based nonlinear optimization,
- a model predictive control objective for heading tracking.

The data files `r.npy`, `Fr.npy` and `Y.npy` are used by the GP-related testing scripts. The script `test.py` demonstrates loading these arrays and initializing the GP model.

---

## Installation

The code was developed in Python. A typical environment can be prepared with:

```bash
conda create -n uuv-tunnel python=3.8
conda activate uuv-tunnel
```

Install the common dependencies:

```bash
pip install numpy scipy pandas matplotlib opencv-python torch torchvision casadi scikit-fuzzy
```

Depending on how the code is executed, additional packages or local interfaces may be required. The full hardware-control code also depends on the local network configuration and the UUV's onboard communication protocol.

---

## Running the code

### Marker recognition

```bash
cd Code_for_marker_recognition
python sonar_identify.py
```

Before running, check and modify:

- the path to `resnet18.py`,
- the path to the trained model weights,
- the TCP image-receiver address and port,
- the UDP local and target addresses,
- whether CUDA or CPU inference should be used.

For standalone testing, modify `inference.py` to load a local image folder or queue source and then run:

```bash
python inference.py
```

### GP model test

```bash
cd auv_control_paper_code_modified/auv_control(6.4)
python test.py
```

This script loads `r.npy`, `Fr.npy` and `Y.npy`, constructs the GP training input and initializes the Gaussian-process model.

### UUV control program

```bash
cd auv_control_paper_code_modified/auv_control(6.4)
python auv_control.py
```

Important: `auv_control.py` is a hardware-facing control program. It opens UDP sockets, receives sensor data and sends commands to the UUV control system. Do not run it on an active vehicle or connected network unless the IP addresses, ports, safety logic and actuator-output commands have been checked.

Default addresses in `auv_control.py` include:

```text
Local main-board socket: 192.168.1.4:20000
Local upper-computer socket: 192.168.1.4:8887
Local sonar socket: 192.168.1.4:8848
Upper-computer address: 192.168.1.5:8886
Main-board address: 192.168.1.6:20000
Sonar address: 192.168.1.7:8321
```

Modify these addresses before running the code on another system.

---

## Data and model availability

This repository provides the source code for the algorithmic components described in the manuscript. Some large data files, trained neural-network weights, engineering blueprints, site-specific infrastructure records and operational tunnel data may be restricted or stored separately because of infrastructure-operator access requirements.

Please refer to the associated manuscript, supplementary data and data-availability statement for details on source data and access conditions.

---

## Reproducibility notes

- The code contains research and experimental-control scripts developed for a specific UUV platform.
- Several scripts require local hardware, sonar data streams or network communication to run as originally used.
- Paths, IP addresses, ports and model-weight locations should be checked before execution.
- The marker-recognition model uses a trained ResNet-18 classifier; the weight file must be supplied separately if it is not included in the repository.
- The RSGP-MPC implementation uses CasADi and GP residual learning; solver behaviour may depend on the Python environment and installed CasADi/IPOPT version.


---

## License

Please add a `LICENSE` file to specify the terms under which the code can be used, modified and redistributed. For academic open-source release, common choices include MIT, BSD-3-Clause and Apache-2.0.
