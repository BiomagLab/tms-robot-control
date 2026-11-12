# tms-robot-control

Robotized transcranial magnetic stimulation controlled by InVesalius Navigator (https://github.com/invesalius/invesalius3)


# Configuration

- Copy `.env.example` to `.env`

- Edit `.env` and set the robot type, site, and other configuration parameters.

## Omron TM5X-700

The Omron driver uses two interfaces on the TMFlow controller:

- **Modbus TCP** (port 502) for the pose, TCP speed, project state and emergency state.
- **TM script socket** (port 5890) for linear, circular and dynamic motion commands.

On the robot, enable the Modbus slave and a listen node on port 5890. Then set:

```bash
SITE=mpi
ROBOT=omron
```

`SITE=mpi` applies the 180 degree rz offset used by the MPI CBS cell. For another
cell, add a matching entry to `SITE_CONFIG` in `robot/constants.py`.

The robot IP is sent by InVesalius when you press **Connect to robot**. Install
the dependencies with `pip install -r requirements.txt`; this includes
`pymodbus`. Force and pressure sensing are provided by the separate sensor
stack, not by the Omron driver.

# Running

The first step is to run the relay_server.py script with an argument to set the server port

`python relay_server.py 5000`

Next, execute the main_loop.py script. Note: the main_loop should be run using the **Python Console**

`python main_loop.py`

After that, run the InVesalius app.py script (https://github.com/invesalius/invesalius3) with the --remote-host argument, specifying the same port as the relay server

 `python app.py --remote-host http://localhost:5000`

Please note that every time InVesalius is started, the main_loop is automatically restarted.



# This work is supported by NeuroMat

The Research, Innovation and Dissemination Center for Neuromathematics (RIDC NeuroMat, or simply NeuroMat) is a Brazilian research center established in 2013 at the University of São Paulo that is dedicated to integrating mathematical modeling and theoretical neuroscience. Among the core missions of NeuroMat is the creation of a new mathematical system to understanding neural data and the development of neuroscientific open-source computational tools, keeping an active role under the context of open knowledge, open science, and scientific dissemination. The research center is  funded by the São Paulo Research Foundation (FAPESP). NeuroMat homepage: http://neuromat.numec.prp.usp.br
