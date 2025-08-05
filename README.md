# Description
This is the code repository for the paper [Navigating Robot Swarm Through a Virtual Tube with Flow-Adaptive Distribution Control](arXiv: https://doi.org/10.48550/arXiv.2501.11938).
# Structure
<pre> ``` Root Directory/ ├── Numerical Simulation/ # Implementation of numerical simulations in the paper │ ├── main/ # Main simulation scripts │ │ ├── main1.m # Numerical simulation script 1 │ │ └── main2.m # Numerical simulation script 2 │ └── clean_utils/ # Auxiliary function folder │ └── [Necessary utility functions] # Provide components for main1.m and main2.m │ └── Robotarium Experiment/ # Implementation of Robotarium experiments in the paper ├── experiment_run_code/ # Custom experiment code │ ├── experiment_run_main.m # Main experiment script (run directly to view simulation results) │ ├── Tube_Data.mat # Tube data storage │ ├── ExperimentData.mat # Experimental data storage │ └── [Other utility functions] # Auxiliary functions for experiments │ ├── [Other subfolders] # Components downloaded from Robotarium platform └── [Other utility functions] # Experiment-related necessary functions ``` </pre>
# Structure Description
This repository provides numerical simulations and Robotarium experiments in the paper, mainly divided into two core modules:
## Numerical Simulation Module
This module implements numerical simulations in the paper:
- **main subfolder**: Stores main simulation scripts
  - `main1.m`: Implements simulations based on the velocity command \(\mathbf{v}_i\), including 3 core functions:
    - Plot the position distribution of the robot swarm in the tube at the termination time;
    - Plot the curve of AMD (Average Minimum Distance) over time;
    - Plot the complete movement trajectory of the robots in the tube.
  - `main2.m`: Implements simulations based on the velocity command \(\mathbf{v}_i\), including 4 core functions:
    - Plot the change of the minimum distance among robots,
    - Plot the change of the minimum distance between robots and tube boundary;
    - Plot the convergence error curve between the current density \rho and the desired density \rho_d;
    - Plot the relative error \(\rho-\rho_d\) of the robot swarm in the occupied region.
- **clean_utils subfolder**: Defines necessary utility functions for numerical simulations, supporting for `main1.m` and `main2.m` to ensure modularity and reusability.
## Robotarium Experiment Module
This module implements the Robotarium physical experiments (or platform simulations) in the paper:
- **experiment_run_code subfolder**: Stores custom experiment code, as the core
  - `experiment_run_main.m`: Main experiment script, run directly to start the simulation on the Robotarium platform and output the experiment results;
  - Other files (e.g., `cal_lIndex.m`, `cal_rho_d_A.m`, etc.): Auxiliary functions required by `experiment_run_main.m`, implementing core logics such as position calculation, density estimation, and velocity control.
- **Other files and subfolders**: All are from the official Robotarium platform (https://www.robotarium.gatech.edu/), to ensure compatibility with the Robotarium platform.
## Summary
Through the code in this repository, all results of the numerical simulations and Robotarium experiments in the paper can be completely reproduced, verifying algorithm effectiveness in virtual and physical environments.
# Usage Instructions
## Running Numerical Simulation Scripts
To run `Numerical Simulation/main/main1.m` and `Numerical Simulation/main/main2.m`:
- **Step 1**: Add the `clean_utils` subfolder to the MATLAB search path. This can be done using the `pathtool` utility in MATLAB.
- **Step 2**: After adding the path, directly run `main1.m` or `main2.m` in MATLAB to execute the numerical simulations.
## Running Robotarium Experiment Script
To run `Robotarium Experiment/experiment_run_code/experiment_run_main.m`:
- **Step 1**: Add the entire `Robotarium Experiment` folder (including all its subfolders) to the MATLAB search path.
- **Step 2**: After adding the path, directly run `experiment_run_main.m` in MATLAB to launch the Robotarium simulation and view the experiment results.
## Note
Ensuring the correct folders are added to the MATLAB search path is critical for resolving dependencies and enabling smooth execution of all scripts.
