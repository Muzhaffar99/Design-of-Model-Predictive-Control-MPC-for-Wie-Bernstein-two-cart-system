# LQ-MPC for Two-Mass Spring Systems with Uncertain Parameters

## Overview

This project implements a **refined Linear Quadratic Model Predictive Controller (LQ-MPC)** for stabilizing a **two-mass spring system** with uncertain parameters. Such systems are important analogs for real-world applications including:

- Coupled-mass systems  
- Power grids  
- Spacecraft docking  
- Interconnected mechanical and electrical systems

These applications require **precise control of interconnected components**, often under uncertainty.

## Objective

The controller is designed to:

- Stabilize the applied force `uₖ`
- Accommodate **demand response** within **specified state and input constraints**
- Ensure **fast and stable system response** (under 10 time steps)

## Features

- **Force control within bounds:**
-1 ≤ uₖ ≤ 1

- **Demand response constraints:**  
|x₃| ≤ 0.5
|x₄| ≤ 0.5

markdown
Copy
Edit

- **Robustness to time constant variations** in the demand response, achieved via adjustments to the **terminal cost matrix**

## Contributions

- Developed a refined LQ-MPC scheme tailored for uncertain two-mass spring dynamics
- Analyzed and mitigated the impact of demand time constant variations
- Maintained performance within strict constraints with rapid stabilization (≤10 steps)

## Applications

This approach is applicable in:

- Smart grid demand-side management
- Precision robotics and actuators
- Aerospace docking maneuvers
- Industrial automation and mechatronics

## Contact

For questions, contributions, or collaborations, feel free to open an issue or reach out via the repository discussion board.

---

