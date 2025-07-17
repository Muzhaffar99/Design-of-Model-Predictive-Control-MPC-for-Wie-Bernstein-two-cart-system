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

- **Robustness to time constant variations** in the demand response, achieved via adjustments to the **terminal cost matrix**

## Region of Attraction (RoA)

The **Region of Attraction** refers to the set of initial states from which the LQ-MPC controller guarantees convergence to the desired equilibrium under the given constraints.

- It represents **asymptotic stability** under feedback control.
- Determined by:
- The solution to the Riccati equation
- The terminal cost matrix
- The linearized system dynamics

**Significance**:  
Staying within the RoA ensures that the system will return to its equilibrium state without violating any constraints.

---

## Region of Feasibility (RoF)

The **Region of Feasibility** is the set of initial conditions for which the MPC optimization problem has a feasible solution that satisfies **all input and state constraints** over the prediction horizon.

- RoF ⊇ RoA in general  
- Related to:
- State/input constraints
- Horizon length
- System uncertainty
- Terminal constraints

**Key Insight**:  
Being inside the RoF ensures a solution exists. Being inside the RoA ensures **stable convergence** as well.

---

## Visualization

Below is a conceptual illustration of the system response and constraint satisfaction for demand and control inputs:

![System Behavior with Constraints](assets/927b570a-1315-4ed2-a0e3-e8d1446b331a.png)

*Figure: The controller limits the applied force `uₖ` within ±1 and keeps demand states `x₃`, `x₄` within ±0.5. The region of attraction lies strictly inside the region of feasibility.*

---


## Contributions

- Developed a refined LQ-MPC scheme tailored for uncertain two-mass spring dynamics
- Analyzed and mitigated the impact of demand time constant variations
- Maintained performance within strict constraints with rapid stabilization (≤10 steps)



