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
