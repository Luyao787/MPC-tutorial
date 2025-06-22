# MPC Tutorial Class Guidance

## Overview

This guidance document provides a structured roadmap for the Model Predictive Control (MPC) tutorial class at TU Delft. The tutorial is organized into a progressive 5-week curriculum that builds from fundamental concepts to MPC applications.

## Week 1: Fundamentals - Discretization and MPC Formulation
**Files**: `week_01_integration.ipynb`, `week_01_mpc_formulation.ipynb`

### Part A: Numerical Integration Methods

#### Learning Objectives
- Understand discretization of continuous-time systems

- Compare numerical integration methods (Forward Euler vs. Runge–Kutta 4)

- Analyze stability and accuracy trade-offs

### Part B: MPC Formulation

#### Learning Objectives
- Master exact discretization of Linear Time-Invariant (LTI) systems

- Understand sparse vs. condensed MPC formulations

- Compare open-loop vs. closed-loop trajectories


## Week 2: LQR and MPC Formulation
**Files**: `week_02_LQR.ipynb`, `week_02_mpc_formulation.ipynb`

### Part A: LQR Theory (`week_02_LQR.ipynb`)

### Learning Objectives
- Learn to solve LQR problems using dynamic programming and matrix factorization techniques

- Understand the stability properties of finite-horizon optimal control laws

### Part B: MPC Formulation (`week_02_mpc_formulation.ipynb`)

### Learning Objectives
- Learn the condensed formulation of liner MPC with inequality constraints

- Understand potential numerical issues associated with the condensed formulation

## Week 3: Projection and Region of attraction
**File**: `week_03_projection.ipynb`

### Learning Objectives
- Compute feasible sets for constrained systems

- Understand Fourier-Motzkin elimination

- Compute the region of attraction for the MPC controller

## Week 4: Terminal Sets and Stability
**File**: `week_04_LQR_set.ipynb`

### Learning Objectives
- Design an appropriate terminal penalty

- Compute the LQR set

- Compute the region of attraction

## Week 5: Offset-free MPC and soft constraints
**Files**: `week_05_offset_free.ipynb`, `week_05_soft_constraints.ipynb`

### Part A: Offset-Free MPC (`week_05_offset_free.ipynb`)

#### Learning Objectives
- Handle model-plant mismatch and unknown disturbances

- Design state and disturbance observers

- Implement the target selector

### Part B: Soft Constraints (`week_05_soft_constraints.ipynb`)

#### Learning Objectives
- Address feasibility issues through constraint relaxation

- Implement ℓ₁-penalty methods
