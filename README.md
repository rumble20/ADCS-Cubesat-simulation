# CubeSat ADCS: Estimation and Control

This repository contains a demonstrator project for an **Attitude Determination and Control System (ADCS)** applied to a CubeSat. The goal is to provide a learning and portfolio project that showcases skills in **dynamics modeling, control, and sensor fusion**, relevant for aerospace and defense applications.

---

## 📌 Features
- CubeSat rotational dynamics simulation (Euler’s equations)
- PD controller for attitude stabilization
- Simulated sensors: gyroscope, magnetometer, sun sensor
- Extended Kalman Filter (EKF) for sensor fusion and state estimation
- Visualization of results: angles, angular velocities, estimation errors

---

## 🛠 Installation
Clone the repository:
```bash
git clone https://github.com/yourusername/cubesat-adcs.git
cd cubesat-adcs
```

Install dependencies (Python 3.9+ recommended):
```bash
pip install numpy scipy matplotlib
```

---

## 🚀 Usage

### 1. Dynamics and Control Simulation
```bash
python adcs_simulation.py
```
Outputs:

- Attitude dynamics with external disturbances
- Stabilization using PD control
- Plots of true angles vs controlled response

### 2. Attitude Estimation with EKF
```bash
python ekf_estimation.py
```
Outputs:

- Simulated noisy sensor measurements
- EKF estimates of angle and angular velocity
- Comparison plots (true vs estimated states)

---

## 📂 Repository Structure
```
cubesat-adcs/
│
├── adcs_simulation.py      # CubeSat dynamics + PD controller
├── ekf_estimation.py       # EKF-based attitude estimation
│
├── plots/                  # Saved figures from simulations
├── docs/                   # Background notes and references
└── README.md
```

---

## 📊 Example Results

### PD Control
- CubeSat stabilizes to target attitude despite disturbances
- Response plotted as angle vs time

### EKF Estimation
- EKF successfully tracks true angle and angular velocity
- Estimation error decreases over time despite sensor noise

---

## 🗺 Roadmap
- Add advanced controllers (LQR, Sliding Mode)
- Include realistic perturbations (gravity gradient, aero, solar pressure)
- Full CubeSat mission simulation

---

## 🎯 Motivation
This project demonstrates practical skills in:

- Nonlinear dynamics modeling

- Real-time control laws

- Sensor fusion with Kalman filtering

- Documentation and reproducibility of simulations


It serves as a portfolio project for applications in **aerospace and defense industries**.

---

## 📖 References
- Markley & Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*
- Schaub & Junkins, *Analytical Mechanics of Space Systems*
- Simon, *Optimal State Estimation*

---

## 📬 Contact
Created by [Your Name].  
Feel free to connect via LinkedIn or open an issue for discussion!
