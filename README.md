# particle-filters-
# 🎯 Particle Filters: Nonparametric Bayesian Estimation

This project explores **Particle Filters** as an advanced method for nonparametric Bayesian estimation, with applications in **robot localization** and dynamic system state estimation under uncertainty. Unlike Gaussian-based filters (like Kalman filters), particle filters represent probability distributions using a finite set of random samples — or particles — making them especially powerful for nonlinear, non-Gaussian problems.

Developed as part of the *System Identification* course under Prof. Battilotti, this work implements particle filters, sequential importance sampling (SIS), and resampling strategies, with simulations demonstrating a mobile robot localization scenario.

---

## 📌 Project Objectives

- Understand and implement particle filters for dynamic systems.
- Apply **Sequential Importance Sampling (SIS)** and **Sequential Importance Resampling (SIR)**.
- Investigate issues such as **particle deprivation**, **resampling variance**, and **adaptive particle size (KLD-sampling)**.
- Simulate robot localization using particle filtering in a 2D environment.
- Study importance sampling and optimal proposal distributions.

---

## 🧩 Methodology

- **Problem Definition:**  
  Estimate the posterior probability of system states based on control inputs and noisy sensor measurements using a set of particles.

- **Techniques Explored:**
  - Importance Sampling (IS)
  - Sequential Importance Sampling (SIS)
  - Sequential Importance Resampling (SIR)
  - Adaptive Resampling
  - KLD-Sampling (adaptive number of particles based on Kullback-Leibler divergence)

- **Resampling Strategies:**
  - Low variance sampling
  - Adaptive resampling based on effective sample size
  - Regularization to maintain diversity in particle population

- **Simulation Case Study:**
  - Mobile robot localization with range sensor measurements.
  - Simulation with **2000 particles** initialized from a normal distribution.
  - State transition and measurement likelihood functions defined for realistic motion and sensing.

---

## 🚀 Simulation Summary

- **Scenario:** Point robot following a predefined path through a set of waypoints.
- **Map Building:** Range sensor readings synchronized with robot pose for map creation.
- **Particles:** Each particle contains `(x, y, θ)` representing robot's position and orientation.
- **Results:**  
  The particle filter successfully tracks the robot's path using simulated noisy sensor data, showcasing the effectiveness of nonparametric Bayesian filtering in robotics.

<img width="434" alt="image" src="https://github.com/user-attachments/assets/cf9f5b6b-12ae-4740-aa57-b926c5741af2" />


---
## References 
1. Particle Filters. Pieter Abbeel, UC Berkley EECS.
2. Probabilistic Robotics. Thrun S., Burgard W., Fox D. 1999-2000.
3. Short Introduction to Particle Filters and Monte Carlo Localization. Cryill Stachniss.
4. Real-time Particle Filters. Kwok C., Fox D., Meila M. University of Washington. 2004.
5. Importance Sampling and Particle Filters. Bagnell D., Bartels J.
6. Bayesian Approached to Localization, Mapping and SLAM. Choset H.
7. Particle Filtering. Sarkka S. 2012.
8. Particle Filters and Their Applications. Hsiao K., De Plinval-Salgues H., Miller J. 2005.
9. KLD-Sampling: Adaptive Particle Filters. Fox D. University of Washington.2002
10. Adapting the Sample Size in Particle Filters Through KLD-Sampling. Fox D. University of
Washington. 2003

---

## 📂 Project Structure

```bash
/Particle-Filters
│
├── particle_filter_simulation.m       # Main simulation script
├── motion_model.m                     # State transition function
├── measurement_likelihood.m           # Sensor model for particle weighting
├── resampling.m                       # Resampling algorithm implementation
├── README.md                          # Project documentation
└── PARTICLE FILTERS.pdf               # Detailed project report

