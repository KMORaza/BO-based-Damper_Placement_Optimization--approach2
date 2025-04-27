## Optimale Dämpferplatzierung in der mechanischen Fahrzeugaufhängung, entwickelt mittels Bayesscher Optimierung (Optimal damper placement in automotive mechanical suspension implemented using Bayesian Optimization)

A multi-objective solution that utilizes Bayesian Optimization (BO) to optimize damper placement and parameters in an automotive suspension system. 

The solution is to minimize three objectives—comfort (ISO 2631), vibration (frequency-weighted PSD), and handling (tire load variation)—across different road profiles (urban, highway, off-road) and damping profiles (linear, digressive, progressive).

* Defines fixed vehicle parameters (e.g., sprung mass = 1500 kg, unsprung mass = 50 kg per wheel, tire stiffness = 200000 N/m).
* Implements a Gaussian Process (GP) regression model & uses a Radial Basis Function (RBF) kernel
* Supports incremental updates to the kernel matrix using the Sherman-Morrison formula for efficiency.
* Provision for mean predictions and for uncertainty estimates, crucial for Bayesian optimization’s exploration-exploitation trade-off.
* Uses three surrogate ensemble instances (one per objective: comfort, vibration, handling) to model the objective function.
* Initializes with 30 random points, then iteratively selects new points using Simulated Annealing to maximize an active learning score combining Expected Hypervolume Improvement (EHVI), variance, and diversity.
* Maintains Pareto front to store non-dominated solutions, with adaptive weights based on crowding entropy to balance objectives.
* Uses EHVI to balance comfort, vibration, and handling, ensuring a diverse Pareto front.
* The ensemble of GP, RF, and NN provides robust predictions and uncertainty estimates, critical for Bayesian optimization.
* Incremental updates (e.g., Sherman-Morrison in GP) improve efficiency as new points are added.
* Parallel evaluation of objectives reduces runtime for expensive simulations.
* Simulated Annealing optimizes the acquisition function, balancing exploration and exploitation.
* Handles invalid objectives (NaN, infinite, or excessively large) by retrying with perturbed parameters or assigning high penalties.

---

Check [__*webpage*__](https://bo-optimal-damper-placement-2.netlify.app/) about the solution.
