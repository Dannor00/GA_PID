# PID Controller Optimization

## Overview
This project uses a Genetic Algorithm (GA) to optimize the parameters (Kp, Ki, Kd) of a PID controller for a third-order system. The GA optimizes the PID gains to minimize one of several cost functions related to system performance. This third-order system represents a typical industrial process with complex dynamics.

## Features
- Implements a real-valued GA using the DEAP library.
- Optimizes PID parameters based on different cost functions:
  - ITAE (Integral of Time-weighted Absolute Error)
  - ISE (Integral of Squared Error)
  - ITSE (Integral of Time-weighted Squared Error)
  - SSE (Steady-State Error)
  - Overshoot
  - Settling Time
- Compares GA-tuned PID with:
  - Ziegler-Nichols tuning
  - Manual tuning
- Plots step response comparisons.

## Installation
Ensure you have Python 3 installed, then install dependencies:
```bash
pip install numpy matplotlib deap control

Running the Code

Run the script with:

python ga_pid.py

Parameters

You can change the cost function in ga_pid.py:

cost_function_choice = 1  # Set to 1-6 (1=ITAE, 2=ISE, etc.)

Genetic Algorithm Configuration

    Population Size: 50
    Mutation Rate: 0.2
    Crossover Probability: 0.7
    Number of Generations: 30

Expected Output

    The best PID parameters found.
    Final step response for GA-optimized PID.
    Comparison of step responses (GA vs. Ziegler-Nichols vs. Manual tuning).

File Structure

code/
│── ga_pid.py   # Main Python script
│── README.md   # This file

References

    DEAP Library: https://deap.readthedocs.io/
    Python Control Systems Library: https://python-control.readthedocs.io/
    Robin T. Bye, "Optimising a PID Controller Using a Genetic Algorithm," 2012 (Used as inspiration for methodology and cost function selection).

License

This project is for educational purposes. Modify and use it freely!



## License
This project is for educational purposes. Modify and use it freely!

