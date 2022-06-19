# Full-and-Reduced-Order-Luenberger-Observer-Based-Feedback-Control-of-system-with-three-tanks
This post presents implementation of Full and Reduced Order Linear Observer using state-feedback controller with integral action.
Firstly,it is essential to drive dynamical equations that govern the height in each tank, working out some math and using physical laws, we derived three nonlinear
differential equations,then we linearized them at a prechosen equilibrium points.The numerical parameters of the dynamical matrix(state matrix) were directly given.
We are interested in the height of the third tank so the output equation is the third height. The objective is to maintain this height at a specific level through a valve that feeds the first tank even in the presence of external disturbances that are assumed to be non-zero constant in this project.
Herein we were able to design a state feedback controller after proving that the system is controllable.As state-feedback control is not robust against steady state error
We designed so a feedfoarward gain to ensure a zero steady state error.Feedforward compensator is though computed only if the state matrices are assumed to be known and constant.
The link below gives more details about the system and its equations.However, the values of physical parameters have not been given but only state matrices.

https://hal.archives-ouvertes.fr/hal-00631726/document
