This repo contains a dynamics simulator for a linear 3 DoF rigid body satellite, along with MOSEK Fusion implementations of 
robust optimization-based control. This approach, detailed in [Bertsimas et al., 2007], allows for increased stability at
the price of on-average larger cost functions. An SDP (slow) and SOCP (fast) receding horizon approach are implemented.

You must install the MOSEK Fusion MATLAB API as a prerequisite.

Code begins at `dynamics_main_robust.m`. Please see the included report for documentation.

![ ](/media/sample_socp.png)
