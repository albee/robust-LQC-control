%{
A double integrator with rotation. Call FD updates to Euler integrate.
Keenan Albee, Alex Steighner
May-18, 2019
%}

classdef Dynamics_3DoF < handle
    properties
        % Real system model
        mass = 9.7;
        I_zz = 10.0;
        x;  % [x; y; theta; xdot; ydot; thetadot]
        % u = [F_x, F_y, T_z]
        A;
        B;
        C;
        w_mag;  % noise magnitude
        
    end
    methods
        % constructor
        function self = Dynamics_3DoF()
            self.x = [2; 2; 0.5; 0; 0; 0];  % initial condition
            self.A = [0 0 0 1 0 0;
                      0 0 0 0 1 0;
                      0 0 0 0 0 1;
                      zeros(3,6)];
            self.B = [zeros(3,3);
                      1/self.mass, 0, 0;
                      0, 1/self.mass, 0; 
                      0, 0, 1/self.I_zz];
            self.C = eye(6, 6);  % full observability
            self.w_mag = .05;
        end
        
        % Run the forward dynamics MODEL, DO update state
        function x_FD = FD_uncertain(self, uk, dt)     
            A = self.A; B = self.B; C = self.C; xk = self.x;

            x_FD = xk + (A*xk + B*uk)*dt + self.w_mag*randn(6,1);  % Euler integration with process noise
            self.x = x_FD;
        end

        function x_FD = FD_ideal(self, uk, dt)     
            A = self.A; B = self.B; C = self.C; xk = self.x;

            x_FD = xk + (A*xk + B*uk)*dt;  % Euler integration
            self.x = x_FD;
        end
        
    end
end