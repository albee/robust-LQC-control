%{
Calculates the robust optimal control over a given time horizon.
%}

function [u] = robust_rhc_socp(N, system)
    import mosek.fusion.*;
    
    % N time horizon DON'T USE MORE THAN LIKE 5
    n_u = 3;
    n_w = 6;
    n_x = 6;

%     system = Dynamics_3DoF();  % current system

    %% set up dynamics
    dynamics=struct('A',[],'B',[],'C',[],'Q',[],'R',[],'q',[],'r',[]);
    dynamics.A = system.A + eye(size(system.A,2));
    dynamics.B = system.B;
    dynamics.C = eye(size(system.A, 2));

    % we assume the weighting matrices never change--they generally don't
    dynamics.Q = eye(size(system.A, 2));  % assume identity
    dynamics.R = eye(size(system.B, 2));  % assume identity

    dynamics.q = zeros(size(system.A, 2), 1);  % assume 0, these are rarely used for actual control problems
    dynamics.r = zeros(size(system.B, 2), 1);  % assume 0, these are rarely used for actual control problems

    x0 = system.x;  % arbitrary initial state
    % x0 = zeros(6,1)
    % x0 = [2,2,2,2,2,2]'
    
    gamma = 1;  % this is the constant representing uncertainty set size. 0 should give LQR solution. gamma == omega

    %% generate SOCP constraint
    % This uses some of the intermediate variables used for the SDP
    % formulation in order to create SOCP constraints.
    % SOCP is equation (34) of Bertsimas et al.
    [constraint_matrix, A, B, C, D, a, b, c, h, F] = generateConstraints(dynamics, N, x0, gamma);
    PSD_size = size(constraint_matrix, 1);
    %% solve SOCP

    M = Model('SOCP_horizon');

    y = M.variable('y', Domain.inRotatedQCone(N*n_u + 2));  % L2 norm sqared, with RHS as scalar idx : SHOULD BE SQUARED
    t = M.variable('t', Domain.inQCone(N*n_w + 1));  % L2 norm on t <= y_tilde
    z = M.variable('z');  % unrestricted domain
    y_tilde = M.variable('y_tilde');  % unrestricted domain

    C_frob_2 = norm(C,'fro')^2;
    gamma_C_frob_2 = C_frob_2 * gamma^2;
    F_T = F';
    
    % constraint 1
    M.constraint( y.index([1]), ...
                  Domain.equalsTo(0.5));  % L2 norm squared on y <= RHS
    
    M.constraint( Expr.sub( y.index([2]) , Expr.sub( Expr.sub(z, Expr.mul(gamma, y_tilde)), gamma_C_frob_2)), ...
                  Domain.equalsTo(0.0));  % L2 norm squared on y <= RHS
              
    % constraint 2
    for j = 1 : N*n_w
        e_j = linspace(0,0, N*n_w);
        e_j(j) = 1;

        M.constraint( Expr.add( Expr.mul(2, Expr.mul( Expr.add(h, Expr.mul( F_T, y.slice([3], [N*n_u + 3]))), e_j) ), t.index([j]) ), ...
            Domain.greaterThan(0.0)); % -t[j] <= x i.e.    0 <= x + t[j]
              
        M.constraint( Expr.sub( Expr.mul(2, Expr.mul( Expr.add(h, Expr.mul( F_T, y.slice([3], [N*n_u + 3]))), e_j) ), t.index([j]) ), ...
            Domain.lessThan(0.0));  % x <= t[j] i.e.    x - t[j] <= 0
    end

    % constraint 3
    M.constraint( Expr.sub( t.index([1]), y_tilde ), ...
              Domain.equalsTo(0.0));  % L2 norm on t <= y_tilde
    
    M.objective(ObjectiveSense.Minimize, z);

    M.solve();

    y = y.level();
    y = y(3:N*n_u + 2);

    % convert to u
    u = B^(-1/2)*y - inv(B)*b;
end