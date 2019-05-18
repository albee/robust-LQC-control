%{
Calculates the robust optimal control over a given time horizon.
%}

function [u] = robust_rhc(N, system)
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
    gamma = 10;  % this is the constant representing uncertainty set size. 0 should give LQR solution

    %% generate SDP constraint
    %this calls the function to generate the big constraint matrix from equation
    %17 on pg 1830, and outputs the constants for future use as well.
    [constraint_matrix, A, B, C, D, a, b, c, h, F] = generateConstraints(dynamics, N, x0, gamma);
    PSD_size = size(constraint_matrix, 1);
    %% solve SDP
    M = Model('SDO_horizon');

    PSD_mat = M.variable('PSD_mat', Domain.inPSDCone(PSD_size));  % PSD
    lambda = M.variable('lambda', Domain.greaterThan(0.));  % > 0
    z = M.variable('z');  % unrestricted domain

    %% specify components of the PSD matrix
    % block_11
    for i = 1:N*n_u
        for j = 1:N*n_u
            PSD_var = PSD_mat.index([i,j]);
            M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD var
        end
    end

    % block_12
    %%% y is unconstrained: no action needed

    % block_13
    for i = 1:N*n_u 
        for j = N*n_u+2 : N*n_u+1 + N*n_w
            PSD_var = PSD_mat.index([i,j]);
            M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD var
        end
    end

    % block_21
    %%% y is unconstrained: no action needed

    % block_22
    i = N*n_u + 1;
    j = N*n_u + 1;
    PSD_var = PSD_mat.index([i,j]);
    constraint = Expr.sub(z, Expr.mul(gamma^2, lambda));
                      % index being set         actual expression you want
    M.constraint(Expr.sub( PSD_var, constraint), Domain.equalsTo( 0.0 ));  % set [i,j] of the PSD var

    % block_23
    i = N*n_u + 1;
    for j = N*n_u + 2 : N*n_u+1 + N*n_w
        PSD_var = PSD_mat.index([i,j]);
        M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD va
    end

    % block_31
    for i = N*n_u+2 : N*n_u+1 + N*n_w
        for j = 1:N*n_u
            PSD_var = PSD_mat.index([i,j]);
            M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD var
        end
    end

    % block_32
    for i = N*n_u + 2 : N*n_u+1 + N*n_w
        j = N*n_u + 1;
        PSD_var = PSD_mat.index([i,j]);
        M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD var
    end

    % block_33
    for i = N*n_u + 2 : N*n_u+1 + N*n_w
        for j = N*n_u + 2 : N*n_u+1 + N*n_w
            PSD_var = PSD_mat.index([i,j]);
            if i==j
                constraint = Expr.add(lambda, constraint_matrix(i, j));
                M.constraint(Expr.sub( PSD_var, constraint), Domain.equalsTo( 0.0 ));  % set [i,j] of the PSD var
            else
                M.constraint( PSD_var, Domain.equalsTo(constraint_matrix(i, j)) );  % set [i,j] of the PSD var
            end
        end
    end

    M.objective(ObjectiveSense.Minimize, z);

    M.solve();

    PSD_answer = PSD_mat.level();

    % disp('sanity check')
    % z.level() - gamma^2 * lambda.level()
    % PSD_mat.index([N*n_u + 1, N*n_u + 1]).level()

    % get y
    y = PSD_answer(PSD_size*N*n_u+1 : PSD_size*N*n_u + N*n_u);  % is this the correct slice?

    % convert to u
    u = B^(-1/2)*y - inv(B)*b;
end