%{
This function generates the giant constraint matrix in eq 17 on pg 1830 of
the Robust Optimization paper: Constrained Stochastic LQC: A Tractable
Approach.

Inputs:
struct with N states, each state including an A estimation, B
estimation, C estimation, Q, q, R, and r.  Reference pg 1830 of paper for
detailed explaination.
N is the number of states available (time horizon)
x0 is the initial state
gamma is the constant representing the noise

Keenan Albee, Alex Steighner
May-18, 2019
%}

function [constraint_matrix, A, B, C, D, a, b, c, h, F] = generateConstraints(dynamics, N, x0, gamma) 
    n_u = 3;
    n_w = 6;
    n_x = 6;
    
    a = zeros(n_x, 1);
    b = zeros(N*n_u, 1);
    c = zeros(N*n_w, 1);
    
    A = zeros(n_x, n_x);
    B = zeros(N*n_u, N*n_u);
    C = zeros(N*n_w, N*n_w);
    D = zeros(N*n_u, N*n_w);
    
    for k = 1:N
        A_til = dynamics.A;  % A0
        B_til = [dynamics.B, zeros(6, (N-1)*n_u)];  % [B0, ...]
        C_til = [dynamics.C, zeros(6, (N-1)*n_w)];  % [C0, ...]
        
        % compute state transition matrices at this time horizon
        for i = 1:k-1
            A_til = A_til*dynamics.A;
            
            B_til(:, n_u+1:n_u*(i+1)) = B_til(:, 1:n_u*i);  % shift over
            B_til(:, 1:n_u) = dynamics.A^i*dynamics.B;  % add latest
            
            C_til(:, n_w+1:n_w*(i+1)) = C_til(:, 1:n_w*i);  % shift over
            C_til(:, 1:n_w) = dynamics.A^i*dynamics.C;  % add latest
        end
        % tilde terms have been evaluated at their k-1 values
        
%         A_til
%         B_til
%         C_til
        
        % find a
        a = a + A_til'*dynamics.q;
        
        %find A
        A = A + A_til'*dynamics.Q*A_til;
        
        % find b
        b = b + (B_til'*dynamics.Q*A_til); % dynamics.q and dynamics.r are 0 so we ignore

        % find B
        B = B + B_til'*dynamics.Q*B_til;
        
        % find c
        c = c + (C_til'*dynamics.Q*A_til);  % dynamics.q and dynamics.r are 0 so we ignore
        
        % find C
        C = C + C_til'*dynamics.Q*C_til;
        
        % find D
        D = D + B_til'*dynamics.Q*C_til;
    end  
%         

    % Calculate Rhat
    Rhat = zeros(n_u*N, n_u*N);
    for k = 1:N
        Rhat((k-1)*n_u +1:(k)*n_u, (k-1)*n_u +1:(k)*n_u) = dynamics.R;
    end
    B = B + Rhat;
    b = b*x0;
    c = c*x0;
    
    % if we decide to add r weighting
%     for k=1:N-1
%         r_hat = [r_hat transpose(dynamics(k).r)]
%     end
%     b = rhat' + b;

    h = c - D'*inv(B)*b;
    F = B^(-1/2)*D;
    
    %y,z, and lambda are decision variables for the optimization
%     y = sym('y',[N*n_u,1]);
%     z = sym('z');
%     lambda = sym('lambda');

    % these are placeholder and aren't actually used
    y = ones(N*n_u, 1)*7;
    z = 7;
    lambda = 0;  % this MUST be zero, lambda is set later

    %Use Eq 17 formula11
    constraint_matrix=[eye(N*n_u), y, F;
                       y', z - gamma^2*lambda, -h';
                       F', -h, lambda*eye(N*n_w) - C + F'*F];
end
