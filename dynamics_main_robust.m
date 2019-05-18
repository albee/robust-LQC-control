%{
Mainfile for running robust optimization control of satellite dynamics.

Keenan Albee, Alex Steighner
May-18, 2019
%}

addpath(genpath('plotting'));
tf = 5;
dt = .1;

x_stored = [];

%% Set up dynamics
sys = Dynamics_3DoF();
x_stored = sys.x';

%% Loop through some inputs
for i = 0:dt:tf
%     uk = [1.0; 1.0; 1.0];
    uk = robust_rhc_socp(10, sys);  % swap out for robust_rhc_sdp( ) for sdp approach
    uk = uk(1:3);
    sys.FD_uncertain(uk, dt)  % uncertain dynamics
%     sys.FD_ideal(uk, dt)  % certain dynamics
    x_stored = [x_stored;
                sys.x' ];
end

x_stored;

plot([0:dt:tf],x_stored(2:end,1:3))
ax.FontSize = 16;
xlabel('Time [sec]','FontSize',16)
ylabel('Displacement [m]','FontSize',16)
legend('x-position','y-position','z-position','FontSize',16)

%% Animate result
r0_mat = [x_stored(:,1:2) zeros(size(x_stored, 1), 1)];
theta_mat = x_stored(:,3);
anim_FK3(r0_mat, theta_mat);
