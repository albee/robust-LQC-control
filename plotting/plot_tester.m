%--- Set up plotting ---%
fig = figure('units','normalized','outerposition',[0 0 1 1]);
grid on
view(3);
hold on
axis equal
axis([-5, 5, -5, 5, -5, 5]*1)

r0 = [0 0 0];
eul = [0.707 0.707 0.707];
rotmXYZ = eul2rotm(eul, 'XYZ');
plot_FK3(fig, r0, rotmXYZ, 1)  % R0 is the body frame relative to inertial, R_IB

% v = VideoWriter('test2.avi');v.FrameRate=15;open(v);writeVideo(v,frame);close(v);