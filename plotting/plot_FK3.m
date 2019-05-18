%{
Plots a cube with orientation on the given fig.
Inputs:
* r0 : 3-element vector that defines the cube origin
* R0: 3x3 rotation matrix that defines cube orientation relative to
inertial, R_IB
* l : cube side length
%}
function fig = plot_FK3(fig, r0, R0, erase)
    figure(fig)
    
    if( erase )
        cla(fig)
    end

%     COM_I = Center_of_Mass(self.r0, self.rL, self.robot);
%     r_I = [self.r0, self.rJ];
% 
%     scatter3(r_I(1,:),r_I(2,:),r_I(3,:),'black','filled');  % plot joints            
%     plot3(r_I(1,:),r_I(2,:),r_I(3,:),'black','Linewidth',2);  % plot links

    CLR = [0, 0, 1];
    ALPHA = .8;
    l = .5;
    plot_cube(r0, R0, l, CLR, ALPHA);

%     scatter3(COM_I(1,:),COM_I(2,:),COM_I(3,:),50,'bl','filled');  % plot COM
end