%{
Takes in a matrix of r0: [n, 3], 
and a matrix of q: [n, 4]
and animates. Time is the column.

Keenan Albee, 4-25-19
%}
function anim_FK3(r0_mat, R0_mat)
    %% Set up animation
    type = 'collage';  % set to 'collage' or 'default'
    fig = figure('units','normalized','outerposition',[0 0 1 1]);
    grid on
    view(3);
    hold on
    axis equal
    axis([-5, 5, -5, 5, -5, 5]*1)

    if size(r0_mat,2) == 2  % 2D
        r0_mat = [r0_mat, zeros(size(r0_mat,1),1)];
    elseif size(r0_mat,2) == 3 % 3D
    end
    
    %% Animate
    switch(type)
        case( 'default' )
            for i=1:1:size(r0_mat,1)
                disp(i);

                r0 = r0_mat(i,:)';

                if size(R0_mat,2) == 1  % assume R0_mat is just 1 angle for now
                    eul = [0.0 0.0 R0_mat(i)];
                    R0 = eul2rotm(eul, 'XYZ');
                elseif size(R0_mat,2) == 4  % quaternion, convert to R0
                    q = R0_mat;
                    R0 = quat2rotm([q(i,4), q(i,1), q(i,2), q(i,3)]);
                end

                % plot
                erase = 1;
                fig = plot_FK3(fig, r0, R0, erase);
                frame(i)=getframe(fig);
            end
                
        case( 'collage' )
            for i=1:1:size(r0_mat,1)
                if mod(i,5) == 0
                    disp(i);

                    r0 = r0_mat(i,:)';

                    % assume R0_mat is just 1 angle for now
                    if size(R0_mat,2) == 1
                        eul = [0.0 0.0 R0_mat(i)];
                        R0 = eul2rotm(eul, 'XYZ')';
                    elseif size(R0_mat,2) == 4  % quaternion, convert to R0
                        q = R0_mat;
                        R0 = quat2rotm([q(i,4), q(i,1), q(i,2), q(i,3)]);
                    end

                    % plot
                    erase = 0;
                    fig = plot_FK3(fig, r0, R0, erase);
                    frame(i)=getframe(fig);
                end
            end
    end

    
    % Make video
%     v = VideoWriter('blargh11.avi');
%     v.FrameRate=15;
%     open(v);
%     writeVideo(v,frame);
%     close(v);
end