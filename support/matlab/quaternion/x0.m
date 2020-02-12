%% Harbour scenery

%[vcoord,vconnect,vcol]=create_cube;
%[vcoord,vconnect,vcol]=create_sensor;
[vcoord,vconnect,vcol]=create_sea;


figure(1);
clf;
xlabel('X (to North)');
ylabel('Y (to West)');
zlabel('Z (to Sky)');
xlim([-8 8]);
ylim([-8 0]);
zlim([-8 8]);
title('Global coordinate system, right hand');
%view(3);
%view(145,40);
view(0,30);

h=draw_geom(vcoord,vconnect,vcol);

%bs2_read('COM7',@(q) gotquat(h,vcoord,vconnect,vcol,q));
%bs2_read('COM16',@(q) gotquat(h,vcoord,vconnect,vcol,q));
bs2_read_bt('btspp://000666868369',@(q) gotquat(h,vcoord,vconnect,vcol,q));

%t=@toto;


% q = [.1 .10 .30 .8];
% q=quatnormalize(q);
% 
% yaw = 0.7854; 
% pitch = 0.1; 
% roll = 0;
% q = angle2quat( yaw, pitch, roll );

% q=angle2quat(30/180*pi,0,0)
% 
% vcoord2 = rotate_geom(vcoord,q);
% 
% draw_geom(vcoord2,vconnect,vcol);



function gotquat(h,vcoord,vconnect,vcol,q)


    vcoord2 = rotate_geom(vcoord,q);
    update_geom(h,vcoord2,vconnect,vcol);
    
    drawnow limitrate nocallbacks
end
