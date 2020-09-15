% Draw a geometry (sensor)

[vcoord,vconnect,vcol]=create_sensor;


figure(1);
clf;
xlabel('X (to North)');
ylabel('Y (to West)');
zlabel('Z (to Sky)');
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
title('Global coordinate system, right hand');
view(145,40);


h=draw_geom(vcoord,vconnect,vcol);

update_geom(h,vcoord,vconnect,vcol);
    
drawnow limitrate nocallbacks