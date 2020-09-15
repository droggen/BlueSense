%% example_sensoranim
% Animates a sensor as the node moves.
% The line "bs2_read" must be modified either as:
% bs2_read('COMx'....
% or
% bs2_read_bt('btspp://0006668C409E',...
% or 
% bs2_read_bt('BlueSense-409E',...

if exist('btaddress','var')==0
    error('The variable btaddress must be defined. E.g. btaddress="btspp://000666868369"');
end


%% Sensor scenery

[vcoord,vconnect,vcol]=create_sensor;



qdef=NaN;               % Default
%qdef=[-cos(45/180*pi) 0 0 sin(45/180*pi)];        % Rotate so that bluetooth points to north in the "zero" position
%qdef=[+cos(45/180*pi) 0 0 sin(45/180*pi)];        % Rotate so that push button points to north in the "zero" position
%qdef=[cos(45/180*pi) sin(45/180*pi) 0 0];        % Rotate so that bluetooth points to sky in the "zero" position




%% Initialise view
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


%% Read loop 
bs2_read('COM9',@(q) gotquat(h,vcoord,vconnect,vcol,q,qdef));
%bs2_read_bt(btspp://0006668C409E,@(q) gotquat(h,vcoord,vconnect,vcol,q,qdef));
%bs2_read_bt('BlueSense-8367',@(q) gotquat(h,vcoord,vconnect,vcol,q,qdef));

%% Read callback
function gotquat(h,vcoord,vconnect,vcol,q,qdef)
    

    vcoord2 = vcoord;
    
    if 0
        if isnan(qdef)
        else
            q=quatmultiply(quatconj(qdef),quatconj(q));
        end
    else
        % Rotate the model.
        if isnan(qdef)
        else        
            vcoord2 = rotate_geom(vcoord,quatconj(qdef));
        end
        % Convert quaterion from QG<-S into QS<-G
        q=quatconj(q);
    end

    vcoord2 = rotate_geom(vcoord2,q);
    update_geom(h,vcoord2,vconnect,vcol);
    
    drawnow limitrate nocallbacks
end
