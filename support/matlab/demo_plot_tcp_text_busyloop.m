%% BS2 Matlab demo: read data from TCP (text format) with busy loop
% This code shows how to get the data from BlueSense over a tcp port
% with a busy loop. It assumes BlueSense data is translated from USB/BT to
% TCP via a companion software such as SensHub or har_relay.

%
% Make sure to adapt the com port to the actual com port of the device.

port = 1001;     % <- Adapt this

%% Ensures any connection to serial devices are terminated
if exist('ser','var')
    fclose(ser);
    clear ser
end

%% Alternate way to ensure connections to serial devices are terminated
% if exist('ser','var')
%     fclose(ser);
%     delete(ser);
%     clear ser;
% end


%% Open connection to the device
% Note: on some computers these functions can take several minutes to
% complete. This seems to be related to the number of com ports on the
% computer, and appears to be an inefficiency of Matlab
ser=tcpip('localhost',port);
fopen(ser)

%% Create a plot
% Setup data structures to store the n most recent data samples
% n: number of points to keep
n=1000;
% datax: timstamp of the sample (here 0 is the most recent, -1 is the previous, etc)
datax=[0-n+1:1:0];
% datay: sample value
datay=zeros(1,n);

% Create new figure
figure(1);
clf;
plot(datax,datay);


%% Continuous loop reading data
it=0;
totread=0;
tic;
while 1
    % Read one line
    str = fgetl(ser);
    
    % Convert data to numbers
    data=str2num(str);
    
    % Potentially the numeric data is empty (e.g. if we read text instead
    % of a number). In this case we skip the plotting.
    if isempty(data)
        continue;
    end
    
    
    it=it+1;
    t = toc;
    totread=totread+1;
    
    % Add data to datay
    datay=[datay data];
    
    % As matlab is too slow if reading sample by sample we minimise the
    % update rate.
    % Minimise computational load - update only every X samples
    if mod(totread,3000)~=0
        continue;
    end
    
    fprintf(1,'Iteration %u. Tot red %u in %f. Sps: %f\n',it,totread,t,totread/t);
    
    % Keeping only the n-most recent samples
    datay=datay(1,end-n+1:end);
    
    
    % Plot the data
    plot(datax,datay);
    drawnow limitrate nocallbacks
       
    
    
end
