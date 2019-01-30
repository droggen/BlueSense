%% BS2 Matlab demo: read data with busy loop
% This code shows how to get the data from BlueSense over a com port
% with a busy loop.
%
% Make sure to adapt the com port to the actual com port of the device.

port = 'COM47';     % <- Adapt this

%% Ensures any connection to serial devices are terminated
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
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
ser=serial(port);
fopen(ser);

%% Create a plot
% Setup data structures to store the n most recent data samples
% n: number of points to keep
n=100;
% datax: timstamp of the sample (here 0 is the most recent, -1 is the previous, etc)
datax=[0-n+1:1:0];
% datay: sample value
datay=zeros(1,n);

% Create new figure
figure(1);
clf;
plot(datax,datay);


%% Continuous loop reading data
while 1
    % Read one line
    str = fgetl(ser);
    
    % Convert data to numbers
    data=str2num(str);
    
    % Potentially the numeric data is empty (e.g. if we read text instead
    % of a number). In this case we skip the plotting.
    if isempty(datay)
        continue;
    end
    
    % Add 1st channel data to datay, keeping only the n-most recent samples
    datay=[datay(2:end) data(1)];
    
    % Plot the data
    plot(datax,datay);
    drawnow limitrate nocallbacks
       
    
    
end
