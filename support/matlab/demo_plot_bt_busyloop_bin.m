%% BS2 Matlab demo: read data with busy loop
% This code shows how to get the data from BlueSense over a com port
% with a busy loop.
%
% This is suitable for the 'a' fast ADC acquisition mode, where only one
% channel is streamed in 8-bit binary.
%

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

% Make sure to adapt the com port to the actual com port of the device.
port = 'btspp://0006668C400E';  % <- Adapt this





%% Open connection to the device
% Note: on some computers these functions can take several minutes to
% complete. This seems to be related to the number of com ports on the
% computer, and appears to be an inefficiency of Matlab
ser=Bluetooth(port,1);
fopen(ser);

%% Create a plot
% Setup data structures to store the n most recent data samples
% n: number of points to keep
n=10000;
% datax: timstamp of the sample (here 0 is the most recent, -1 is the previous, etc)
datax=[0-n+1:1:0];
% datay: sample value
datay=zeros(1,n);

% Create new figure
figure(1);
clf;
plot(datax,datay);
ylim([0 255]);

%% Continuous loop reading data
it=0;
totread=0;
tic;
while 1
    % Read one byte
    [dataread,datareadn]=fread(ser,ser.InputBufferSize);
    
    it=it+1;
    t = toc;
    totread=totread+datareadn;
    fprintf(1,'Iteration %u: read %u. Tot red %u in %f. Sps: %f\n',it,datareadn,totread,t,totread/t);
    
    
      
    % Potentially the numeric data is empty (e.g. if we read text instead
    % of a number). In this case we skip the plotting.
    if datareadn==0
        continue;
    end
    
    % Add data to datay, keeping only the n-most recent samples
    datay=[datay dataread'];
    datay=datay(1,end-n+1:end);
    size(datay);
    
    % Plot the data
    plot(datax,datay);
    drawnow limitrate nocallbacks
       
    
    
end
