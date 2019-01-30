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


%% Continuous loop reading data
while 1
    % Read one line
    str = fgetl(ser);
    % Print it as a string
    fprintf(1,'%s\n',str);
    
    %% Do some dummy processing
    % Convert data to numbers
    data=str2num(str);
    % Compute some mathematical function
    datam = mean(data);
    fprintf(1,'Data mean: %f\n',datam);
       
    
    
end
