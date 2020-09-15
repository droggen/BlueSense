function [vcoord,vconnect,vcol]=create_sensor()

%                ^ +Y West
%                I
%            +---I---+
%            I   I   I
%            I B I T I
%            I   I   I
%            +---I---+
%                --------------> +X North
%                I
%                I
%                I
%               +-+
%               +-+ USB
%                I


% This geometry shows the node BT on the positive Y; and node USB on the
% negative Y.  
% This is not the zero position of the node (push button is on positive X
% (North) in the zero position).
% Therefore, this geometry is rotated prior to use.

height = 0.2;

vcoord=[
        % Bottom face
        -0.5 -0.5 -height/2;     % 1
        -0.5 0.5 -height/2;      % 2
        0.5 0.5 -height/2;       % 3
        0.5 -0.5 -height/2;      % 4
        
        % Top face
        -0.5 -0.5 +height/2;     % 5
        -0.5 0.5 +height/2;      % 6
        0.5 0.5 +height/2;       % 7
        0.5 -0.5 +height/2;      % 8
        
        % Bluetooth
        -0.4 +0.5 +height/2;    % 9
        -0.4 +0.2 +height/2;    % 10
        +0.4 +0.2 +height/2;    % 11
        +0.4 +0.5 +height/2;    % 12
        
        % USB
        -0.1 -0.4 +height/2;    % 13
        -0.1 -0.5 +height/2;    % 14
        +0.1 -0.5 +height/2;    % 15
        +0.1 -0.4 +height/2;    % 16
        
        % Push button
        +.5 +.05 +height/2;      % 17 
        +.5 -.05 +height/2;      % 18
        +.45 -.05 +height/2;      % 19
        +.45 +.05 +height/2;      % 20
        
    ];


vconnect = [
    % Bottom face
    1 2;
    2 3;
    3 4;
    4 1;
    % Top face
    5 6;
    6 7;
    7 8;
    8 5;
    % Link top and bottom faces
    1 5;
    2 6;
    3 7;
    4 8;
    
    % Bluetooth
    9 10;
    10 11;
    11 12;
    12 9;
    % USB
    13 14;
    14 15;
    15 16;
    16 13;
    % Button
    17 18;
    18 19;
    19 20;
    20 17;
    
    ];

vcol=[
    % Bottom
    'r';
    'r';
    'r';
    'r';
    % Top
    'b';
    'b';
    'b';
    'b';    
    % Links
    'k';
    'k';
    'k';
    'k';
    % Bluetooth
    'k';
    'k';
    'k';
    'k';
     % USB
    'k';
    'k';
    'k';
    'k';
    % Button
    'k';
    'k';
    'k';
    'k';
    
    ];


