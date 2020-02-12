function [vcoord,vconnect,vcol]=create_cube()

vcoord=[
        % Bottom face
        -0.5 -0.5 -0.5;     % 1
        -0.5 0.5 -0.5;      % 2
        0.5 0.5 -0.5;       % 3
        0.5 -0.5 -0.5;      % 4
        
        % Top face
        -0.5 -0.5 +0.5;     % 5
        -0.5 0.5 +0.5;      % 6
        0.5 0.5 +0.5;       % 7
        0.5 -0.5 +0.5;      % 8
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
    ];

vcol=[
    % top
    'b';
    'b';
    'b';
    'b';
    % bot
    'r';
    'r';
    'r';
    'r';
    % links
    'k';
    'k';
    'k';
    'k';
    ];


