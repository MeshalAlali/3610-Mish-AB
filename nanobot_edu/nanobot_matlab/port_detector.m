% first port '/dev/cu.usbmodem101'
% second port '/dev/cu.usbmodem1011'
%
%%%%%%% 
% We are going to use this code to determine which port you are planning
% to use to connect the computer to the Arduino.
%%%%%%% 

clc
clear all

%%%%%%% 
% NOTE:  When MATLAB detects the presence of an Arduino, it may ask you 
% to install some libraries for communicating with it.  You DO NOT need 
% to install these because everything you need is in nanobot_edu.  (We 
% will be using JSON packets to communicate with the Arduino manually.)
% However, make sure you do have the lastest version of MATLAB installed 
% (2023a or higher).
%%%%%%%

% First, plug the microUSB cable into the Arduino.

% Make sure the other end of the microUSB cable is NOT plugged into the 
% computer yet.

message = ['The Arduino''s microUSB cable should *NOT* be plugged into ' ... 
    'your comput' ...
    'er yet.  Press Enter to confirm.'];
disp(message);
pause;
clc

% Now, scan the computer's current USB devices (it should not find the
% Arduino yet)
initialPorts = serialportlist;

% Prompt the user
message = ['Plug the Arduino''s microUSB cable into your computer. ' ... 
    'Press enter when done.'];
disp(message);
pause; % Wait for user to press Enter

% Scan the computer's USB devices again after the user has plugged in the 
% Arduino
newPorts = serialportlist;

% Find the new port that is being used by the Arduino by comparing the two 
% lists
newPort = setdiff(newPorts, initialPorts);

% Display the new port name
if isempty(newPort)
    disp('No new Arduino detected.');
else
    disp(['New Arduino detected at port: ''', newPort{1}, '''']);
    disp('Copy and paste that port name into your lab file!')
end