%%%%%%%%%%%%%
% ECE 3610
% INTEL LAB 1 -- Intro to Intelligence 
%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or 
%  port_detector.m. Note that the clc and clear all will clear your 
%  command window and all saved variables!

clear; clc; close all; % initialization

% for PC:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

nb.ledWrite(0); % make sure the LED is off

%% 2. COLLECT AND STUDY SOME ACCELEROMETER DATA 
% This section of the code enables accelerometer streaming and then 
% collects a 1.5-second segment of data.  Run this section several times to
% get a feel for what the accelerometer data looks like for different
% gestured numbers.  Each time you run the code a new figure with your 
% data will appear.  Start by gesturing "1" and "0", but feel free to try 
% additional gestures! You can add "close all" here if you ever want to
% close all of the figures (alternatively, run the next section of 
% code).

% For all of these intel labs, you can hold the dowel however you want.  
% However, you should try to be consistent in how you hold it from one lab 
% to the next.

% countdown is defined in a separate file in the nanobot_matlab folder
countdown("Beginning in", 3); % display a countdown to prep user to move 
                              % accelerometer 

nb.ledWrite(1); %Turn on the LED to signify start of recording data
disp("Make A Gesture!"); %prompt user to make a movement

numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
vals = zeros(3,numreads);
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
nb.ledWrite(0); %Turn off the LED to signify end of recording data
clc;

data = [vals(1,:);vals(2,:);vals(3,:)]'; % stop recording

figure(); plot(data, 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes 
title("Add note here about the number gestured"); % to help you keep track 
                                                  % of what is what 

%% Close the figures
% Run this section of code whenever you want to close all of the figures 
% at once.
close all;
                                                  
%% 3. DETERMINE THE GESTURE FOR THE ACCELEROMETER TRACE 
% In this part of the code you will make one gesture, classify it, and 
% plot the results.

countdown("Beginning in", 3); %display a countdown to prep user to move accelerometer 

nb.ledWrite(1); %Turn on the LED to signify start of recording
disp("Make A Gesture!"); %prompt user to make a movement

numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
vals = zeros(3,numreads);
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
nb.ledWrite(0); %Turn off the LED to signify end of recording
clc;

data = [vals(1,:);vals(2,:);vals(3,:)]'; % stop recording

% Below, you need to write your own code to use the 3-axis (x,y,z)
% accelerometer data to determine what number the user gestured. 
% At this point, we are not going to use any machine learning.
% At a minimum, you need to write code capable of determining if the user 
% gestures a ZERO or a ONE. Currently, the code just randomly sets the 
% variable "label" to ONE or ZERO. You should write code to automatically 
% set the variable "label" to ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, 
% SEVEN, EIGHT, or NINE. Start with differentiating between ZERO and ONE. 
% Then try to incoprorate TWO. Then add THREE, then FOUR, etc.


%%%%%%%%%%%%% DETERMINE WHICH GESTURE (YOUR CODE GOES HERE) %%%%%%%%%%%%%%

if max(data(:,3)) > 1
    label = {'ONE'};
else
    label = {'ZERO'};
end


%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%

% PLOT THE ACCELEROMETER TRACE ALONGSIDE THE CLASSIFICATION  
label = categorical(label); %convert the text into a categorial label
figure(); plot(data, 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification: " + string(label)); %title plot with the label

% Keep the Arduino taped to the wooden dowel, since you'll be using it
% again next time.

%% 4. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
close all
