%%%%%%%%%%%%%
% ECE 3610
% LAB 10 -- Actuators 3: Combining Sensorimotor Loops
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Modern cyberphysical systems are simultaneously handling sensor input 
% from a number of sources, then intelligently combining that into a 
% number of actuator signals. So far you have only mapped sensors and 
% actuators in a 1:1 fashion. This lab is purposefully open-ended; 
% hopefully it will give you time to both catch up if you're behind and 
% explore if you're ahead. 

% Deliverables:
%   - A circuit and associated code which uses no fewer than six of your 
%   components and contains (not counting the Arduino, motor carrier,  or 
%   a resistor that is part of a voltage divider used to read values) 
%   at least two different feedback/sensorimotor loops operating at the 
%   same time. At least one of the sensorimotor loops should fuse sensor 
%   data from at least two different sources.
%   - Tell me a story about what your device is meant to do. This can be
%   just part of a larger (imaginary) system, you can use analogies 
%   ("instead of an led, this would be a ____ "). Get creative!
%
% Extensions:
%   - Work with a partner to have your systems interact.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all

% for PC:
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');
%%
% Building your sensorimotor loops
% Take stock of your available input and output devices. Write down a list
% of each. Categorize each input as either BINARY or CONTINUOUS. Next to
% each input in your list, if it is a sensor, write down the associated
% physical quantity being transduced.

% HINT: When working with design challenges, it can help to clearly define
% your objectives, list usable components, and brainstorm possible next
% steps. Once you have a decent idea pinned down, start by breaking your
% design task into sections.

% E.g. something for this lab could look like:
%
% Objective: 6 components, 2 interacting sensorimotor loops
%
% Components: DC Motor, LED, potentometer, photoresistor, etc.
%
% Possible concepts: System for robot that changes LED with ultrasonic,
% motor speed dependent on another sensor, etc.
%
% CODE STRUCUTRE:
% 1. pin assignments and initialization
% 2. defining useful variables and ranges
% 3. start of while loop
% 4.   data collection/processing
% 5.   condition testing
% 6.   sensorimotor/feedback loops
% ...
% X. End of while loop/ resetting functions

% You've been setting up sensorimotor loops almost every lab so far, so go
% crazy with finding ways to combine multiple sensors and actuators/transducers!

% pot
nb.pinMode('A0','ainput');
% force sensor
nb.pinMode('A3','ainput');
% ultrasonic
nb.initUltrasonic1('D2','D3');
% rgb led 
nb.initRGB('D10','D11','D12');
nb.initPiezo('M4');
% servo 
nb.setServo(1,90);
% start rgb off
nb.setRGB(0,0,0);

% tuning
minDistCm = 5;
maxDistCm = 50;
minFreqHz = 220;
maxFreqHz = 880;
minBPM = 40;
maxBPM = 180;
loopSec = 0.5;
toneMs = round(loopSec*1000);
bpmCounter = 0;
DownBeat = 0;
red =0;
%%
while (true)     
    % pot -> bpm -> servo
        potVal = nb.analogRead('A0');
         bpm = round((1/potVal)*100)
         if bpmCounter > bpm
             if DownBeat == 1
                 nb.setServo(1,(50));
                 bpmCounter = 0;
                 DownBeat = 0
             else
                 nb.setServo(1, (20));
                 bpmCounter = 0;
                 DownBeat = 1
             end
         else
            bpmCounter = bpmCounter +1;
        end



   %pitch
    distCm = nb.ultrasonicRead1()*0.015;
    distN = (distCm-minDistCm)/(maxDistCm-minDistCm);
    distN = min(max(distN,0),1);
    baseFreqHz = minFreqHz + (1-distN)*(maxFreqHz-minFreqHz);

    % % vibrato 
    forceVal = nb.analogRead('A3')/1023;
    forceVal = min(max(forceVal,0),1);
    vibDepth = 0.10*forceVal;
    freqHz = baseFreqHz*(1 + vibDepth*sin(2*pi*6*toc));
    freqHz = min(max(freqHz,150),1200);

     nb.setPiezo(freqHz, toneMs);
    % nb.setPiezo(100, 1);


%    volume from distance

    % if (distCm>0)
    %     nb.setRGB(255,255,255);
    % else
    %     nb.setRGB(0,0,0);
    % 
    % end

    pause(0.025);
end


%% X. EXTENSION (optional)
% - Work with a partner to have your systems interact.


%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
clear all

% for PC:
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');

% nb.initUltrasonic1('D2','D3');
% while(true)
%     %Take an ultrasonic reading
%     val = nb.ultrasonicRead1();  % change 1 to 2 if you initialized the 
%                                  % second ultrasonic sensor above
%     fprintf('val = %i\n', val) % write output to the screen
%     pause(0.5); % adjust me if you want faster or slower samples
%                 % (move the object to a new position during each 0.5 sec)
% end

%%
nb.initRGB('D10','D11','D12');
while(true)
nb.setRGB(255,255,255);
end



