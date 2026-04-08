%%%%%%%%%%%%%
% ECE 3610
% LAB 9 -- Actuators 2: Music with Piezoelectricity
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Digital sound production is all around us -- I'm listening to music from
% my computer speakers as I write this! Piezoelectric actuators are some 
% of the simplest possible speakers, and their amenability to 
% miniaturization means that they are ubiquitous. Because of their low 
% fidelity, they are mostly just used as annoying buzzers and alarms. Let's
% make some music instead.

% Deliverables:
%   - Play the C major scale with one quarter note per beat (in 4/4 time) 
%     at 100 BPM using your piezo.
%   - Play Hot Cross Buns using your piezo at a much different BPM by
%   adjusting a potentiometer.

% Extensions:
%   - Take arbitrary character input from your keyboard to play a song.
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
%nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');


%% 2. Set up a potentiometer for variable BPM
% For this lab, we're going to implement the ability to change the BPM 
% using a potentiometer. Wire up your pot to 3.3V and GND, with
% the wiper connecting to one of the analog pins.

% Next, we want to convert the pot values (output of the ADC for the chosen 
% analog pin) to values between our desired minimum and maximum BPM limits. 
% A good range of BPM values for most songs is between 40 and 180 BPM, so 
% perhaps use these as the min and max BPM values. Then, to find the 
% maximum and minimum ADC values, use the live plot code.

 %nb.pinMode('A0', 'ainput');
 %nb.livePlot('analog', 'A0'); % Use me to find your min and max analog values

minADC = 0; % take the actual minimum, not an averaged minimum
maxADC = 1024;
minBPM = 40;
maxBPM = 180;

%% 3. Define a function for finding the length of a quarter note
% If we want to change the BPM, we need to recalculate the length in 
% time of a quarter note. To make the code *slightly* less cumbersome, 
% we've defined a function at the end of this code (per MATLAB syntax 
% guidelines) to calculate and return the period (in ms) of a quarter note. 
% Jump to the end of this code to define the function, then return here.
%
% Give your function definition a test here once you've filled it out.
% Make sure you put the function arguments in the same order here when
% calling the function in this section as when you define the function (at 
% the end of this code)
% e.g. qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);
% 
% Adjust your potentiometer and re-run this section, are the BPM values
% different?  When the pot is turned all the way down or up, do you
% correspondingly get the min and max BPM?

qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);


%% 4. Set up the piezo and the note frequencies
% Insert a wire from your piezo into the M4+ port, and the other into the
% M4- port (the piezo is bipolar, so it doesn't matter which one is plugged 
% into each port). Make sure you screw down until they are in there firmly.
%
% When you start using the piezo in these next parts of the lab, if your 
% buzzer doesn't make noise when you try to play something on it,
% check that the screw contacts are touching the wires, not the insulated
% jacket. Try power cycling the switch on your motor carrier board
% (disconnecting and reconnecting battery power). If in doubt, you can also
% try reconnecting to your Arduino.
% 
% The piezo will not be very loud.  You can try to make it louder by
% pinning it partially down (on one edge) with a fingernail or screwdriver
% face down against a hard surface.  You can also check the voltage of the
% battery (make sure it is high enough).  You can also try using a paper
% cup as a makeshift amplifier cone.
% 
% Modify the code below to contain the correct frequencies corresponding to
% a C major scale. We want to use equal-tempered frequencies for
% key-independence.
%
% Here's some info on the C major scale:
% https://en.wikipedia.org/wiki/C_major

% Solution:
% Scale frequency array (using equal-tempered frequencies for key-independence)
noteFrequencies = [261.63, 293.66, 329.63, 349.23, 392.0, 440, 493.8, 523.25];

%% 5. Play an ascending C major scale
% Using your frequency array and your note lengths, play an ascending
% C major scale at 100 BPM in 4/4 time (rerun section 3 of this lab until
% your pot is set to provide a BPM of ~100).

nb.initPiezo('M4');  % check nanobot_demo for help
nb.pinMode('A0', 'ainput'); % set up the pinMode for the pot

qNote = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM); % use your function definition

for note = 1:length(noteFrequencies)  % play through all the notes of the scale
    nb.setPiezo(noteFrequencies(note), round(qNote));
    pause(qNote/1000);  % THIS IS IMPORTANT! It delays the code so that 
                        % the piezo has time to play before MATLAB goes 
                        % on and tries to play the next note.  The pause
                        % needs to be at least as long as the note being
                        % played.  Here, qNote is divided by 1000
                        % because the pause function takes a value in
                        % units of seconds, whereas qNote is in units of
                        % milliseconds.
end

%% 6. Play 'Hot Cross Buns' on a Piezo Speaker
% Adjust your pot so that it provides a much different BPM than what you 
% used for the C major scale. Then return here and play Hot Cross Buns. 
% Refer to the music sheet on the Canvas lab page and select the correct 
% notes using your frequency arrray. We've given you the first note of 
% bar 1 so you can see the structure of each note.  The first note is an E, 
% which is the third note in the C major scale (hence, we are calling on 
% the third value in the frequency array).  The first note is also a half 
% note, which is twice as long as a quarter note (hence the qNote*2).
% HINT: Using 'for' loops for Bar 2 of the song may help you reduce code
% clutter.

% Solution:
nb.initPiezo ('M2');
nb.pinMode('A0', 'ainput');

% Set the speed
Note = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM);

% Bar 1:
% First note (half): 
nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*2/1000);  % the pause here needs to be the length of the note 



nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*2/1000);  % the pause here needs to be the length of the note 



nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(5), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(1), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(2), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(3), round(qNote*2));
pause(qNote*2/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(4), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(4), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(4), round(qNote));
pause(qNote*2/1000);  % the pause here needs to be the length of the note 


nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 

nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*1/1000);  % the pause here needs to be the length of the note 
nb.setPiezo(noteFrequencies(3), round(qNote));
pause(qNote*2/1000);  % the pause here needs to be the length of the note




                      % that was just played in units of seconds so that
                      % MATLAB will wait before trying to play the next 
                      % note.

% Bar 2:

% Bar 3:


%% 5. EXTENSION (optional)
%  Take arbitrary character input from your keyboard to play a song.
% HINT: Since MATLAB doesn't have default support for keyboard event
% listening, we could set up a song by getting input from the command line.
% One way of doing this may be to use CELL ARRAYS, with each cell
% containing a two-letter string. The first letter could indicate the note,
% and the second could indicate the timing (quarter, half, whole, etc.)
% HINT: One way of mapping characters to frequency values and periods is to
% use the matlab containers.Map() function.
% HINT: Cell arrays are indexed as follows:
%   - cellArray(i) -> returns the actual cell object at i
%   - cellArray{i} -> returns the object stored in cell i
%   - cellArray{i}(j) -> returns a value at index j in an array stored in cell i

% Here are some useful resources:
% - https://www.mathworks.com/help/matlab/cell-arrays.html
% - https://www.mathworks.com/help/matlab/ref/containers.map.html

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

%% Function Definitions
% This is a function definition: 
% - qNoteCalc is the name of the function.
% - The goal of this function is to calculate how long each quarter note
% should be in milliseconds so that we can tell the piezo speaker how long 
% it should turn on for each note.  
% - qNoteLen (quarter note length) is the stand-in name for the return  
% value of this function. 
% - You need to pass all the required values into the function when it is 
% called from the main code. In this case, it's the nanobot (nb), as well 
% as the interpolation bounds.
% - The order of the parameter names must match the order of the numerical 
% values you pass into the function when the function is called in the 
% main code.

% Replace the fixme's below with appropriate interpolation bounds. 
function qNoteLen = qNoteCalc(nb, minADC, maxADC, minBPM, maxBPM); % min and max adc, min and max BPM

    % Using the parameters you pass to the function, do an analog read,
    % then linearly interpolate to find the corresponding bpm value. Then
    % calculate the length (in ms) of a quarter note at that bpm.
    adc = nb.analogRead('A0'); % Take a reading here!
    bpm = round(interp1([minADC,maxADC],[minBPM,maxBPM],adc)); % Use linear interpolation here!
    fprintf("Currently selected BPM: %d\n", bpm);
    % Convert the BPM to length in time for each beat (each quarter note)
    % i.e. the units for qNoteLen is milliseconds per beat
    qNoteLen = 60000/bpm; 
end