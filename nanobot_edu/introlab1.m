%%%%%%%%%%%%%
% ECE 3610
% LAB 1 -- BLINKY
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Making an LED blink is the "Hello World" of using a microcontroller; it's
% a clear way to draw a connection between code you write and a physical,
% visible output. 
%
% Follow the lab instructions on Canvas and in this file to write a program 
% that connects to your Arduino and then makes the onboard LED blink (the
% orange LED in the corner of the Arduino) at 5 Hz with a 50% duty cycle 
% for 3 seconds. For an optional challenge, you can make the LED blink out 
% "Hello World" in Morse code. 
%%%%%%%%%%%%%%

%%%%%%%%%%%%%%
% IMPORTANT: Run each section of this code individually. For example, in 
% the first section of this code, the computer connect to the Arduino, and
% in the last section of this code, the computer disconnects the Arduino. 
% We don't want to connect and disconnect the Arduino every time we change 
% and test a variable value in the code.  The different sections of this 
% code are separated by a %% and a carriage return.  To run a section, 
% click (put your cursor inside) of the section and press "Run Section" 
% in the menu above or CTRL+ENTER.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or 
%  port_detector.m. Note that the clc and clear all will clear your command 
%  window and all saved variables!

clc
clear all

% The line below calls on the constructor in the file nanobot.m
% (it creates an instance of the class nanobot and calls it "nb").
% Replace the first argument with the text corresponding to
% the correct serial port you found through the port_detector.m file. 
% This part of the code runs successfully if "nb" shows up in your
% workspace and you don't see any errors in the command window.

% NOTE:  If you get the following warning, your Arduino may not have been
% flashed.  In this case, take the Arduino to a TA and tell them it 
% probably needs to be flashed.
%    Warning: The specified amount of data was not returned within the 
%    Timeout period for 'readline'.
%    'serialport' unable to read any data. For more information on 
%    possible reasons, see serialport Read Warnings. 
%    Error: unable to open serial connection!

% for PC:
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% 2. CALCULATE YOUR LED PARAMETERS
%  First calculate some of the important values for this program using the
%  desired behavior (i.e. Onboard LED blinks at 5 Hz with a 50% duty cycle)

% These are variables you can choose:
blinkTime = 5; % time in seconds for the blink program to run 
                 % for testing purposes, it's good to not make this too 
                 % long or too short. 
                 % IMPORTANT:  Remove the '?' (including the quotes) 
                 % and replace with just a number.
blinkFrequency = 1;  %frequency that LED blinks in Hz
dutyCycle = 0.5; % duty cycle of the LED in decimal form (e.g., 20% is 0.2)

% Additional variables we will need:
blinkPeriod = 1/blinkFrequency;
onTime = blinkPeriod * dutyCycle;
offTime = blinkPeriod - onTime; 

%% 3. STEADY BLINK YOUR LED 
%  Use the calculated parameters from the previous section to blink your
%  LED with the desired pattern.

tic
while (toc < blinkTime)
    % The function below appears in the nanobot.m file:
        % Method to write to the onboard LED
          % function ledWrite(obj, value)
              % obj.write('led', 0, value);
          % end
    % obj in this case is nb, which is your Arduino.
    % Call this function/method using: nb.ledWrite(value)    
    % and replace "value" with a numerical value.
    % Keep in mind the built-in programmable LED on the Arduino is 
    % connected to a digital pin (so "value" should be a 1 or a 0!).
    % An example of this is provided in nanobot_demon.m
    nb.ledWrite(1)
    pause(onTime)
    nb.ledWrite(0)
    pause(offTime)
end
nb.ledWrite(0)  % turn off the onboard LED

%% 4. BLINK IN MORSE (optional)
%  Rules of International Morse Code:
%  1. The length of a dot is one unit.
%  2. A dash is three units.
%  3. The space between parts of the same letter is one unit.
%  4. The space between letters is three units.
%  5. The space between words is seven units.
%
%  Define the length of one unit in seconds (unitTime):
unitTime = 0.1; 
%  Set the number of words to be blinked in Morse Code:
words = 2; 

%  Create a matrix to hold "HELLO WORLD" in Morse Code. The following 
%  matrix already has "HELLO".  Fill in the rest of it with "WORLD". 
letters = ["....",".",".-..",".-..","---"; "?","?","?","?","?"]; 

%  HINT: The trickiest part of this code is getting the integer multiples
%  of "unitTime" correct for the pauses after each part of the letter, each
%  letter, and each word. Think about the TOTAL "units" which will elapse
%  in each case, based on execution of the for loops.

nb.ledWrite(0)
for i = 1:words
    for j = 1:length(letters(i,:))
        letter = letters(i,j);
        for z = 1:length(letter{1})
            if letter{1}(z) == '.'
                nb.ledWrite(1)
                pause(unitTime)
            elseif letter{1}(z) == '-'
                nb.ledWrite(1)
                pause('?' * unitTime)
            end
            nb.ledWrite(0)
            pause('?' * unitTime)
        end
        nb.ledWrite(0)
        pause('?' * unitTime)
    end
    nb.ledWrite(0)
    pause('?' * unitTime)
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all