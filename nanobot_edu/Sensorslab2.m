%%%%%%%%%%%%%
% ECE 3610
% LAB 4 -- Force-Sensitive Resisitors
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, we will try to get you thinking about the conversion factor
% between a sensor physical stimulus, output electrical quantity, and your
% ADC output, given a circuit you design. We will use a new type of sensor,
% the force-sensitive resistor (FSR).  The FSRs we will be working with 
% today are based on mechanical stimuli, and they can experience an
% appreciable delay between the application of stimulus and the
% resulting electrical change. We will directly measure this for your FSR
% (albeit crudely). We will then dive deeper into the interplay between
% sensor sampling and code, specifically to design an interactive game
% using our FSRs. This lab will be significantly more or less difficult
% depending on your programming experience; don't hesitate to ask
% instruction staff for help.
%
% Deliverables:
% - Make a "Hit the Target" thumb wrestling game.
%
% Extensions:
% - Turn the target, player 1, and player 2 values into a live plot
%       - There are multiple ways to do this: one is to append the new
%       values every loop to a sample array, then re-plot the whole thing
%       each time.
% - Think of fun variations on this game using any components you have on
%   hand
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
% for PC:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% 2. Set up the FSR voltage divider
% Create a voltage divider with the FSR and a single resistor: Use the 10k 
% resistor and the FSR to build a voltage divider between 3.3V and ground, 
% with the FSR nearer 3.3V.  Be careful to grab the FSR as close to the
% pins as possible when inserting into or removing it from the breadboard!

nb.pinMode('A1', 'ainput');
nb.livePlot('analog', 'A1');

% Look at the data sheet for the FSR provided in Canvas, particularly 
% Figure 1. Assuming the voltage divider operates at 3.3V with a 10k 
% resistor, what output from the ADC would you see at 20g? 100g? 1000g?
% Ideally, we want the output of the ADC to extend over the entire 
% 0-1023 ADC range.  We can see what the range is for a few specific
% forces:

% If 20g, then the FSR resistance from the plot is '?' and 
Vout_20g = 3.3*(10000/100000+10000)
% If 100g, then the FSR resistance from the plot is '?' and  
Vout_100g = 3.3*(10000/8000+10000) 
% If 1000g, then the FSR resistance from the plot is '?' and 
Vout_1000g = 3.3*(10000/1100+10000) 

% Convert the voltage to units of ADC output = Vout * (1023 / Vcc) 
ADC_output_20g = (Vout_20g / 3.3) * 1023 
ADC_output_100g = (Vout_100g / 3.3) * 1023 
ADC_output_1000g = (Vout_1000g / 3.3) * 1023 
% Compare these values to the 0 - 1023 range of the ADC output.


%% 2.5 Set up the RGB LED
% Similar to last lab, wire up the RGB LED. Some example pins could be
% D12 for R, D11 for G, D10 for B, and GND for '-'

%% 3. Change LED intensity with the FSR
% Write code that makes the RGB LED a brighter red color the harder you 
% squeeze the FSR.  The maximum value for each RGB value is 255. Here are 
% some example RGB values for red:
% RGB of (255, 0, 0) = bright red (on 100%)
% RBG of (127, 0, 0) = medium bright red (on ~50%)

% HINT: setRGB does not accept decimal values. Use the MATLAB round
% function.

% HINT: It is good practice to include a short pause in every MATLAB while
% loop which sends data to/from the Arduino. Your PC is much faster than
% your microcontroller.

% NOTE: Depending on the quality of your FSR, the minimum resistance could
% range from a few hundred Ohms, to several kOhms. Therefore, since your 
% FSR may not reduce its resistance to a value that is negligible 
% compared to 10 kΩ, you should find the max ADC value of your voltage 
% divider setup.  This can be done through live plotting, or recording a 
% max value over a sampling time.  Record the highest value you observe 
% over the sampling time (don't take the average of the highest values).

% For live plotting (uncomment lines below):
% nb.pinMode('?', 'ainput');
% nb.livePlot('analog', '?');
% maxVal = '?'

% For sampling (uncomment lines below):
% % Forcefully squeeze the FSR within 5 seconds to record the max ADC value
% nb.pinMode('?', 'ainput');
% tic
% vals = [];
% while (toc < 5)
%     vals(length(vals)+1) = nb.analogRead('?');
% end
% maxVal = max(vals);
% fprintf('max val = %.0f\n', maxVal);

% fill in the max ADC output value we expect to get from the FSR voltage 
% divider setup:
maxVal = 1100;

nb.pinMode('A1', 'ainput');
nb.initRGB('D12','D11','D10');
adcRange = [0, maxVal]; % expected range of output values from the ADC.
rgbRange = [0, 255]; % The maximum value for each RGB value is 255.  

tic
while(toc < 20) % Run for 20 seconds
    % To smooth out noise:
    % Here is an example of taking 5 readings, then averaging:
    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    adc = mean(vals);
    
    % Just in case, make sure the ADC output is always equal to or below 
    % the maxVal we selected!
    if adc > maxVal
        adc = maxVal;
    end

% In the last lab, we used the formal definition of linear interpolation to 
% find the corresponding brightness values and colors of the RGB LED. 
% Another way we can do this is by using MATLAB's interp1() function:
% https://www.mathworks.com/help/matlab/ref/interp1.html
% The format for using this function is: 
% interp1([current_min current_max], [desired_min desired_max], value_to_be_converted);

    % Use interp1 to convert the ADC output to an intensity value for the
    % LED
    brightness = interp1(adcRange, rgbRange, adc);

    % Calculate the corresponding RGB values (modify the values being
    % multiplied by brightness to any value between 0 and 255 to change
    % colors!)
    r = round(brightness);
    g = 0;
    b = 0;

    nb.setRGB(r, g, b);
    pause(0.01);
end
nb.setRGB(0, 0, 0); % turn off the RGB LED

%% 4. Measure the rise time
% There is a delay between applying a force to the FSR and seeing the 
% impact of that force in the measurements.  Let's get a feeling for how 
% much of a delay there is.

% Apply a sudden force to the FSR and measure the approximate rise time in 
% seconds between application of force and the settling of the FSR 
% resistance value. Rise time is defined as time it takes for a value to go 
% from 10 percent of its max value to 90 percent of its max value.

clc; % let's clear the command line so we can just see the output from 
     % this section

% If you haven't changed your circuit, your max value from the
% previous section may be used.
maxVal = 1100;

nb.pinMode('A1', 'ainput');

% When we perform repeated analogReads, we get output at a certain rate.  
% Let's determine that rate and use it to determine the rise time in 
% seconds. First, let's find how many samples we can record over 1 second:
ctr = 0; % ctr is short for counter
tic
while(toc < 1)
    temp = nb.analogRead('A1');
    ctr = ctr + 1; % increment counter by one every time a read is 
                   % performed within one second of time
end
% The time (period, in seconds) between samples is:
period = 1/ctr;
fprintf('Time between samples is ~%d seconds.\n', period);

% Let's find the rise time:
riseLow = 0.1 * maxVal; % value for 10 percent of the max value
riseHigh = 0.9 * maxVal; % value for 90 percent of the max value
ctr = 0; % reset the counter
fprintf('Sometime in the next <10 seconds, start squeezing your FSR hard to find the rise time!\n');
fprintf('(Once the rise time is measured, the code will stop taking measurements.)\n\n');
tic
while(toc < 10) 
    adcVal = nb.analogRead('?');

    if(adcVal > riseLow) 
        while(adcVal < riseHigh) 
            % while the ADC output is rising between riseLow and riseHigh, 
            % count how many readings are performed
            adcVal = nb.analogRead('?');
            ctr = ctr + 1; % increase the counter by one
        end
        break; % break out of the 10-second loop as soon as the rise time 
               % measurement is complete
    end
end

% The rise time is:
riseTime = ctr * period; % multiply the counter by the time between readings
fprintf("The rise time is: %d seconds\n", riseTime);

%% 5. Making an FSR "thumb wrestling" game
% Grab a partner and turn your two FSRs into a "thumb wrestling" game using
% a single Arduino and computer. This game should be a race to pinch your
% FSR at a specific, randomly generated force amount for around a full 
% second. Turn the LED to either red or blue, depending on who wins first.

% HINT: Have a tolerance for an acceptable +/- from the desired ADC output
% using the "and" operator in the if statement, &&

% HINT: display something like
% fprintf("Target: %.0f | P1: %.0f | P2: %.0f \n",target,p1val,p2val)
% every time in the while loop.

% HINT:  If you know the loop period, you can count the number of samples
% at the desired output level as a proxy for time.

% NOTE: If you prefer to work by yourself on this for whatever reason,
% please ask instruction staff for a second FSR to borrow.

% Solution (question marks need to be filled in):
% Setup:
nb.pinMode('A1', 'ainput'); % for player 1 FSR
nb.pinMode('A2', 'ainput'); % for player 2 FSR
nb.initRGB('D12','D11','D10');
nb.setRGB(0, 0, 0); % start with the LED off 
plusMinTol = '?'; % ADC can fluctuate between 2 * this val centered around target
adcMax = '?'; % Max ADC value for this setup, based on live read.
adcTarget = randi(adcMax); % generate a random integer between 0 and adcMax
lowBnd = adcTarget - plusMinTol; % lower bound
upBnd = adcTarget + plusMinTol; % upper bound
p1time = 1; % start with 1 second for player 1 and count down to zero 
            % during the time that the target range has been met
p2time = 1; % start with 1 second for player 2 and count down to zero 
            % during the time that the target range has been met
p1Win = 0; % will be set to 1 if player 1 wins
p2Win = 0; % will be set to 2 if player 1 wins

% Since there will be some noise in the ADC output from the FSR voltage 
% dividers, for determining the winner, it is helpful to use averaged 
% values from the FSR voltage divider ADC outputs rather than a single 
% sampled value in time.  Let's see how many averaged readings we can get 
% over 1 second:
ctr = 0; % set the counter to zero
tic
while(toc < 1)
    % To smooth out noise:
    %Here is an example of taking 5 readings and then getting the average:
    numreads = 5;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('?'); % Pin corresponding to player 1
    end
    temp = round(mean(vals));
    ctr = ctr + 1;
end
% Thus the period (in seconds) between the smoothed (averaged) samples is:
period = 1/ctr;

% Game Loop:
while(true)
    % Get an averaged ADC output from player 1
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('?'); % Pin corresponding to player 1
    end
    p1Val = round(mean(vals));

    % Get an averaged ADC output from player 2
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('?');  % Pin corresponding to player 2
    end
    p2Val = round(mean(vals));
 
    % Print the targe value and the averaged reading from each player
    fprintf("Target: %d | P1: %d | P2: %d\n", adcTarget, p1Val, p2Val);

    % Check to see  if 
    if((p1Val >= '?' && p1Val <= '?') && p1time > '?') % if the target 
                                                       % range has been met
                                                       % but not held for 1 
                                                       % full second yet
        p1time = p1time - 2*period; % Subtract 2 time periods since two 
                                    % averaged reads were done since the
                                    % last check (one for each player)
    elseif(p1time <= '?') % if the target range has been met for at least   
                          % one full second
        p1Win = 1;  % player 1 wins
    else
        p1time = 1; % Reset the clock if no one has won yet and the target 
                    % range has not yet been met 
    end

    if((p2Val >= '?' && p2Val <= '?') && p2time > '?') % if the target 
                                                       % range has been met
                                                       % but not held for 1 
                                                       % full second yet
        p2time = p2time - 2*period; % Subtract 2 time periods since two 
                                    % averaged reads were done since the
                                    % last check (one for each player)
    elseif(p2time <= '?') % if the target range has been met for at least 
                          % one full second
        p2Win = 1;  % player 2 wins
    else
        p2time = 1; % Reset the clock if no one has won yet and the target 
                    % range has not yet been met 
    end

    if(p1Win || p2Win)
        break % break out of the loop if either player has won
    end
end

% Winner selection
if(p1Win && p2Win)
    fprintf("WINNER: It's a tie! :O\n")
    nb.setRGB('?', '?', '?'); % Set to green or tie color
elseif(p1Win)
    fprintf("WINNER: Player 1 wins!\n")
    nb.setRGB('?', '?', '?'); % Set to red or player 1's win color
else
    fprintf("WINNER: Player 2 wins!\n")
    nb.setRGB('?', '?', '?'); % Set to blue or player 2's win color
end
pause(5);
fprintf("Thanks for playing!\n");
nb.setRGB(0, 0, 0);

%% 6. EXTENSION (optional)
% - Turn the target, player 1, and player 2 values into a live plot
%       - There are multiple ways to do this: one is to append the new
%       values every loop to a sample array, then re-plot the whole thing
%       each time.
% - Think of fun variations on this game using any components you have on
%   hand

%% 7. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all