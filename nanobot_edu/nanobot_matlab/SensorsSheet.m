%%%%%%%%%%%%%
% ECE 3610
% LAB 3 -- Analog Inputs and Variable Resistances
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% This lab will serve as a basic introduction to one of the most important
% functions of a microcontroller: interfacing with external circuitry and
% sensors to read measured stimulus and convert it into a
% computationally-tractable form. We will use an essential basic circuit
% topology, the voltage divider, along with two common components: a
% potentiometer and a flex sensor.
%
% Deliverables:
% - Print the incoming analog value of the potentiometer circuit, converted
% to a voltage, at approximately 2 Hz.
% - Change the RGB LED's brightness with the potentiometer, and change the
% color of the LED from green to red depending on the flex sensor angle.
%
% Extensions:
% - Use the potentiometer knob position to set the flex sensor bend angle
% which will turn on the onboard LED.
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

%% 2. A pot as a voltage divider
% The blue potentiometer is a 10k pot, meaning there is a total of 
% 10k ohms of resistance between the two outside pins of the pot.  
% We can use use the knob of the pot to adjust how that 10k resistance 
% is split into two parts (where both parts always add to 10k). In this 
% way, the pot is a voltage divider, and the voltage measured at the 
% middle pin is adjustable using the knob.  Let's see this in action.   

% Using the pot and three wires, build a voltage divider between the 
% Arduino's +3.3V and the Arduino's GND.  Connect the analog read A1 pin 
% to the middle pin of the pot so that we see how the voltage changes as 
% we rotate the knob.

% In the last lab, we saw that the values provided by the Arduino's
% analog pins vary from 0 to 1023 (2^10-1).  (The Arduino 
% uses a 10-bit ADC, which means it can only display 1024 different 
% values.)  We saw that it registers 0 for 0 V and 1023 for the maximum 
% voltage of 3.3V (the Arduino's 5V pin is powered by the USB cable and 
% not by the Arduino board, since the Arduino's operating voltage is 3.3V).

% Let's convert the analog value recieved at the Arduino's A1 pin from a 
% the ADC value into a voltage. 

nb.pinMode('A1', 'ainput'); % Set the analog pin to read analog input

% Since the measured value fluctuates a bit, let's average 10 samples of 
% the input to get the average analog value recorded:
numreads = 10;
vals = zeros(1,numreads);
for i = 1:numreads
    vals(i) = nb.analogRead('A1');
end
meanval = mean(vals);
fprintf('mean ADC reading = %.0f\n', meanval);

% To convert this to a voltage, we can use the equation:
% 
% analog_voltage_measured / max_system_voltage 
%                               = ADC_reading / total_number_of_ADC_values 
%
mean_voltage = '?';
fprintf('mean voltage = %.1f\n', mean_voltage);

%% 3. Printing analog voltages
% Sometimes it is useful to print values to the screen at a certain
% rate.  Convert the incoming analog value to a voltage and print it to the
% command window at a rate of 2 Hz.  As this part is running, you can
% adjust the knob on the pot to see how the voltage changes.

% Solution:
nb.pinMode('A1', 'ainput');
while(true)
    tic
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1');
    end
    meanval = mean(vals);
    mean_voltage = '?';  %copy the same equation from the last section
    fprintf('mean voltage = %f V\n', mean_voltage);
    while(toc < '?') % What period in seconds corresponds 2 Hz? 
        pause(0.01); % Do nothing in small increments until time to re-print
    end
end

%% 4. Using a flex sensor
% Make a voltage divider circuit on your breadboard using the flex sensor
% and the 10k resistor. Put the 10k resistor closer to 3.3V and the
% flex sensor closer to GND.  Connect Vout to A1 on your Arduino. Be 
% careful when plugging the flex sensor in; grip it as close to the base as
% possible.
%
% Check the image in Canvas for today's lab.  The flex sensor is only 
% designed to be flexed in one direction -- away from the side with the 
% stripes.  Bending the sensor in the other direction will not produce any 
% reliable data and may damage the sensor.  Also, take care not to bend the 
% sensor at its base (at the connectors), as they have a tendency to kink 
% and fail.  Instead, bend the middle portion of the flex sensor (where 
% the stripes are).
%
% Use a live plot to see what happens to the analog value when you bend
% the flex sensor. 


nb.pinMode('A1', 'ainput');
nb.livePlot('analog', 'A1');

%% 5. Calculating approximate flat vs. bent resistance
% Use the live plot analog values obtained in the last section to 
% calculate the approximate resistance of the sensor when it is flat vs.
% bent 45 degrees.  Look at the datasheet for the flex sensor (see Canvas).  
% Do the values you get agree with the datasheet specifications for flat
% vs. bent?  Remember, the stated flat resistance in the datasheet is 
% +/-30%, which means your measured flat resistance can be quite a bit 
% different from the stated 25k Ohms!  You also may want to make sure the
% resistor you are using is 10k Ohms.

% Solution:
flat_voltage = '?' *(3.3 / 1023); % fill in the corresponding ADC value 
                           % from the plot generated in the last section.
                           % The (3.3 / 1023) factor converts it from ADC 
                           % to volts as we saw in Section 2.
% If we say R1 is the 10k resistor and R2 is the flex sensor, we can use 
% the voltage divider equation: 
%    Vout = Vin * (R2 / (R1 + R2))
% to find R2 (the resistance of the flex sensor. In this case, Vin is the 
% total voltage, Vout is the voltage recorded at A1 (the voltage across 
% the force sensor). Multiply both sides by (R1 + R2):
%    Vout * (R1 + R2) = Vin*R2
% Move all of the R2 terms to one side:
%    Vout * R1 = R2 * (Vin - Vout) 
% Solving for R2:
%    R2 = (Vout * R1) / (Vin - Vout)
flat_resistance = ('?' * '?') / (3.3 - '?'); % replace the '?' blanks with 
                                             % variable names, not numbers.
fprintf(['Approximate resistance of the flat flex sensor ' ...
    'is: %f Ohms\n'], flat_resistance);

bent_voltage = '?' *(3.3 / 1023);
bent_resistance = ('?' * '?') / (3.3 - '?'); 
fprintf(['Approximate resistance of the flex sensor ' ...
    'at 45 degrees is: %f Ohms\n'], bent_resistance);

%% 6. Using the flex sensor and potentiometer to control an RGB LED
% Leave your flex sensor voltage divider intact for this Step.
% We're going to control an RGB LED using the flex sensor and
% potentiometer. To hook up your RGB LED, connect R to pin D12, G to D11,
% and B to D10. Connect the '-' sign on the LED to GND. Leaving your flex
% sensor voltage divider intact, hook up your potentiometer to 3.3V and
% GND. Hook up the wiper (middle) pin of the potentiometer to pin A2 on 
% the Arduino.
%
% We're going to control the brightness of the LED using the potentiometer,
% and we'll make the LED change color from green to red depending on how
% much it's been flexed.

nb.pinMode('A1', 'ainput'); % For flex sensor
nb.pinMode('A2', 'ainput'); % For potentiometer
nb.initRGB('D12','D11','D10'); % Initialize the RGB

% First, find the ADC values that bound the range of each
% sensor. The easiest way to do this could be by doing a live plot of each
% sensor and noting what ADC values the range seems to be bound by.

% Uncomment the following line(s) to run a live plot
% nb.livePlot('analog', 'A1');
% nb.livePlot('analog', 'A2');

% Record your max and min ADC values for each sensor:
flexMax = '?';
flexMin = '?';
potMax = '?'; 
potMin = '?'; 

% Start our loop:
tic
while toc < 30 % for up to 30 seconds
    % Let's first figure out what color the LED should be.
    % Take a reading from the flex sensor:
    flexVal = nb.analogRead('A1');
    % We want to linearly interpolate the flex reading that exists within 
    % the range of the flex sensor values to the range of the LED 
    % (0 to 255). To do this, we can compare ratios as in the equation 
    % below (where "readVal" is the reading from the flex sensor, 
    % "readMin" is the minimum possible value from the flex sensor, and 
    % "readmax" is the maximum possible value from flex sensor; 
    % analogously, all of the "target" values on the right side of the 
    % equation correspond to LED values between 0 and 255):
    %
    %       (readVal - readMin) / (readMax - readMin) = 
    %                    (targetVal - targetMin) / (targetMax - targetMin)
    %
    % Rearranging to solve for targetVal:
    %
    %       targetVal = ((readVal - readMin)/(readMax - readMin))*
    %                            (targetMax - targetMin) + targetMin

    % Before we actually perform the linearly interpolation, let's make
    % sure the value read from the flex sensor is within the range we
    % specified 
    
    if flexVal > flexMax % if flex reading is greater than declared maximum 
        flexVal = '?'; % set to what so we don't get a value out of bounds?
    elseif flexVal < flexMin % if flex reading is less than declared minimum 
        flexVal = '?'; % set to what?
    end
    
    % Now we can linearly interpolate the flex reading to an RGB value:
    rawRGB = (('?' - '?')/('?' - '?'))*('?' - '?') + '?';

    % rawRGB can range from 0 to 255.  We need to use that one value to set 
    % both the red and green LED values.  As one grows, the other should 
    % decrease:  
    redRGB = rawRGB;
    greenRGB = 255 - redRGB;

    % Next, let's scale the brightness of the LED.
    % Take a reading from the output of the pot:
    potVal = nb.analogRead('A2');
    % Since the pot simply controls the brightness, and both the ADC 
    % reading from the pot and the RGB LED ranges have a minimum value of 
    % zero, we can set the brightness using a ratio of the current pot 
    % reading divided by the max pot value:
    bright = '?'; % pot ADC reading divided by the max pot ADC value

    % Use that ratio to scale the brightness:
    redRGB = redRGB * bright;
    greenRGB = greenRGB * bright;

    % The resulting RGB values may not be integers, but the setRGB function 
    % we will use to control the LED only accepts integer values. Lets 
    % round them:
    redRGB = round(redRGB);
    greenRGB = round(greenRGB);

    % Now we can set the RGB to the correct values (and include an optional
    % pause for less computation/noise by not looping around again so 
    % quickly)
    nb.setRGB('?', '?', 0);
    pause(0.05);
end
nb.setRGB(0,0,0); % turn off the LED

%% 7. EXTENSION (optional)
%  - Use the potentiometer knob position to set the flex sensor bend angle
% which will turn on the onboard LED (i.e. when you bend the flex sensor to
% the angle equal to or larger than the angle specified by the 
% potentiometer, the onboard LED will turn on).

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all