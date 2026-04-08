%%%%%%%%%%%%%
% ECE 3610
% LAB 6 -- Ultrasonic Mapping
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Ultrasonic rangefinding was one of the first techniques for mobile robot 
% obstacle detection and avoidance. Even in a world where onboard computer 
% vision is near ubiquitous, ultrasonic ranging provides a simple, 
% affordable, and computationally inexpensive method of obstacle detection.
% A vast number of small wheeled robots still use ultrasonic and/or 
% infrared rangefinding for autonomy. We will be developing some of the 
% code required to interface with this new sensor, and then characterizing 
% the sensor performance. Getting this right is important -- you'll be 
% using this same module for your final project. We wouldn't want your 
% robots smashing into any walls.
%
% Deliverables:
% - Code that continuously graphs the ultrasonic rangefinder's distance 
% measurement (correctly converted), corrects for any detected 
% nonlinearity, and turns on an LED with varying intensity depending on 
% obstacle distance.
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

%% 2. Connect and initialize the ultrasonic rangefinder
%  First, we need to set up the rangefinding module. Find a line of code in
%  nanobot_demo.m to help you initialize the ultrasonic rangefinder (the
%  file defines two ultrasonic sensors, but we will only use one of them
%  for now.  You can use either, but use #1 since we'll refer to that one 
%  in later sections).

%  Wire up your rangefinder according to Vcc, GND, and the trigger/echo 
%  pins you select during initialization.  For Vcc, use 5V on the Arduino 
%  (3.3 may be too low and Vin will be too high if connected to the motor 
%  carrier).

%Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic1('D2','D3')

%Take a single ultrasonic reading
front = nb.ultrasonicRead1();
fprintf('Front dist = %0.0f\n', front*0.01615);

%% 3. Explore the rangefinder
% In the sub-sections below, using your ruler, you will find the scale, 
% factor resolution, max and min range, and linearity of your ultrasonic 
% rangefinder. 
%  ~1cm min
% 


%% Read the Ultrasonic Data:
% Use this section to take a series of measurements for an object at 
% different distances.  Note, for object placement, there are two circular 
% components on the front of the ultrasonic rangefinder:  one is the 
% transmitter and the other is the receiver.

while(true)
    %Take an ultrasonic reading
    val = nb.ultrasonicRead1();  % change 1 to 2 if you initialized the 
                                 % second ultrasonic sensor above
    fprintf('val = %i\n', val) % write output to the screen
    pause(0.5); % adjust me if you want faster or slower samples
                % (move the object to a new position during each 0.5 sec)
end

%% Graphing Custom Data:
% Take your results from the previous sub-section and enter them into the 
% "val" array below.  The "dist" array holds the distances along your ruler 
% where you placed your object (every 2 cm).

dist = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % in cm
% Replace the strings below with the appropriate value measured at the
% corresponding distance:
val = [140,240,330,470,610,780,890,1000,1113,1360,1440,1650,1812,1900,2033];

% For plotting your data:
scatter(dist, val);
xlabel('distance (cm)'); % label the x-axis!
ylabel('ultrasonic read value'); % label the y-axis!
hold on
% Add to the plot a line that fits the data
fit = polyfit(dist, val, 1);
plot(dist, polyval(fit,dist), '-r');   % plot the regression line
hold off

%% Find the Scale Factor
% The scale factor of our ultrasonic rangefinder can be thought of as the
% coefficient that can be used to convert the rangefinder's numeric output
% to a physical distance that the measurement corresponds to. In this case, 
% we can solve for an average scale factor by using the measured values we 
% recorded at 2 cm increments.

arraySize = size(dist, 2); % determine how many numbers are in dist
scaleFactorList = zeros(1, arraySize); % create and initialize to zero an 
                                       % array of the same size as dist (we
                                       % will use this array to hold the
                                       % scale factor values).
for i = 1:arraySize
    scaleFactorList(i) = val(i)/dist(i); % In [units/cm] 
end
avgScaleFactor =  mean(scaleFactorList); % In [units/cm] % find the mean value
fprintf("The average scale factor is %.3f units/cm\n", avgScaleFactor);

%% Find the Sensor Resolution
% To find the resolution of your sensor (since we're using digital pins on 
% the Arduino), find the smallest distance change that results in a 
% reading change from the rangefinder.  That is, record here by how much 
% the reading (ADC output) changes when you move the object by the smallest 
% amount possible (it will be a very rough approximation, since the output 
% is noisy).
resolution = 0.1; %in cm

%% Measure the Linearity
% To get a numerical estimate of the linearity of your sensor, we can use 
% the R^2 value of a linear regression to get a sense of how well our 
% rangefinder correlates with a true linear response.  An R^2 value close 
% to 1 (100%) means the model (a line in this case) represents the data 
% well, while a value near 0 means the model does not represent the data 
% well.  

yCalc = zeros(1, arraySize);
for i = 1:arraySize
    % Using your scale factor, create an array of expected outputs at each 
    % value of dist:
    yCalc(i) = avgScaleFactor*dist(i); % y(i) = m * x(i), what do m and x correspond to? 
end
% Calculate R^2:
Rsq = 1 - sum((val - yCalc).^2)/sum((val - mean(val)).^2); 
fprintf("The R^2 value for a pure linear fit based on the scale factor is: %f\n", Rsq);

%% Find the Max and Min Range
% These are the extreme values at which your rangefinder still measures a
% stable value. Due to the crowded classroom with many other active
% rangefinder pulses, measuring a stable max range could prove difficult.
% Experiment and find out what you can get as your minimum and maximum
% effective ranges.  This could be in units of cm or in the units of the 
% measured ultrasonic values as long as you are consistent with the units 
% later when you use these variables.
minRange = min(val);
maxRange = max(val);

%% 4. Vary the LED intensity
% Write code to make the red RGB LED vary its brightness based on the 
% distance the ultrasonic sensor reads.

% Solution:
nb.initUltrasonic1('D2', 'D3'); % Fill in with your ultrsonic trigger and echo pins
nb.initRGB('D4','D5','D6'); % Fill in with your RGB module pins
while (true)
    pulseVal = nb.ultrasonicRead1();
    cmVal = pulseVal / avgScaleFactor;
    fprintf("Last read: %0.1f cm\n",cmVal);
    relBright = round((cmVal-2)*(255/(30-2))); % linear interpolation based on custom data. 
                            % HINT: See how you used linear interpolation 
                            % in lab 3 or lab 4 (sensors 1 or 2)
    if (relBright > 255)
        relBright = 255;
    elseif (relBright < 0)
        relBright = 0;
    end
    % Set the RGB so that the red LED intensity decreases for larger
    % distances
    nb.setRGB(255-relBright, 0,0); 
    pause(0.05); % The code is fast, the measurements are not!
end

%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all