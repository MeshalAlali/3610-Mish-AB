%%%%%%%%%%%%%
% ECE 3610
% LAB 7 -- Sensors Milestone
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% This is the first lab where you will work with the robots used in
% your final project. To start, we will work on line detection and
% interfacing with the IR reflectance sensors on the robot. Since there 
% are a finite number of robots, you will be working in groups to 
% accomplish this lab.
%
% Deliverables:
% - Develop a formula for error that results in large values when the
% robot is not centered on the line, and near zero when it's aligned.
% - Using what you've learned, implement code that changes the color and
% brightness of an RGB LED dependent on the sign and magnitude of the
% error.
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

%% 2. Connect to the reflectance array
% First, get into groups of 2-3 and get a project robot from the cabinet.
% The Arduino should be plugged into the motor carrier and the LiPo battery
% should be plugged in and turned on.

% Follow the wiring diagram in Canvas to connect your Arduino to the
% reflectance arrays. Included below is a short key with the connections
% for reference.  As needed, use a breadboard.
% 
% KEY:
% Pololu Array Pin | Arduino Pin
%     1 | V_IN
%     3 | A0 (IR #5)
%     4 | D8 (IR #1)
%     9 | D10 (IR #2)
%    11 | GND
%    12 | GND
%    14 | +5V
%    15 | A1 (IR #6)
%    18 | D12 (IR #4)
%    22 | D11 (IR #3)
% JP1-2 | +5V  (start with this connected)

% If the red LEDs on the bottom of the array do not light up and stay lit, 
% you will not get reliable readings.  If the red lights do not light up:
% -- Double check your wiring.
% -- The LiPo battery may be too low. 
% -- Double check that that two VUSB pads are soldered together on the 
%    underside of the Arduino.
% -- You can also see if connecting or disconnecting the +5V on the 
%    JP1-2 helps. 

% Once the red lights are on, give your reflectance sensor a test using 
% some demo code to verify that everything is working correctly.

% Initialize the reflectance sensor with default pins A0, D12, D11, D10,
% D8, and A1 (it is a combination of analog and digital pins so that you
% have both types of pins still available for your final project).
nb.initReflectance(); 

% Take a single reflectance sensor reading
val = nb.reflectanceRead;

% Write out the values:
% - The sensor values are saved as fields in a structure. 
% - Values one to six are in order from left to right in the sensor array.
% - For our sensors, the time is measured in microseconds. Higher values 
% correspond to less reflective surfaces, and vice versa.
fprintf(['one: %.2f, two: %.2f, three: %.2f, four: %.2f\n, ' ...
    'five: %.2f\n, six: %.2f\n'], val.one, val.two, val.three, ...
    val.four, val.five, val.six);


%% 3. Estimate the Error 
% How can we quantify how "off-the-line" we are, given the 6 values we
% record from our reflectance array? Since our robots will eventually be
% moving and following lines using line detection, it's important that we
% stay centered.

% One way of approaching this is by using an error. An error value can be
% created by a simple mixture of addition and subtraction of signals, but 
% the key is that the error should reduce to zero when an optimal condition
% is reached and when the robot is off-the-line in one direction the error 
% will change one way and when it is off-the-line in the other direction 
% the error will change in a different way.  Additionally, the error should
% grow as the robot gets further off the line.

% Below is some code to help you experiment with error. Each value is 
% multiplied by a coefficient and is either added or subtracted.  Take 
% note of the maximum error values you tend to see. They'll come in handy 
% later.  Remember, values one to six are in order from left to right in 
% the sensor array.

nb.initReflectance();
while(1)
    val = nb.reflectanceRead();
    error = -5*val.one - 2*val.two - val.three ...
        + val.four + 2*val.five + 5*val.six;
    fprintf("Error: %d\n", error);
end

% Record the max errors observed:
maxError = 3000;

%% 4. Line detection
% Your ultimate goal is to implement line detection such that an LED lights 
% up in one color corresponding to error in one direcion, and another 
% color for error in the other direction. When implemented correctly, the 
% LED should shift colors similarly to how a pair of differential drive 
% motors would correct for line deviance (i.e. one color corresponds to 
% turn left, the other color corresponds to turn right, a mixture of both 
% colors means go straight). You have been introduced to all of the tools 
% needed to do this already, but don't hesitate to ask questions if you 
% get stuck!

% See if you can get started on your own, but here are some helpful hints 
% if you decide at any point that you want some assistance:
% - You'll be using the error formula you developed in the last section, 
% along with linear interpolation to get an RGB intensity value.
% - For the purposes of determining the LED intensity, use your minimum and
% maximum recorded error values.
% - Depending on whether your error is positive or negative, your color
% should change from one to the other.
% - As one color's intensity decreases, the other should increase
% proportionally. This means that when the robot is centered, both colors
% should be at equal brightness.
% - You last used the LED lights in a similar manner in Lab 6, so your 
% solution for that lab may serve as a resource for completing this section.
% - It would be a good idea to write your error to the screen so you can
% double check your result.

% The general code structure should follow something like this:
% - define error and RGB ranges
% - while loop (take many readings sequentially and each time adjust the LEDs)
%   - perform read
%   - calculate error from read
%   - write your error to the screen for checking with LED color later
%   - linearly interpolate error to RGB
%   - modify RGB value to split into two colors that change proportionally
%   - pause for short time (optional)

nb.initReflectance();

maxError = 5000;

nb.initRGB('D2','D3','D4');
errorrange = [-100, maxError];
rgbrange = [0, 255];


while(1)
    val = nb.reflectanceRead();
    error = -val.one - val.two - val.three ...
        + val.four + val.five + val.six;
    fprintf("Error: %d\n", error);

    if error > 0
        red = interp1(errorrange, rgbrange, abs(error)); 
    else
        red = 0;
    end

    if error < 0
        blue = interp1(errorrange, rgbrange, abs(error)); 
    else
        blue = 0;
    end
    nb.setRGB(red, 0,blue); 
end




%% 5. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all

