%%%%%%%%%%%%%
% ECE 3610
% LAB 12 -- PID-Based Line Following
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, you will be working in teams (ideally your final project 
% teams, but this is not required for this lab) to create a PID feedback loop
% and use it to tune a robot to smoothly follow a black line on a white
% background. Line following will be a core part of your final project, so
% it's good to get some experience with it early!
%
% Deliverables:
%   - Demonstrate that your robot accurately and smoothly follows a 
%     provided line without losing tracking.
%%%%%%%%%%%%%%

%% WIRING
% This lab has lots of wiring! Be sure to check out the wiring diagram on
% Canvas to map all of the wires where they need to go. Double check this
% before connecting the Arduino to power.

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

%% 2. RECORD WHICH ROBOT YOU'RE USING 
% From now on (for the final project), your group will use the same robot 
% for consistency.  Record here which robot you are using for future 
% reference.

% ROBOT = '114'

%% 3.  TEST IF ROBOT GOES STRAIGHT (NO LINE FOLLOWING YET)
% First, make sure the battery pack is plugged in and is on.  If the
% battery has sufficient charge, the red lights under the sensing array
% should be on.
%
% There is an emergency shutoff section at the end of the code that you 
% can use if needed.
%
% Your motors may not turn at the same rate when fed the same duty
% cycle, meaning your robot will constantly drift to one side, which can
% make tuning difficult. To combat this, find a factor mOffScale that 
% roughly makes your robot go in a straight line. Apply this value later 
% to the control signal of the stronger/weaker motor to even out the speed 
% difference. (Make sure you use switch out the LiPo battery when it starts
% to get low, so that your values and tuning aren't impacted by a low
% battery.)

% At first, the best way to see if the wheels are moving at the same speed 
% might be to pick the robot up and watch the wheels.

mOffScale = 1.2; % Start with 1 so both motors have same duty cycle.

% The base motor duty cycle (speed) you wish to travel. 
% (recommended values are 9 or 10)
motorBaseSpeed = 9;

% Set the duty cycle of each motor
m1Duty = -motorBaseSpeed;
m2Duty = mOffScale * motorBaseSpeed;
% m2Duty = motorBaseSpeed;


tic % start the time

% It can be helpful to initialize your motors to a higher duty cycle
% for a very brief moment (here 0.03 seconds), just to overcome the 
% gearbox force of static friction so that lower duty cycles don't stall 
% out at the start.
% (recommendation: ~10, with mOffScale if needed)
nb.setMotor(1, mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 2.5) % adjust the time to test if robot goes in straight line
                % (shorter time saves battery; longer tests longer path)
    nb.setMotor(1, m1Duty);
    nb.setMotor(2, m2Duty);
end

% Turn off the motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% 4.  CALIBRATE THE SENSOR VALUES 
% The values provided by the sensor array at the front of the robot will 
% vary depending on lighting conditions even though it uses IR and each
% sensor has its own source.  For example, if the projector is on in the 
% room, it could change the performance of your line sensing.  As a result, 
% its a good idea to use calibrated sensor values to account for the 
% current conditions.  In the next two sections, we will record the max 
% and min values.

%% MIN REFLECTANCE VALUE CALIBRATION (white background)
% Remember, if the red LEDs on the bottom of the array do not light up and 
% stay lit, you will not get reliable readings.  If the red lights do not 
% light up:
% -- Double check your wiring.
% -- The LiPo battery may be too low. 
% -- Double check that that two VUSB pads are soldered together on the 
%    underside of the Arduino.
% -- You can also see if connecting or disconnecting the +5V on the 
%    JP1-2 helps. 
% Put the sensor array over a white background and find the expected min
% reflectance array values.

% Initialize the reflectance array.
nb.initReflectance();

%Average a few values 
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
minVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];

fprintf(['Min Reflectance - one: %.2f, two: %.2f, three: %.2f four:' ...
    '%.2f five: %.2f six: %.2f\n'], ...
    minVals(1), minVals(2), minVals(3), minVals(4), minVals(5), minVals(6));
minReflectance = [96,82.7,80.4,75.7,94.7,119.20]; % Set me to min reflectance 
                                             % values for each sensor for
                                             % future reference

%% MAX REFLECTANCE VALUE CALIBRATION (all sensors over black tape)
% Put the sensor array over a black background and find the expected max
% reflectance array values.

% Initialize the reflectance array.
nb.initReflectance();

%Average a few values
avgVals = zeros(10, 6);
for i = 1:10
    read = nb.reflectanceRead();
    avgVals(i, 1) = read.one;
    avgVals(i, 2) = read.two;
    avgVals(i, 3) = read.three;
    avgVals(i, 4) = read.four;
    avgVals(i, 5) = read.five;
    avgVals(i, 6) = read.six;
end
maxVals = [mean(avgVals(:,1)), mean(avgVals(:,2)), mean(avgVals(:,3)), ...
    mean(avgVals(:,4)), mean(avgVals(:,5)), mean(avgVals(:,6))];
fprintf(['Max Reflectance - one: %.2f, two: %.2f, three: %.2f '...
    'four: %.2f five: %.2f six: %.2f\n'], ...
    maxVals(1), maxVals(2), maxVals(3), maxVals(4), maxVals(5), maxVals(6));
maxReflectance = [1349.9,982.7,854,819.5,971.9,1369.5]; % Set me to max reflectance 
                                             % values for each sensor for
                                             % future reference

%% 5.  LINE FOLLOWING PID LOOP

minVals = [70,70,70,70,70,70]; % Set me to min reflectance 
maxVals = [900,900,900,900,900,900]; % Set me to max reflectance 


% Though PID tuning can be tedious and frustrating at times, the payoff is
% often worth it! A well-tuned PID system can be surprisingly robust.
% Good luck, and don't hesitate to ask for help if you're stuck.

% 1st Note:  In the last lab, you used a PID controller to control the  
% motor speed (using information from the motor encoder).  In this lab, you 
% will be using a PID controller to help the robot follow the line (using 
% information from the IR sensing array).  As a result, you will not be 
% able to just copy and paste your exact PID code from the last lab to 
% this lab. 

% 2nd Note: Make sure you only use the values produced by the reflectance 
% array when the red LEDs on the underside of the reflectance array are on!
% When your battery is getting low, the red LEDs on the underside of the 
% reflectance array will turn off, and the numbers you get from the 
% reflectance array will no longer be reliable. If you suspect that your 
% battery is dead, grab an instructor's attention and they will swap your 
% battery for a fully charged one.

% Depending on the wiring, if you used a negative sign for any of the motor
% duty cycles in section 3 above, make sure to implement those here as well.


% First initialize the reflectance array.
nb.initReflectance();

% Get an initial reading
vals = nb.reflectanceRead();

% Set the motor offset factor (use the value you found earlier)
mOffScale = 1.2; % Start with 1 so both motors have same duty cycle.

% TUNING:
% Start very small. (Using reflectance values, the calculated error can  
% range from zero to several thousand! Think about what coefficient you 
% want multiplying by values that could get up to a thousand or so.)
kp = 10;
ki = 0;
kd = 0.5;

% Basic initialization
vals = 0;
prevError = 0;
prevTime = 0;
integral = 0;
derivative = 0;

% Determine a threshold to detect when white is detected 
% (choose a value that will be used as a threshold by all sensors to know 
% if the robot has lost the line)
whiteThresh = 200; % Max value detected for all white

% The base duty cycle "speed" you wish to travel down the line 
% (recommended value is 9)
motorBaseSpeed = 9;

tic % start time

% Use a higher duty cycle for a very brief moment to overcome the gearbox 
% force of static friction 
% (recommendation: 10, with mOffScale if needed)
nb.setMotor(1, -mOffScale * 10);
nb.setMotor(2, 10);
pause(0.03);

while (toc < 5)  % Adjust me if you want to stop your line following 
                 % earlier or let it run longer.

    % TIME STEP
    dt = toc - prevTime; % find the amount of time that has elapsed

    prevTime = toc; % set the current time to the previous time in the 
                    % next time step

    % take a reading                
    vals = nb.reflectanceRead()

    % change from a struct to a list for convenience
    vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    
    % Calibrate sensor readings
    calibratedVals = zeros(1,6); % initialize to zero
    for i = 1:6
        calibratedVals(i) = (vals(i) - minVals(i))/(maxVals(i) - minVals(i));
        % overwrite the calculated calibrated values if get a reading below 
        % or above our set minVals (white) or maxVals (black), respectively
        if vals(i) < minVals(i) 
            calibratedVals(i) = 0;
        end
        if vals(i) > maxVals(i) 
            calibratedVals(i) = maxVals(i);
        end
    end

    % Calculate the three errors to be used in the PID control 
    
    error = sum([-3 -2 -1 1 2 3] .* calibratedVals) / sum(calibratedVals); % Designing this error term can sometimes be just as 
                 % important as the tuning of the feedback loop (how you  
                 % set the PID controller output). In the last lab, this  
                 % error was a simple difference between two values (RPM  
                 % and the target RPM). Here, you're working with 6 sensors  
                 % that are located in different positions. You might want 
                 % to Look back at how you calculated the error in the
                 % Sensors 5 lab. Don't forget here to use the calibrated 
                 % values to calculate the error.  

    integral = integral + error * dt;

    derivative = (error - prevError) / dt;

    % Create your PID controller output here using the previously defined 
    % gain values and the three errors computed above. 
    control = kp * error + ki * integral + kd * derivative;

    % STATE CHECKING 
    if (vals(1) < whiteThresh && ...
            vals(2) < whiteThresh && ...
            vals(3) < whiteThresh && ...
            vals(4) < whiteThresh && ...
            vals(5) < whiteThresh && ...
            vals(6) < whiteThresh)

        % ALL SENSORS READ WHITE (lost tracking):
        nb.setMotor(1, 0); % stop the motors
        nb.setMotor(2, 0);
        break; % exit the while loop

     if (vals(1) > Black && ...
            vals(2) > Black && ...
            vals(3) > Black && ...
            vals(4) > Black && ...
            vals(5) > Black && ...
            vals(6) > Black)
         break out of this script


    else

        % LINE DETECTED:
        % We want to travel at a fixed speed down the line and the control 
        % should make minor adjustments that allow the robot to stay 
        % centered on the line as it moves. These should be equations and
        % not just numbers:
        m1Duty = -(motorBaseSpeed + control)
        m2Duty = mOffScale * (motorBaseSpeed - control)

        nb.setMotor(1, m1Duty);
        nb.setMotor(2, m2Duty);
    end

    prevError = error;
end
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% EMERGENCY MOTOR SHUT OFF
% If this section doesn't turn off the motors, turn off the power switch 
% on your motor carrier board.

% Clear motors
nb.setMotor(1, 0);
nb.setMotor(2, 0);

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
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