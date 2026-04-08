%% Setup %%
% connect robot
clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

%% PID Loops %%
% Setup gain variables
% Initialize integral, previous error, and previous time terms
% Initialize or perform calculations for targets/values your loop will be working with
% tic
% small delay to prevent toc from blowing up derivative term on first loop

% begin while
	% calculate dt based on toc and previous time
	% update previous time
	% take reading of data
	% Perform conditional checking whether we want to exit the loop (e.g. read all black on reflectance array during line following)
		% exit the loop if so, and perform any needed end behaviors (e.g. setting motors to zero)
	% calculate error term
	% use error to calculate/update integral and derivative terms
	% calculate control signal based on error, integral, and derivative
	% update previous error
	% calculate desired setpoint quantities from control signal
	% apply limits to setpoints if needed
	% apply setpoint to plant (e.g. setting duty cycle for motors)

% line pid setup
lineKp = 7; % p gain
lineKi = 0; % i gain
lineKd = 0.6; % d gain

% pid state
lineIntegral0 = 0; % start i
linePrevError0 = 0; % start error
linePrevTime0 = 0; % start time

% line target
lineTarget = 0; % line center
lineMotorBaseSpeed = 9; % base speed
lineMaxMotor = 12; % motor cap

%% Gesture Classification %%
% initialize variable to track whether we’ve given valid input
% define list of possible output values, in the order that the network was trained on
% while we haven’t given valid input
	% perform gesture classification as we had done in labs 4 and 5 (outputs index of classified output according to the output value list)
	% let user know what was gesture was classified as, and ask to verify
	% perform gesture classification again
	% check if the first gesture was verified according to the second gesture.
		% if so, set valid input variable such that we exit the loop on the next iteration
		% if not, proceed to the next iteration of the loop

%% Line Following %%
% line follow setup
lineMinVals = [70,70,70,70,70,70];
lineMaxVals = [900,900,900,900,900,900];

lineMOffScale = 1.2; % motor offset

lineWhiteThresh = 200; % white thresh
lineBlackThresh = 700; % black thresh

lineRuntime = 99; % run time

% run line follow
followLine(nb, lineMinVals, lineMaxVals, lineMOffScale, ...
    lineKp, lineKi, lineKd, lineIntegral0, linePrevError0, ...
    linePrevTime0, lineTarget, lineMotorBaseSpeed, ...
    lineWhiteThresh, lineBlackThresh, lineMaxMotor, lineRuntime);

%% Wall Following %% 
%(True distance-maintaining)
% Set up PID loop
	% Check for all black condition (made a complete circle)
	% Calculate error as the difference between the distance you intend to follow and the measured ultrasonic distance
	% Use prior values of error and/or motor setpoints/encoder rates to determine if the robot is reading a point in front of or behind it, and correct the control signal/motor setpoints accordingly to stabilize.

%OR

%% (Hacky solution)%% 
% Move forward while reading sensors
	% If reflectance is all black, exit loop
	% if the side ultrasonic reading exceeds a certain value
		% turn by a specified amount (or turn slowly, reading the ultrasonic until you read a new value greater than or equal to the current exceeded value), then proceed

% wall follow setup
wallMOffScale = 1.2; % motor offset
wallMotorBaseSpeed = 8; % base speed
wallTurnAdjust = 2; % turn adjust
wallTurnDuty = 8; % turn speed
wallMaxMotor = 11; % motor cap

% wall thresholds
wallBlackThresh = 700; % stop thresh
frontStopCm = 10; % front stop
wallNearThresh = 1.5; % near band
wallFarThresh = 1.5; % far band
wallRuntime = 60; % run time

usScale = 0.01615; % us scale

% run wall follow
followWallHacky(nb, wallMOffScale, wallMotorBaseSpeed, wallTurnAdjust, ...
    wallTurnDuty, wallBlackThresh, frontStopCm, wallNearThresh, ...
    wallFarThresh, wallMaxMotor, usScale, wallRuntime);

%% Odometry %% 
%(Straight-line)
% Set up PID loop
	% Using measurements of wheel geometry, calculate the needed encoder counts that each motor needs to go
	% Define a default speed for the robot to move at
	% Calculate the error as being the difference between the motor encoder rates (either countspersec or counts divided by a measured dt)
	% Keep track of the total number of encoder counts for one or both motors and exit the loop and stop the motors once you reach the number of encoder counts needed to travel a certain distance.
		% Alternatively, you could just drive and continue until another condition is met (e.g. the RGB sensor reads red)

%(Angular)
% Set up PID loop
	% Using measurements of wheel geometry, calculate the needed encoder counts that each motor needs to go
	% Similarly to the straight line case, we can monitor the encoder counts/rates until we match the needed value, or come within a certain threshold of it.

%% Helpers %%
% line helpers

%followLine(): follows the line using reflectance pid.
function followLine(nb, minVals, maxVals, mOffScale, kp, ki, kd, ...
    integral0, prevError0, prevTime0, targetVal, motorBaseSpeed, ...
    whiteThresh, blackThresh, maxMotor, runtime)
    % pid state
    integral = integral0; % i term
    prevError = prevError0; % last error
    prevTime = prevTime0; % last time

    % sensor init
    nb.initReflectance(); % init reflectance

    % motor wake
    kickMotors(nb, mOffScale, 10, 0.03); % break static friction

    % loop timer
    tic % start timer
    pause(0.03); % avoid tiny dt

    % control loop
    while toc < runtime
        dt = toc - prevTime; % step time
        prevTime = toc; % save time

        if dt <= 0
            continue
        end

        vals = readReflectanceVec(nb); % read sensors

        % state checks
        if allDark(vals, blackThresh) % stop marker
            setMotorsToZero(nb);
            break
        end

        if allLight(vals, whiteThresh) % lost line
            turnTillLine(nb, prevError, whiteThresh, 8); % find line
            attemptCenter(nb, mOffScale, whiteThresh, 4); % center line
            continue
        end

        % reflectance calc
        calibratedVals = calibrateReflectance(vals, minVals, maxVals); % scale vals

        if sum(calibratedVals) <= 0
            continue
        end

        % pid calc
        linePos = sum([-3 -2 -1 1 2 3] .* calibratedVals) / sum(calibratedVals); % line pos
        error = linePos - targetVal; % line error
        [control, integral, prevError] = pidStep(error, integral, prevError, dt, kp, ki, kd); % pid step

        % motor update
        m1Duty = -(motorBaseSpeed + control); % left motor
        m2Duty = mOffScale * (motorBaseSpeed - control); % right motor

        m1Duty = clampVal(m1Duty, -maxMotor, maxMotor); % clamp m1
        m2Duty = clampVal(m2Duty, -maxMotor, maxMotor); % clamp m2

        nb.setMotor(1, m1Duty); % set m1
        nb.setMotor(2, m2Duty); % set m2
    end

    setMotorsToZero(nb); % stop
end

%followWallHacky(): follows wall using simple distance thresholds.
function followWallHacky(nb, mOffScale, motorBaseSpeed, turnAdjust, ...
    turnDuty, blackThresh, frontStopCm, nearThresh, ...
    farThresh, maxMotor, usScale, runtime)
    % sensor init
    nb.initReflectance(); % init reflectance
    nb.initUltrasonic1('D2','D3'); % init front us
    nb.initUltrasonic2('D4','D5'); % init side us

    % wall start
    kickMotors(nb, mOffScale, 10, 0.03); % break static friction
    approachWall(nb, frontStopCm, mOffScale, motorBaseSpeed, usScale); % go to wall

    turnRight90(nb, turnDuty, 0.42); % face wall path
    pause(0.10); % settle

    targetDist = nb.ultrasonicRead2() * usScale; % lock wall distance

    % loop state
    tic % start timer
    darkReadCount = 0; % reflectance divider

    % wall loop
    while toc < runtime
        darkReadCount = darkReadCount + 1; % count loops

        % stop check
        if darkReadCount >= 3
            vals = readReflectanceVec(nb); % read line

            if allDark(vals, blackThresh) % done marker
                setMotorsToZero(nb);
                break
            end

            darkReadCount = 0; % reset count
        end

        % distance check
        sideDist = nb.ultrasonicRead2() * usScale; % side distance

        % motor choose
        if sideDist > targetDist + farThresh % too far
            m1Duty = -(motorBaseSpeed - turnAdjust); % pull in
            m2Duty = mOffScale * (motorBaseSpeed + turnAdjust);
        elseif sideDist < targetDist - nearThresh % too close
            m1Duty = -(motorBaseSpeed + turnAdjust); % push out
            m2Duty = mOffScale * (motorBaseSpeed - turnAdjust);
        else
            m1Duty = -motorBaseSpeed; % straight
            m2Duty = mOffScale * motorBaseSpeed;
        end

        m1Duty = clampVal(m1Duty, -maxMotor, maxMotor); % clamp m1
        m2Duty = clampVal(m2Duty, -maxMotor, maxMotor); % clamp m2

        nb.setMotor(1, m1Duty); % set m1
        nb.setMotor(2, m2Duty); % set m2
    end

    setMotorsToZero(nb); % stop
end

%readReflectanceVec(): reads reflectance values into a vector.
function vals = readReflectanceVec(nb)
    vals = nb.reflectanceRead(); % raw read
    vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six]; % pack vals
end

%calibrateReflectance(): scales reflectance values to 0 to 1.
function calibratedVals = calibrateReflectance(vals, minVals, maxVals)
    % init vals
    calibratedVals = zeros(1, 6); % init vals

    % normalize loop
    for i = 1:6
        calibratedVals(i) = (vals(i) - minVals(i)) / (maxVals(i) - minVals(i)); % normalize

        if calibratedVals(i) < 0
            calibratedVals(i) = 0; % floor
        elseif calibratedVals(i) > 1
            calibratedVals(i) = 1; % cap
        end
    end
end

%pidStep(): updates pid terms and returns control.
function [control, integral, prevError] = pidStep(error, integral, prevError, dt, kp, ki, kd)
    % pid math
    integral = integral + error * dt; % update i
    derivative = (error - prevError) / dt; % update d
    control = kp * error + ki * integral + kd * derivative; % pid out
    prevError = error; % save error
end

%turnTillLine(): turns in place (center of mass doesn’t move) until the reflectance sensor reads a  line detected condition.
function turnTillLine(nb, turnDir, whiteThresh, turnDuty)
    % search loop
    while true
        vals = readReflectanceVec(nb); % read line

        if ~allLight(vals, whiteThresh) % line found
            break
        end

        if turnDir >= 0
            nb.setMotor(1, turnDuty); % turn one way
            nb.setMotor(2, turnDuty);
        else
            nb.setMotor(1, -turnDuty); % turn other way
            nb.setMotor(2, -turnDuty);
        end
    end

    setMotorsToZero(nb); % stop
end

%kickMotors(): briefly sets the motors to a high duty cycle before returning to modify the duty cycle in a function that calls it. Helps to break the static friction of the gearbox so that the motor can operate at lower duty cycles.
function kickMotors(nb, mOffScale, kickDuty, kickTime)
    nb.setMotor(1, -mOffScale * kickDuty); % kick m1
    nb.setMotor(2, kickDuty); % kick m2
    pause(kickTime); % hold kick
end

%attemptCenter(): once a line is detected under the reflectance array, kicks the motors before setting them to zero and looping to slowly center the array on the line.
function attemptCenter(nb, mOffScale, whiteThresh, turnDuty)
    % wake motors
    kickMotors(nb, mOffScale, 10, 0.02); % wake motors

    % center loop
    while true
        vals = readReflectanceVec(nb); % read line

        if allLight(vals, whiteThresh) % lost line
            break
        end

        % side sum
        leftSum = vals(1) + vals(2) + vals(3); % left side
        rightSum = vals(4) + vals(5) + vals(6); % right side

        % trim turn
        if abs(leftSum - rightSum) < 25 % centered enough
            break
        elseif leftSum > rightSum
            nb.setMotor(1, -turnDuty); % trim left
            nb.setMotor(2, -turnDuty);
        else
            nb.setMotor(1, turnDuty); % trim right
            nb.setMotor(2, turnDuty);
        end
    end

    setMotorsToZero(nb); % stop
end

%initAllSensors(): an all-in-one function to initialize the needed sensors that you can run once at the start of your program.
function initAllSensors(nb)
    nb.initReflectance(); % init reflectance
    nb.initUltrasonic1('D2','D3'); % init front us
    nb.initUltrasonic2('D4','D5'); % init side us
end

%approachWall(): drives in a straight line until the front ultrasonic sensor reads below a certain value.
function approachWall(nb, frontStopCm, mOffScale, motorBaseSpeed, usScale)
    % approach loop
    while true
        frontDist = nb.ultrasonicRead1() * usScale; % front distance

        if frontDist <= frontStopCm % close enough
            break
        end

        nb.setMotor(1, -motorBaseSpeed); % drive forward
        nb.setMotor(2, mOffScale * motorBaseSpeed);
    end

    setMotorsToZero(nb); % stop
end

%turnRight90(): turns right in place for a fixed time.
function turnRight90(nb, turnDuty, turnTime)
    nb.setMotor(1, turnDuty); % start turn
    nb.setMotor(2, turnDuty);
    pause(turnTime); % hold turn
    setMotorsToZero(nb); % stop
end

%One180(): turns in place for a larger fixed turn.
function One180(nb, turnDir, turnDuty, turnTime)
    if turnDir >= 0
        nb.setMotor(1, turnDuty); % turn one way
        nb.setMotor(2, turnDuty);
    else
        nb.setMotor(1, -turnDuty); % turn other way
        nb.setMotor(2, -turnDuty);
    end

    pause(turnTime); % hold turn
    setMotorsToZero(nb); % stop
end

%setMotorsToZero(): sets the motors on the robot to 0% duty cycle
function setMotorsToZero(nb)
    nb.setMotor(1, 0); % stop m1
    nb.setMotor(2, 0); % stop m2
end

%allDark(): checks the reflectance array values to see if all of the values are above the threshold needed to classify as “dark.” Returns true/false.
function tf = allDark(vals, blackThresh)
    tf = all(vals > blackThresh); % dark check
end

%allLight(): checks if all reflectance values are light.
function tf = allLight(vals, whiteThresh)
    tf = all(vals < whiteThresh); % light check
end

%turnOffLine(): moves the reflectance array off of a line or bar by turning in place by a certain amount or for a certain time.
function turnOffLine(nb, turnDir, turnDuty, turnTime)
    if turnDir >= 0
        nb.setMotor(1, turnDuty); % turn one way
        nb.setMotor(2, turnDuty);
    else
        nb.setMotor(1, -turnDuty); % turn other way
        nb.setMotor(2, -turnDuty);
    end

    pause(turnTime); % hold turn
    setMotorsToZero(nb); % stop
end

%clampVal(): limits a value to a range.
function out = clampVal(val, lo, hi)
    out = min(max(val, lo), hi); % clamp
end

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

% disconnect robot
clc

if exist('nb','var')
    nb.setMotor(1, 0); % stop m1
    nb.setMotor(2, 0); % stop m2
    delete(nb); % close robot
    clear('nb'); % clear robot
end

clear all % clear vars

%% EMERGENCY MOTOR SHUT OFF
% If this section doesn't turn off the motors, turn off the power switch 
% on your motor carrier board.

% Clear motors
if exist('nb','var')
    nb.setMotor(1, 0); % stop m1
    nb.setMotor(2, 0); % stop m2
end