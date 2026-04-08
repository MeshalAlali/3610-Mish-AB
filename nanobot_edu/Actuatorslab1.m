%%%%%%%%%%%%%
% ECE 3610
% LAB 8 -- Actuators 1: DC Motors and Encoders
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Feedback between sensor inputs and actuator outputs is at the core of a
% huge number of robotic and cyberphysical system applications. In this
% lab, we will explore two different methods of sensorimotor feedback with
% naive control implementations. We will build on some of the underlying
% skills later, but for now we will largely be able to use code and
% concepts from earlier labs.
%
% Deliverables:
%   - An IMU tilt sensor that controls the speed of a DC motor
%   rotation, and reverses direction depending on tilt direction
%   - A flex sensor that sets the angle of a servo motor to match the
%   bend angle.
%
% Extensions:
%   - Use the IMU tilt sensor to control the speed of the DC motor using
%   one axis, and change the angle of the servo motor using the other.
%   (Speed and steering control!)
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all

% for PC:
%nb = nanobot('', 115200, 'serial');
% for Mac:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

%% 2. DC motor speed based on IMU Tilt
%  Using readings from your onboard IMU, control the speed and direction of 
%  your DC motor based on the tilt angle of the board.  Specifically, 
%  write your code so that the motor:
%  - Stops when the board is in its starting position (perhaps horizontal)
%  - Reaches a specified max speed when the board is fully tilted in one 
%    direction you choose
%  - Reaches the same max speed but running in the opposite direction when 
%    the board is fully tileted in the direction opposite to the one you 
%    chose above. 

%  To do this, first connect your motor to one of the motor ports (pair of 
%  motor pins) on your motor carrier using the screw-down connectors (such 
%  as M1- and M1+;  unscrew each connector first, insert the wire from the 
%  side, screw it closed again, and make sure the wire is being held 
%  securely by the screw).  Take note of which motor pins you are using.

%  ** IMPORTANT! **  Don't forget to plug in the motor carrier battery and 
%  turn the motor battery switch to ON.

tic

while (toc < 20) % ADJUST ME if you want to have longer/shorter trials

    %Here is an example of taking 5 accelerometer readings, then averaging
    %each axis:
    numreads = 5;
    vals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    %Note the index, getting every column in a specific row for each axis:
    meanx = mean(vals(1,:));
    meany = mean(vals(2,:));
    meanz = mean(vals(3,:));
    
    % Implement your tilt to motor PWM code (control the speed of your 
    % DC motor based on the tilt angle of the board).  Try completing this
    % on your own first.  Then as desired, look at the hints below.

    % Put your lines of code here (you should only neeed a very small 
    % number of lines):

    % --- Choose which motor port you wired to ---
motorPort = 1;

% --- Tunables ---
maxPWM   = 60;        % keep < 100 to be gentler on battery/motor
deadband = 0.05;      % region around "flat" where motor stops

% --- Tilt proxy (pick ONE axis) ---
tilt = meanx;        

% --- Map tilt -> signed PWM (direction from sign) ---
if abs(tilt) < deadband
    pwm = 0;
else
    pwm = maxPWM * tilt;              
    pwm = max(-maxPWM, min(maxPWM, pwm)); % clamp
end

nb.setMotor(motorPort, pwm);
pause(0.05);          % don’t update too fast







    % HINT #1: It is probably useful to restrict your expected motor speed 
    % range. +/-100% duty cycle can be a bit aggressive on the motor and 
    % will drain your battery quickly.  So specify a max speed that is
    % <100%.
    % HINT #2:  You probably want to add a pause, since you don't need
    % to update the speed super often (too often).
    % HINT #3:  You can choose the tilt direction that you want to have 
    % control the speed of the motor.  For simplicity, you can choose just
    % one axis.
    % HINT #4: You can write some values to the screen so that you know the  
    % range of values you can expect from your inertial measurement 
    % unit (IMU) (you last used it in the Sensor 3 module). 
    % HINT #5:  Also think about the setMotor()'s accepted PWM values (you 
    % can look at setMotor() in nanobot_demo.m).
    % HINT #6: If the motor doesn't turn when instructed to do so, check
    % the LiPo battery level.  You can also try pushing the reset button on
    % the top of the Arduino.

end
nb.setMotor(1, 0);  % set motor speed to zero when done

%% Run me to reset the motor speed to zero 
% (if you manually interrupted the above code)

nb.setMotor(1, 0);

%% 3. Servo angle matching flex sensor
% For this section, we will try to match the bend angle of our flex sensor
% with our servo's wiper. Keep in mind, the resistance of the flex sensor 
% is likely to only change significantly between 0 and 90 degrees.  
% Remember the flex sensor is only designed to be flexed in one direction -
% away from the ink.  bending the sensor in the other direction will not
% produce reliable data and may damage the sensor.  Also, take care not to
% bend the sensor at its base, as they have a tendency to kink and fail.
% Instead, bend the middle portion of the flex sensor.
% 
% Hook up your servo to one of the servo ports (with the brown wire aligned
% to ground, which corresponds to the white stripe on the underside of the 
% board), and set up a voltage divider with your flex sensor and 10k 
% resistor. Connect it to one of your analog read pins, and use the value 
% to determine the corresponding flex sensor and set the servo angles.
tic

nb.pinMode('A1','ainput'); % for the flex sensor; look back at Sensors 1 if 
                          % you need a refresher on how to set it up


flexMax = 250;
flexMin = 50;


while (toc < 30)
    % Average 10 samples to get a steady signal
    numreads = 10;
    vals = zeros(1,numreads);
    for i = 1:numreads
        vals(i) = nb.analogRead('A1')
    end
    meanval = mean(vals);

    % PUT YOUR CODE HERE.  Try completing this on your own at first.
    % Then as desired, look at the hints below.

 % --- Clamp to expected range (Hint #2) ---
    meanval = max(flexMin, min(flexMax, meanval));

    % --- Map analog value -> servo angle 0..90 (Hint #3) ---
    angle = (meanval - flexMin) * (90 / (flexMax - flexMin));
    angle = round(angle);          % servo likes clean ints

    nb.setServo(4, angle);
    pause(0.05);




    % HINT #1:  First determine what range of values correspond to bend 
    % angles between 0 and 90 degrees:


    % HINT #2:  Once you determine the range of values corresponding to the 
    % bend angles between 0 and 90 degrees, what do you want to have happen 
    % if you happen to get a value for meanval that is just beyond the max 
    % and min values you just found:


    % HINT #3:  Knowing that your servo angle range should go between 0 and 
    % 90, convert your analog value into a servo angle, and set the
    % corresponding servo motor:



end
nb.setServo(4, 0);

%% Run me to reset servo angle to zero 
% (if you manually interrupted the above code)

nb.setServo(4,0);

%% 4. EXTENSION (optional)
%  Use the IMU to control motor speed and the servo angle at the same
%  time!  (Use the IMU tilt sensor to control the speed of the DC motor 
%  using one axis, and change the angle of the servo motor using the 
%  other).

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all


%% 
nb.pinMode('A1','ainput')
while(1)
nb.analogRead('A1')
end
