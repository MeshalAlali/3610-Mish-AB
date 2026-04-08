%%%%%%%%%%%%%
% ECE 3610
% LAB 11 -- Encoders and PID control of a DC motor
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab you will work IN TEAMS (ideally your final project teams, but 
% this is not required for this lab) to tune a PID controller that sets a 
% DC motor duty cycle to achieve a desired wheel RPM. Low level controllers 
% like this are critical for wheeled robot motion, and will be necessary 
% for your final project!
%
% Deliverables:
%   - Target 80 RPM with a rise time under 400 ms, average steady state
%   error < 20 rpm, and oscillations < ~10 rpm.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
% Connect the encoder on the robot wheel to your motor carrier and Arduino 
% as shown in Canvas.

%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all
% for PC:
nb = nanobot('?', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% 2. Approximate the encoder's sampling frequency
% It is helpful to determine the encoder's sampling frequency, so that 
% we know how often we can take readings from the encoder without 
% experiencing aliasing artifacts. In other words, beyond a certain motor 
% speed, the encoder will not provide accurate counts. Experiment around 
% to try and find the encoder's approximate sampling frequency (above this 
% sampling frequency, your encoder will no longer provide accurate counts).

% Try to complete this section.  But if you want help at any time, you are 
% welcome to look at the hints listed at the end of this section.

val = 0; %initialize the counts to zero just in case

% We will turn on the motor and wait for it to reach a certain speed. Then 
% we will record the number of encoder counts for a certain amount of time 
% and decide if the encoder count is correct or not.  Select how long you 
% want to take samples for.  (Long run times will wear out the battery
% faster, so choose a reasonable runtime.)
runtime = '?'; % in units of seconds 

% Now we will set the motor speed.  Try different values here and work 
% towards the max motor speed (duty cycle) that you can set while still 
% obtaining accurate encoder readings: 
dutyCycle = '?'; %  If this number is too small, the motor won't be able 
         % to move. If the number is too large, you will see aliasing (the 
         % motor will be moving too fast for the encoder to keep up with the
         % required speed of the corresponding counting).
         % Try a positive number here and then also a negative number 
         % to see the impact.
nb.setMotor('?',dutyCycle); % turn on the motor
pause(1); % Wait 1 second to allow the motor to reach a constant speed
val = nb.encoderRead(1); % Initial read to set the counter to zero
pause(runtime);
val = nb.encoderRead(1); % Read again to get counts since last read
nb.setMotor('?',0); % turn off the motor

% If you look in your workspace at "val" (or type val and hit return at 
% the command line), you'll see that val stores two values.  Access the 
% first stored value (counts) using the notation: val.counts

% From the second val = nb.encoderRead(1) in the code above, we have the 
% number of counts that occurred in runtime (seconds). 

% Calculate the encoder's sampling freuency:
sampleFreq = '?'; % how many counts did you get over your runtime?

% There are about 1440 encoder counts per revolution of the wheel.  You can
% check (estimate) whether the number of counts is correct based on how 
% many times the wheel moved around vs. 1440.  If your answer does not seem
% correct, look at the hints at the end of this section.
fprintf(['For dutyCycle: %f, counts since last read: %i, '...
    'counts per second: %i\n'], dutyCycle, val.counts, sampleFreq);

% Answer these questions:

% 1. What is the relationship between the signs of the motor duty cycle  
% and the encoder count?

% 2. What is the max motor duty cycle you can use while still getting 
% accurate encoder counts? (fill this in after running this section 
% several times and finding the value)
maxDutyCycle = '?';

% HINTS, if your sampleFreq seems incorrect:

% 1st Hint:  If you want to make sure your encoder is working, you can try
% running the ENCODER READ section in nanobot_nano, move the wheel, and 
% then run the ENCODER READ section again to see if you get a suitable
% number.

% 2nd HINT: Try recording how many counts you get in one second at 
% different duty cycles. Beware, if you go too high, your motor will spin 
% too fast for your encoder to read accurately, and you will see your 
% counts per second start to decrease as you increase the duty cycle. 
% If you go even faster, your encoder may not even register any counts! 
% Your maxDutyCycle should be set to a number lower than the point at 
% which the encoder counts start to decreases and become inaccurate.

% 3rd HINT:  Hold the wheel in the same orientation every time (e.g.
% vertically). 

% 4th HINT: Start at a low duty cycle of, say, 5.  Then increase in 
% increments of between 1 and maybe 3 (you can start with incrementing by 
% 3, and then increment by values of 1 once you get close to the max value).

% 5th HINT:  Is your val.counts a positive or negative number?  Do you 
% need an absolute value somewhere?


%% Use PID tuning to match a desired revolutions per minute (RPM)
% We will turn on the DC motor and use a PID controller to get it to 
% converge as smoothly and quickly as possible to a target RPM value.  

rpm_targ = 80;  %The goal RPM.  You can also test out other RPMs.

maxDutyCycle = '?'; % use here the max value you came up with in the last 
                    % section of this lab that allowed you to obtain an
                    % accurate count (before the counts started to
                    % decrease again from their max value, i.e. you saw 
                    % aliasing).

% Look at the PID diagram in Canvas for this lab.
% Set the three PID gain values.
% Tip:  Follow this procedure: 
%    (1) Set the proportional gain to one (this makes the PID's output 
%    control value directly proportional to the error with a 1:1
%    relationship) and set the integral and derivative gains both to zero.  
%    (2) Check the response and try other proportional gain value as 
%    desired.  
%    (3) Once you see the impact of the proportional gain, start slowly
%     increasing the integral gain to see its impact (you will probably 
%     want to start with values smaller than 1, but not too small). 
%    (4) Once you see the impact of the integral gain, start 
%    slowly increasing the derivative gain (likely starting with values 
%    much much smaller than 1) to see its impact.
kp = '?';         % proportional gain (applies to the current error)
ki = '?';         % integral gain (applies to the accumulation of past 
                  % errors over time)
kd = '?';         % derivative gain (applies to the rate of change of the 
                  % current error i.e. an estimate of the future error)

integral = 0; % intialize the initial running sum value to zero
prevError = 0; %initialize the previous error value to zero

val = 0; %initialize the encoder counts to zero

% For graphing
rpms = 0; % store the current RPM as time evolves
times = 0; % store how much time has evolved each time through the while 
           % loop

% Change me to change how long the program runs (in seconds)
runtime = 3;

val = nb.encoderRead(1); % take an initial reading

tic % start time

pause(0.03); % Small delay to avoid an initial case causing dt to blow up

% Start a while loop to iterate through time. Each time we iterate through 
% this loop, we will recalculate and plot what the current RPM is of the 
% DC motor.  Very generally, as we iterate through this while loop, we want
% the motor speed to get closer to rpm_targ.
while toc < runtime

    val = nb.encoderRead(1); % Get the counts since the last time we 
                              % called it
   
    times(end+1) = toc; % Collect the latest elapsed time for graphing

    % Compute the time difference between the last encoder reading and this 
    % one (each time we iterate through this while loop, dt number of 
    % seconds has elapsed)
    dt = times(end) - times(end - 1); 

    % Calculate the current RPM (try filling this in first, and then as 
    % desired, look at the hints below):
    rpm = '?';
    

    % Hint #1: Start by finding the number of counts obtained in the amount 
    % of time that has elapsed.

    % Hint #2: There are about 1440 encoder counts per revolution of the 
    % wheel 
    
    % Hint #3: Remember, we want this result in revolutions per minute 
    % (RPM)

    % Hint #4: The encoder will count negative numbers in one direction and
    % positive numbers in the opposite direction.  You want the rpm
    % calculation to be a positive number regardless of the direction of
    % the wheel.

    rpms(end+1) = rpm; % Collect the current RPM for graphing
    
    % calculate the three errors to be used in the PID control 
    error = '?'; % Difference between our target and current rpm 
    integral = '?'; % Running sum that adds the error occuring across each 
                    % time step as time evolves (your data is discrete  
                    % rather than continuous, so we are using a simple 
                    % discretized (stair-cased) approximation of the 
                    % integral due to discretization in time, i.e. finding 
                    % the total area under the error curve). This term
                    % accounts for past values of the error.
    derivative = '?'; % Rate of change estimate using the difference
                      % between the current and prior error over the
                      % time between them (again, we are using a simple 
                      % discretized (stair-cased) approximation of the 
                      % derivative due to discretization in time). This
                      % term is an estimate of the future trend of the
                      % error.
                                         
    prevError = error; % save the current error to be used as the previous 
                       % error in the next time step

    % Create your PID controller output here using the previously defined 
    % gain values and the three errors computed above.  
    control = '?'; % Put the error terms and coefficients together
                   % into one control signal!
    
    % Caps the motor duty cycle at +/- max encoder duty cycle to prevent
    % aliasing
    if control > maxDutyCycle
        control = maxDutyCycle;
    end 
    if control < -maxDutyCycle
        control = -maxDutyCycle;
    end
   
    nb.setMotor('?', control); % Send the control signal to the motor.
end

nb.setMotor('?',0);

% Graph the RPM results vs. elapsed time
clf
hold on
plot(times,rpms)
yline(rpm_targ,'-','Target')
yline(rpm_targ+10,'--','max')
yline(rpm_targ-10,'--','min')
xline(0.4,'--')
ylim([0, rpm_targ+50])
xlim ([0 runtime])
xlabel('Fill in')
ylabel('Fill in')
hold off

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all