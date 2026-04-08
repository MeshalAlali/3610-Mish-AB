clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');

%%
% LIVE PLOT

%This starts a dynamically updating plot of the accelerometer values. Click
%the "stop" button and wait for the figure to close to exit.
nb.livePlot('accel');

%You can also plot an analog read value. Note that I have to initialize the pin as an analog input ("ainput")
%first:
nb.pinMode('A2','ainput');
nb.livePlot('analog','A2');
%%
% DIGITAL READ

%First set the mode of desired pin to digital input 'dinput'
    %fyi: dinput is INPUT_PULLUP, so will return 1 by default
nb.pinMode('D10','dinput');

%Here is an example of taking a single reading:
val = nb.digitalRead('D10');
fprintf('val = %i\n', val)

%%
% ANALOG READ

%Note that I have to initialize the pin as an analog input ("ainput")
%first:
nb.pinMode('A3','ainput');

%Here is an example of taking a single reading:
val = nb.analogRead('A3');

%Here is an example of taking a second of readings, then averaging:
tic
vals = [];
while (toc < 1)
    vals(length(vals)+1) = nb.analogRead('A3');
end
meanval = mean(vals);
fprintf('mean val = %.0f\n', meanval)

%Here is an example of taking 100 readings, then averaging:
numreads = 100;
vals = zeros(1,numreads);
for i = 1:numreads
    vals(i) = nb.analogRead('A3');
end
meanval = mean(vals);
fprintf('mean val = %.0f\n', meanval)

%%
% DIGITAL WRITE

%First set the mode of the desired pin to output
nb.pinMode('A7','output');

%Then write to that pin either high (1) or low (0)
nb.digitalWrite('A7', 1);

%%
% ACCEL READ

%Here is an example of taking a single accelerometer reading:
val = nb.accelRead();

%The axis values are saved as fields in a structure:
fprintf('x: %.2f, y: %.2f, z: %.2f\n', val.x, val.y, val.z);

%Here is an example of taking 10 accelerometer readings, then averaging
%each axis:
numreads = 10;
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
fprintf('mean x: %.2f, mean y: %.2f, mean z: %.2f\n', meanx, meany, meanz);
%%
% SET MOTOR

%Set the motor with motor NUM on motor carrier, DUTY CYCLE (-100:100).
%Negative numbers cause the motor to spin in the opposite direction.
%A +/-100% duty cycle can be a bit aggressive on the motor and will drain 
%your battery quickly.
nb.setMotor(3,25) % (motor number, duty cycle)
pause(1)
nb.setMotor(,0)

%%
% ENCODER READ

%Specify which encoder to read from (e.g., HA1/HB1 or HA2/HB2)
val = nb.encoderRead(1);
fprintf('counts since last read: %i, counts per second: %i\n', val.counts,val.countspersec);

%%
% SET SERVO

%Set the servo with NUM, ANGLE (0:180)
angle = 0;
while (angle <= 180)
    nb.setServo(1,angle)
    angle = angle+20;
    pause(0.25)
end
clear angle

%%
% ONBOARD LED WRITE

tic
while (toc < 3)
    nb.ledWrite(1)
    pause(0.05)
    nb.ledWrite(0)
    pause(0.05)
end
nb.ledWrite(1)

%%
% OFFBOARD RGB LED WRITE

nb.initRGB('D12','D11','D10');

nb.setRGB(255,0,0);
pause(0.5);
nb.setRGB(0,255,0);
pause(0.5);
nb.setRGB(0,0,255);
pause(0.5);

for val = 0:25:250
    nb.setRGB(val,0,0)
    pause(0.1);
end
nb.setRGB(0,0,0);

%%
% ULTRASONIC DISTANCE

% In the final project, you'll be using two ultrasonic sensors (#1 and #2).
% If you are working on a lab that only uses one ultrasonic sensor, you can
% use just one of the functions (e.g. nb.initUltrasonic1 and 
% nb.ultrasonicRead1).

%Initialize the ultrasonic sensor with TRIGPIN, ECHOPIN
nb.initUltrasonic1('D2','D3')
nb.initUltrasonic2('D4','D5')

%Take a single ultrasonic reading
front = nb.ultrasonicRead1();
left = nb.ultrasonicRead2();

fprintf('Front dist = %0.0f   Left dist = %0.0f\n', front*0.01615, left*0.01615);
%%
% PIEZO BUZZER

%Initialize the piezo with the motor pins being used.
%Only 'M3' or 'M4' are valid for piezo use.
nb.initPiezo('M4');

%Set the piezo tone with FREQUENCY [Hz], DURATION [ms]
nb.setPiezo(600,3000);

%%
% REFLECTANCE

%Initialize the reflectance sensor array with default pins D8, D12, D11, 
% D10, A0, A1
nb.initReflectance();

%Take a single reflectance sensor reading
val = nb.reflectanceRead();

%The sensor values are saved as fields in a structure:
fprintf('one: %.2f, two: %.2f, three: %.2f four: %.2f five: %.2f six: %.2f\n', val.one, val.two, val.three, val.four, val.five, val.six);


%%
% RGB Sensor

%Initialize the RGB color sensor
nb.initColor();

%Take a single RGB color sensor reading
values = nb.colorRead();

%The sensor values are saved as fields in a structure:
red = values.red;
green = values.green;
blue = values.blue;
fprintf('red: %.2f, green: %.2f, blue: %.2f\n', red, green, blue);


%%
% CLOSING OUT

% Close and clear the nanobot instance; if you change the name from "nb",
% make sure to update the clear() function here

clear('nb');