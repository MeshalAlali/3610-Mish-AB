clc
clear all

% Create an instance of the nanobot class
nb = nanobot('/dev/cu.usbmodem24101', 115200, 'serial');
%%
% Set the number of iterations for the benchmark
numIterations = 10;

% Benchmark digitalRead operation
start = tic;
for i = 1:numIterations
    nb.digitalRead('D2'); % Replace with the desired pin number
end
elapsedTime = toc(start);
digitalReadFreq = numIterations / elapsedTime;

% Benchmark analogRead operation
start = tic;
for i = 1:numIterations
    nb.analogRead('A1'); % Replace with the desired pin number
end
elapsedTime = toc(start);
analogReadFreq = numIterations / elapsedTime;

% Benchmark digitalWrite operation
start = tic;
for i = 1:numIterations
    nb.digitalWrite('D3', 1); % Replace with the desired pin number and value
end
elapsedTime = toc(start);
digitalWriteFreq = numIterations / elapsedTime;

% % Benchmark digitalWrite operation
start = tic;
for i = 1:numIterations
    vals = nb.accelRead();
end
elapsedTime = toc(start);
accelReadFreq = numIterations / elapsedTime;

% Benchmark encoder reads
start = tic;
for i = 1:numIterations
    val = nb.encoderRead(1);
end
elapsedTime = toc(start);
encoderReadFreq = numIterations / elapsedTime;

% Benchmark a PD loop
start = tic;
kp = 0.01;
kd = 0.001;
lastval = 0;
for i = 1:numIterations
    val = nb.encoderRead(1);
    val = val.countspersec * 50;
    val_delta = val - lastval;
    control = round(kp * val - kd * val_delta);
    control = min(max(control,-100),100);
    nb.setMotor(1,control);
    lastval = val;
end
elapsedTime = toc(start);
pidLoopFreq = numIterations / elapsedTime;

% Display the results
disp(['Digital Read Frequency: ', num2str(digitalReadFreq), ' Hz']);
disp(['Analog Read Frequency: ', num2str(analogReadFreq), ' Hz']);
disp(['Digital Write Frequency: ', num2str(digitalWriteFreq), ' Hz']);
disp(['Accelerometer Read Frequency: ', num2str(accelReadFreq), ' Hz']);
disp(['Encoder Read Frequency: ', num2str(encoderReadFreq), ' Hz']);
disp(['PID Loop Frequency: ', num2str(pidLoopFreq), ' Hz']);

% Close and clear the nanobot instance
delete(nb);
clear('nb');