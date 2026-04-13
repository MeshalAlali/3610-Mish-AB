%% ===================== Setup ====================== %%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot

%%

myNeuralNetwork = gesture_learn(); % train gesture classifier

%% =================== Main Loop ==================== %%
init(); % init sensors

% ORDER
if exist('myNeuralNetwork', 'var')
    order = gesture_detect(myNeuralNetwork); % classify gesture from wand
else
    order = 2; % test mode
end

switch order
    case 0
        % LINE TASK

        straight();
        spin_find_line(0);
        follow_line(); % home
        spin_find_line(0);
        follow_line(); % home

        % WALL TASK

        straight();
        follow_line(); % home
        follow_wall();
        follow_line(); % home

        % COLOR + HOME

        straight();
        spin_find_line(1);
        spin_find_line(color_sense());
        follow_line(); % home
        stop(); % stop

    case 1
        % WALL TASK

        straight();
        spin_find_line(1);
        follow_line();
        follow_wall();
        follow_line(); % home

        % LINE TASK

        straight();
        follow_line();
        spin_find_line(0);
        follow_line(); % home

        % COLOR + HOME

        straight();
        spin_find_line(0);
        spin_find_line(color_sense())
        follow_line(); % home
        stop(); % stop

% --------------------- Testing --------------------- %
    case 2
        %test stuff

        turn_right();
end

%% ================= Line Following ================= %%

function follow_line()
    nb = get_nb();
    Kp = 10; 
    Ki = 0; 
    Kd = -0.5;
    integral = 0; % i state
    prevError = 0; % last error
    prevTime = 0; % last time

    drive(67, 67, 0.03); % wake motors

    tic
    pause(0.03); % avoid tiny dt

    % PID LOOP
    while toc < 99 % runtime cap
        dt = toc - prevTime;
        prevTime = toc;

        if dt <= 0
            continue
        end

        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        if all_black(vals) || all_white(vals)
            stop();
            break
        end

        % PID
        calibrated = (vals - white_thresh()) / (black_thresh() - white_thresh()); % normalize
        if sum(calibrated) <= 0
            continue
        end
        error = sum([-3 -2 -1 1 2 3] .* calibrated) / sum(calibrated); % line pos

        integral = integral + error * dt; % i update
        derivative = (error - prevError) / dt; % d term
        control = Kp * error + Ki * integral + Kd * derivative; % pid out
        prevError = error;

        % STEERING
        left = 67 + control * 100/max_duty();
        right = 67 - control * 100/max_duty();

        drive(left, right);
    end

    drive(0, 0);
end

%% ---------------- Test Line Follow ---------------- %%

init(); % init sensors
follow_line(); % test line follow

%% ================= Wall Following ================= %%

function follow_wall()
    nb = get_nb();
    Kp = 20; 
    Ki = 0; 
    Kd = 4.5;
    integral = 0; % i state
    prevError = 0; % last error
    prevTime = 0; % last time

    sawAllWhite = false; % state flag

    sideTarget = 8; % wall target
    cylinderDetect = 7.5; % cylinder detect distance (left sensor)

    % DRIVE TO CYLINDER
    drive(67, 67, 0.03); % wake motors
    while true
        if cm(nb.ultrasonicRead2()) <= cylinderDetect % cylinder in range (left sensor)
            break
        end
        drive(67, 67); % forward
    end
    drive(0, 0);
    pause(0.10); % settle

    % TURN RIGHT
    %turn90();
    encoder_turn(90);
    pause(0.10); % settle

    drive(67, 67, 0.03); % wake motors

    % WALL LOOP
    tic
    pause(0.03); % avoid tiny dt
    while toc < 60 % runtime cap
        dt = toc - prevTime;
        prevTime = toc;

        if dt <= 0
            continue
        end

        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        if ~sawAllWhite
            if all_white(vals)
                sawAllWhite = true;
            end
        else
            if some_black(vals)
                break
            end
        end

        sideDistanceRaw = nb.ultrasonicRead2();
        sideDistance = cm(sideDistanceRaw); % raw to distance (side sensor)

        error = sideTarget - sideDistance; % side target
        
        integral = integral + error * dt; % i update
        derivative = (error - prevError) / dt; % d term
        control = Kp * error + Ki * integral + Kd * derivative; % pid out
        prevError = error;

        left = 67 - control * 100/max_duty(); %remove control if getting too close
        right = 67 + control * 100/max_duty();

        drive(left, right);
    end

    drive(0, 0);

end

%% ---------------- Test Wall Follow ---------------- %%

init(); % init sensors
follow_wall(); % test wall follow

%% ================= Color Sensing ================== %%

function isRed = color_sense()
    nb = get_nb();
    pause(0.10); % settle
    [r, ~, b] = nb.colorRead(); % read color on line
    drive(0, 0);

    if r > b
        isRed = 1; % red
        turn180_right(); % red -> right 180
    else
        isRed = 0; % blue
        turn180_left(); % blue -> left 180
    end
end

%% --------------- Test Color Sensing --------------- %%

init(); % init sensors
color_sense(); % test color sense


%% ==================== Helpers ===================== %%
% variables
function t = white_thresh(), t = 250; end
function t = black_thresh(), t = 800; end
function t = max_duty(), t = 12; end   % motor duty scale
function nb = get_nb(), nb = evalin('base', 'nb'); end

function init()
    nb = get_nb();
    nb.initReflectance();
    nb.initUltrasonic1('D2','D3');
    nb.initUltrasonic2('D4','D5');
    nb.initColor();
end


% reflectance checks
function r = all_white(vals), r = all(vals < white_thresh()); end
function r = all_black(vals), r = all(vals > black_thresh()); end
function r = some_black(vals), r = any(vals > black_thresh()); end

% raw to cm
function d = cm(raw)
    d = interp1([0 133 267 400 533 667 800 933 1067 1200 1333 1467 1600 1733 1867 2000], ...
                [0 2   4   6   8   10  12  14  16   18   20   22   24   26   28   30],...
                raw, 'linear', 'extrap');
end

%% ==================== Movement ==================== %%

% ------------------ Motor Drive ------------------- %
function drive(leftSpeed, rightSpeed, duration)
    nb = get_nb();
    leftDuty = leftSpeed / 100 * max_duty(); % speed → duty
    rightDuty = rightSpeed / 100 * max_duty(); % speed → duty

    if nargin < 3
        nb.setMotor(1, leftDuty);
        nb.setMotor(2, rightDuty);
    else
        nb.setMotor(1, leftDuty);
        nb.setMotor(2, rightDuty);
        pause(duration);
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
    end
end

% spin until white then black (find line)
function spin_find_line(direction)
    nb = get_nb();
    sawAllWhite = false; % state flag

    nb.initReflectance(); % sensor init

    % TURN LOOP
    while true
        vals = nb.reflectanceRead();
        vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];

        if direction == 1
            drive(67, -67); % rotate right (CW)
        else
            drive(-67, 67); % rotate left (CCW)
        end

        if ~sawAllWhite
            if all_white(vals)
                sawAllWhite = true;
            end
        else
            if some_black(vals)
                break
            end
        end
    end

    drive(0, 0);
end

%%  Test  %%
spin_find_line(1)

%% ------------------ Encoder-Based Turning ------------------ %%

% ENCODER TURN - pivot in place using encoder counts
function encoder_turn(angle)
    nb = get_nb();

    % ROBOT GEOMETRY
    wheelDiam = 4.2; % wheel diameter in cm
    wheelBase = 10.0; % distance between wheels in cm
    countsPerRev = 1440; % encoder counts per wheel revolution

    % COUNTS NEEDED
    wheelCirc = pi * wheelDiam; % wheel circumference
    arcLength = abs(angle) / 360 * pi * wheelBase; % arc each wheel travels
    targetCounts = arcLength / wheelCirc * countsPerRev; % encoder target

    % PID SETUP
    Kp = 0.15;
    Kd = 0.02;
    prevError = 0;
    totalCounts = 0;
    minDuty = 40; % keep motors from stalling
    maxTurnDuty = 80; % cap max speed

    % FLUSH ENCODERS
    nb.encoderRead(1);
    nb.encoderRead(2);

    drive(67 * sign(angle), -67 * sign(angle), 0.03); % wake motors

    tic
    pause(0.03); % avoid tiny dt

    % TURN LOOP
    while toc < 10 % safety timeout
        enc = nb.encoderRead(1); % read encoder 1
        totalCounts = totalCounts + abs(enc.counts); % accumulate

        error = targetCounts - totalCounts; % how far left to go

        if error <= 5 % close enough
            break
        end

        derivative = error - prevError; % d term
        control = Kp * error + Kd * derivative; % pd output
        prevError = error;

        % CLAMP DUTY
        duty = max(minDuty, min(maxTurnDuty, abs(control)));

        if angle > 0
            drive(duty, -duty); % turn right
        else
            drive(-duty, duty); % turn left
        end
    end

    drive(0, 0);
end

%% ----------------- Turn 180 Right ----------------- %%
function turn180_right(), encoder_turn(180); end
function turn180_right_timed(), drive(67, -67, 2); end % old manual backup
%%  Test  %%
turn180_right()

%% ----------------- Turn 180 Left ------------------ %%
function turn180_left(), encoder_turn(-180); end
function turn180_left_timed(), drive(-67, 67, 2); end % old manual backup
%%  Test  %%
turn180_left()

%% ----------------- Turn 90 Right ------------------ %%
function turn90(), encoder_turn(90); end
function turn90_timed(), drive(67, -75, 1.6); end % old manual backup
%%  Test  %%
turn90()

%% ------------------- Turn Right ------------------- %%
function turn_right(), encoder_turn(45); end
function turn_right_timed(), drive(67, 0, 0.4); end % old manual backup
%%  Test  %%
turn_right()

%% ------------------- Turn Left -------------------- %%
function turn_left(), encoder_turn(-45); end
function turn_left_timed(), drive(0, 67, 0.4); end % old manual backup
%%  Test  %%
turn_left()

%% -------------------- Straight -------------------- %%
function straight(), drive(67, 67, 0.4); end
%%  Test  %%
straight()
%% -------------------- Stop -------------------- %%
function stop(), drive(0, 0); end
%%  Test  %%
stop()

%% ==================== Gesture ===================== %%

% TRAIN NETWORK
function net = gesture_learn()
    % LOAD DATA
    [file, path] = uigetfile('10DigitsTrainingSet_MergedData.mat');
    load(fullfile(path, file));

    % PREPARE FEATURES
    digitCount = height(data);
    trialCount = width(data) - 1;
    TrainingFeatures = zeros(3, 150, 1, digitCount*trialCount);
    labels = zeros(1, digitCount*trialCount);
    k = 1;
    for a = 1:digitCount
        for b = 1:trialCount
            TrainingFeatures(:,:,:,k) = data{a,b+1};
            labels(k) = data{a,1};
            k = k + 1;
        end
    end
    labels = categorical(labels);

    % SPLIT TRAIN/TEST
    selection = ones(1, digitCount*trialCount);
    selectionIndices = [];
    for b = 1:digitCount
        selectionIndices = [selectionIndices, round(linspace(1,trialCount,round(trialCount/4))) + (trialCount*(b-1))];
    end
    selection(selectionIndices) = 0;
    xTrain = TrainingFeatures(:,:,:,logical(selection));
    yTrain = labels(logical(selection));
    xTest  = TrainingFeatures(:,:,:,~logical(selection));
    yTest  = labels(~logical(selection));

    % BUILD NETWORK
    [inputsize1, inputsize2, ~] = size(TrainingFeatures);
    numClasses = length(unique(labels));
    learnRate = 0.003;
    maxEpoch = 40;
    layers = [
        imageInputLayer([inputsize1, inputsize2, 1])
        convolution2dLayer([3,9], 16)
        batchNormalizationLayer
        reluLayer
        convolution2dLayer([1,7], 32)
        batchNormalizationLayer
        reluLayer
        fullyConnectedLayer(64)
        dropoutLayer(0.20)
        fullyConnectedLayer(numClasses)
        softmaxLayer
        classificationLayer
    ];
    options = trainingOptions('sgdm', 'InitialLearnRate', learnRate, ...
        'MaxEpochs', maxEpoch, 'Shuffle', 'every-epoch', ...
        'MiniBatchSize', 248, 'ValidationData', {xTest, yTest});
    net = trainNetwork(xTrain, yTrain, layers, options);
end

%% ================ Classify Gesture ================ %%

function g = gesture_detect(net)
    wand = nanobot('/dev/cu.usbmodem101', 115200, 'serial'); % connect wand

    numreads = 150;
    pause(0.5);
    countdown("Beginning in", 3);
    disp("Make a Gesture!");
    wand.ledWrite(1);

    vals = zeros(3, numreads);
    for i = 1:numreads
        val = wand.accelRead();
        vals(1,i) = val.x;
        vals(2,i) = val.y;
        vals(3,i) = val.z;
    end
    wand.ledWrite(0);

    % CLASSIFY
    xLive = zeros(3, 150, 1, 1);
    xLive(:,:,1,1) = vals;
    pred = classify(net, xLive);
    g = str2double(char(pred));

    delete(wand); % close wand
end

%% ------------------ Test Gesture ------------------ %%
myNeuralNetwork = gesture_learn(); % train gesture classifier
%%
gesture_detect(myNeuralNetwork)

%% =================== Disconnect =================== %%

clc
if exist('nb','var')
    drive(0, 0); % stop motors
    delete(nb); % close robot
    clear('nb'); % clear robot
end
clear all

%% ============ Emergency Motor Shut Off ============ %%

drive(0, 0); % stop motors

%% ===================== Setup ====================== %%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot
