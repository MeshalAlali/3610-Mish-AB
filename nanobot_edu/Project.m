%% ===================== Setup ====================== %%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot
useWand = 0;

if useWand
    myNeuralNetwork = gesture_learn(); % train gesture classifier
end
%% =================== Main Loop ==================== %%
init(); % init sensors

% ORDER
if useWand == 1
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
        stop_driving(); % stop_driving

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
        stop_driving(); % stop_driving

% --------------------- Testing --------------------- %
    case 2
        %test stuff

        turn_right();
end

%% ================= Line Following ================= %%

function follow_line()

    minVals = [70,70,70,70,70,70]; % Set me to min reflectance 
    maxVals = [900,900,900,900,900,900]; % Set me to max reflectance 

    nb = get_nb();
    Kp = 8; 
    Ki = 0; 
    Kd = 0.5;
    integral = 0; % i state
    prevError = 0; % last error
    prevTime = 0; % last time

    tic
    pause(0.03); % avoid tiny dt

    % PID LOOP
    while toc < 99 % runtime cap
        dt = toc - prevTime;
        prevTime = toc;

        if dt <= 0
            continue
        end

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

        if all_black(vals) || all_white(vals)
            stop_driving();
            break
        end

        % PID
        error = sum([-3 -2 -1 1 2 3] .* calibratedVals) / sum(calibratedVals); % Designing this error term can sometimes be just as 

        integral = integral + error * dt; % i update
        derivative = (error - prevError) / dt; % d term
        control = Kp * error + Ki * integral + Kd * derivative; % pid out
        prevError = error;

        % STEERING
        left = 8 - control ;
        right = 8 + control;

        drive(left, right);
    end

    drive(0, 0);
end

%% ---------------- Test Line Follow ---------------- %%

init(); % init sensors
follow_line(); % test line follow
spin_find_line(1);
follow_line();

%% ================= Wall Following ================= %%

function follow_wall()
    nb = get_nb();
    Kp = 2; 
    Ki = 0; 
    Kd = 10;
    integral = 0; % i state
    prevError = 0; % last error
    prevTime = 0; % last time

    sawAllWhite = false; % state flag

    sideTarget = 500; % wall target
    cylinderDetect = 500; % cylinder detect distance (left sensor)

    % % DRIVE TO CYLINDER
    % drive(9, 9, 0.03); % wake motors
    % while true
    %     if cm(nb.ultrasonicRead2()) <= cylinderDetect % cylinder in range (left sensor)
    %         break
    %     end
    %     drive(9, 9); % forward
    % end
    % drive(0, 0);
    % pause(0.10); % settle
    % 
    % % TURN RIGHT
    % %turn90();
    % turn90_timed();
    % pause(0.10); % settle
    % 
    % drive(9, 9, 0.03); % wake motors

    % WALL LOOP
     tic
     pause(0.03); % avoid tiny dt
     while toc < 10 % runtime cap
    %     dt = toc - prevTime;
    %     prevTime = toc;
    % 
    %     if dt <= 0
    %         continue
    %     end
    % 
    %     % vals = nb.reflectanceRead();
    %     % vals = [vals.one, vals.two, vals.three, vals.four, vals.five, vals.six];
    %     % 
    %     % if ~sawAllWhite
    %     %     if all_white(vals)
    %     %         sawAllWhite = true;
    %     %     end
    %     % else
    %     %     if some_black(vals)
    %     %         break
    %     %     end
    %     % end
    % 
    %     sideDistanceRaw = clip(nb.ultrasonicRead2(),0,1000);
    %     sideDistance = cm(sideDistanceRaw); % raw to distance (side sensor)
    % 
    %     error = (sideTarget - sideDistance)/sideTarget ; % side target
    % 
    %     integral = integral + error * dt; % i update
    %     derivative = (error - prevError) / dt; % d term
    %     control = Kp * error + Ki * integral + Kd * derivative % pid out

          sideDistanceRaw = clip(nb.ultrasonicRead2(),0,2000);
          control = clip(interp1([0,500,2000,200],[12,9,9,0],sideDistanceRaw),0,9)
          leftControl = clip(interp1([0,500],[1,0],sideDistanceRaw),0,9)

          if control == NaN
              control = 0;
          end
          if leftControl == NaN
              leftControl = 0;
          end
    % 
    %     prevError = error;
    %

          left = 9 + 2; %remove control if getting too close
          right = 9 + control; 
          drive(right, left);
     end
    % 
     drive(0, 0);
end

%% ---------------- Test Wall Follow ---------------- %%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot
init(); % init sensors


follow_wall(); % test wall follow

%%
stop_driving()

%%
clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot

init();
while(1)
    nb.ultrasonicRead2
end
%% ================= Color Sensing ================== %%

function isRed = color_sense()
    nb = get_nb();
    pause(0.10); % settle
    [r, ~, b] = nb.colorRead(); % read color on line
    drive(0, 0);

    if r > b
        isRed = 1; % red
    else
        isRed = 0; % blue
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
    nb.initUltrasonic2('D5','D4');
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

    if nargin < 3
        nb.setMotor(1, leftSpeed);
        nb.setMotor(2, rightSpeed);
    else
        nb.setMotor(1, leftSpeed);
        nb.setMotor(2, rightSpeed);
        pause(duration);
        nb.setMotor(1, 0);
        nb.setMotor(2, 0);
    end
end
%%
drive(-10,-10,1)



%%

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
            drive(9, -9); % rotate right (CW)
        else
            drive(-9, 9); % rotate left (CCW)
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
spin_find_line(0)

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

    % SPEED - high enough to break static friction in pivot
    turnSpeed = 12; % max duty for pivot
    slowSpeed = 10; % slow approach near target
    slowZone = 0.25; % last 25% of turn slows down

    % FLUSH ENCODERS (drop stale counts)
    nb.encoderRead(1);
    nb.encoderRead(2);

    totalCounts = 0;

    % START PIVOT (no duration so motors stay on)
    if angle > 0
        drive(turnSpeed, -turnSpeed); % right (CW)
    else
        drive(-turnSpeed, turnSpeed); % left (CCW)
    end

    tic
    % TURN LOOP
    while toc < 10 % safety timeout
        enc1 = nb.encoderRead(1);
        enc2 = nb.encoderRead(2);
        totalCounts = totalCounts + (abs(enc1.counts) + abs(enc2.counts)) / 2; % avg both wheels

        if totalCounts >= targetCounts % reached target
            break
        end

        % SLOW DOWN NEAR TARGET to reduce overshoot
        if totalCounts >= targetCounts * (1 - slowZone)
            if angle > 0
                drive(slowSpeed, -slowSpeed);
            else
                drive(-slowSpeed, slowSpeed);
            end
        end
    end

    drive(0, 0);
end

%% ----------------- Turn 180 Right ----------------- %%
function turn180_right(), encoder_turn(180); end
function turn180_right_timed(), drive(9, -9, 2); end % old manual backup
%%  Test  %%
turn180_right()

%% ----------------- Turn 180 Left ------------------ %%
function turn180_left(), encoder_turn(-180); end
function turn180_left_timed(), drive(-9, 9, 2); end % old manual backup
%%  Test  %%
turn180_left()

%% ----------------- Turn 90 Right ------------------ %%
function turn90(), encoder_turn(-90); end
function turn90_timed(), drive(-12, 10, 0.6); end % old manual backup
%%  Test  %%
turn90_timed()

%% ------------------- Turn Right ------------------- %%
function turn_right(), encoder_turn(45); end
function turn_right_timed(), drive(9, 0, 0.4); end % old manual backup
%%  Test  %%
turn_right()

%% ------------------- Turn Left -------------------- %%
function turn_left(), encoder_turn(-45); end
function turn_left_timed(), drive(0, 9, 0.4); end % old manual backup
%%  Test  %%
turn_left()

%% -------------------- Straight -------------------- %%
function straight(), drive(9, 9, 0.4); end
%%  Test  %%
straight()
%% -------------------- Stop -------------------- %%
function stop_driving(), drive(0, 0); end
%%  Test  %%
stop_driving()

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
    stop_driving(); % stop_driving motors
    delete(nb); % close robot
    clear('nb'); % clear robot
end
clear all

%% ============ Emergency Motor Shut Off ============ %%

stop_driving(); % stop_driving motors

%% ===================== Setup ====================== %%

clc
clear all
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial'); % robot
