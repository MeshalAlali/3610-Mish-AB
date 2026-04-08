%%%%%%%%%%%%%
% ECE 3610
% INTEL LAB 2 -- Intro to Machine Learning & Feature Generation
%%%%%%%%%%%%%
% We will use Linear Discriminate Analysis (LDA) to classify between 
% gesturing a zero or one. Similar to the last lab, we will need to 
% choose what is important in the data (i.e. choose the features, such as 
% range, max, min, etc.).  But this time, LDA will analyze those features 
% make the classification decision for us.

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or port_detector.m. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clear; clc; close all; %initialization

clear nb;

% for PC:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

nb.ledWrite(0);  % make sure the LED is off

%% 2. Specify initial parameters:
trialCount = 10; % specify how many times you will gesture each digit
digits = [0, 1]; % specify which digits you will be gesturing
numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5

%% 3. Collect Multiple Gestures
% For all of these intel labs, you can hold the dowel however you want.  
% However, you should try to be consistent in how you hold it from one lab 
% to the next.
digitCount = length(digits); % determine the number of gestures
data = cell(digitCount, trialCount+1); % preallocate cell array
for i = 1:digitCount
    
    data{i,1} = digits(i);  % create a cell matrix to store the results
                            % for each trialCount of each digit
end

clc; % clear the command line

for a = 1:digitCount % iterate through all the digits
    
    fprintf(['Press return when you are ready to gesture ' ...
        'the %d''s\n'], digits(a));
    pause; % wait for a return from the user
    %display a countdown to prep user to move accelerometer 
    countdown("Beginning in", 3); 
    
    b = 1; %index for trials
    while b <= trialCount % iterate through all the trials
        % Displays the prompt in the command window
        fprintf("Draw a %d (%d of %d)",digits(a), b, trialCount); 
        % Turn on the LED to signify the start of recording data
        nb.ledWrite(1); 

        % Gesture is performed during the segment below
        for i = 1:numreads
            val = nb.accelRead();
            vals(1,i) = val.x;
            vals(2,i) = val.y;
            vals(3,i) = val.z;
        end

        nb.ledWrite(0);  %Turn off the LED to signify end of recording data  
        try
            % put the 3-axis (x,y,z) accel data of length numreads into 
            % the cell array at location a,b+1 (b=1 is reserved for 
            % recording which digit it is)
            data{a,b+1} = [vals(1,:);vals(2,:);vals(3,:)]; 
            b = b + 1;
        catch
            disp("Data capture failed. Trying again in 3 seconds")
            pause(3);
        end
        clc; % clear the command line
        pause(1); % wait one second
    end
end

pause(1); % wait one second
clc; % clear command line

% Save your data to a file so that you can reload it and reuse it 
% later if desired.  The file will be saved in the current folder listed
% in the "Current Folder" panel of MATLAB.
if menu("Would you like to save the dataset you just recorded?", ...
        "Yes", "No") == 1
    % Add some parameters to the filename, so we know which is which
    t = clock;
    filename = sprintf("%d%d%d_%d%d%d_TrainingSet_%dDigits%dTrials", ...
        t(1),t(2),t(3),t(4),t(5),round(t(6)),height(data),width(data)-1);
    save(filename, "data");
end

%% 4. (OPTIONAL, AS NEEDED) 
% Once you have a good set of training data for this lab, as needed you 
% can reload that data from the corresponding file here.  MATLAB will look 
% in the current directory for the file.
clear; clc; close all; %initialization
filename = "202634_184025_TrainingSet_2Digits6Trials.mat";  % add the directory before the filename 
                                 % if needed
data = importdata(filename);

%% 5. Calculate 3 features for each "image" 
% (One "image" is the 3-axis accelerometer data for one gesture.)
% Now we will define what features we want the LDA to use to classify the 
% digits. One of the features will be applied to the x-axis accelerometer 
% data, the second will be applied to the y-axis accelerometer data, and 
% the third will be applied to the z-axis data.

%determine the number of digits and trials based on data size
digitCount = height(data); %number of digits is the number of rows (height)
trialCount = width(data)-1; %number of trials is the number of columns (width)

% Create the matrix that will store the features of the data
Features = zeros(digitCount, trialCount, 3); % 3 because the accelerometer 
                                             % provides 3 axes of data

% Cycle through all of the gestures (trials for each digit). The 
% accelerometer data for each gesture is stored in "singleLetters" as 
% defined below.  For each "singleLetter", we need to define and calculate 
% the x-axis, y-axis and z-axis features to be used by the LDA. 
for a = 1:digitCount %iterate through all digits
    for b = 1:trialCount %iterate through all trials

        singleLetter = data{a,b+1}; % get the individual 3-axis 
                                    % accelerometer data for trial "b" of 
                                    % digit "a". The resulting singleletter
                                    % is of size [3,numreads]. 

        %%%%%%%%%% CALCULATE FEATURES (YOUR CODE GOES HERE) %%%%%%%%%%%%%%%

        % All of the gestures created a lot of data (100 x, y, and z accel
        % readings for each gesture).  We are going to reduce all of that 
        % data to a small number of features (three features) for each 
        % gesture, which is easier for the LDA to work with and analyze.
        %
        % The right side of each of the three lines below creates a random
        % number.  It is a placeholder so that the code will compile.  
        % You should replace the three random numbers with the feature
        % we want to extract from the x data, the y data, and the z data, 
        % respectively.
        %
        % HINTS: Ultimately, pick the features that will help you to best 
        % distinguish between the gestures.  Some options that you can test 
        % include: max, min, range, mean, square the data, variance, root 
        % mean square, how many times the data crosses zero, etc.  You can 
        % use the same type of feature for all three x,y,z components, or
        % you can use different features for the x,y,z components.          
        % Try at least 3 different features for comparison.  
        % Note that you can re-analyze the same gesture data you already 
        % obtained, so you do not need to rerun the earlier part of the 
        % code once you have a good dataset to work with.

        Features(a,b,1) = mean(singleLetter(1,:)); % Delete rand(1,1) and replace it with 
                                     % the feature you want to extract from
                                     % the x-axis data
        Features(a,b,2) = range(singleLetter(2,:)); % feature for the y-axis data
        Features(a,b,3) = var(singleLetter(2,:)); % feature for the z-axis data

        %%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%
    end
end

%% 6. Plot features
% The features will be plotted in 3-D.  You can click on the "Rotate 3-D" 
% button on the top right side of the plot to view it from different 
% angles.  
figure(); hold on; grid on; % create plot
labels = strings(a,1); % pre-allocate to hold labels for the legend
for a = 1:digitCount
    % plot x, y, z values from features 1, 2, and 3 respectively
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled'); 
    % rotate so we can see that the plot is 3-D and not 2-D
    view(-60,60)
    labels(a)=string(a); % assign labels to populate the legend
end
title("Features"); %title plot with the label
xlabel('X'); ylabel('Y'); zlabel('Z'); %label axes
legend(labels); % add a legend to the plot
hold off;

%% 7. Perform linear discriminate analysis
%If you get an error running this section, it may be because you didn't
%install the MATLAB Statistics and Machine Learning Toolbox (e.g. it can't 
%find the fitcdiscr function).
digits = [data{:,1}];
%reshape data so that it's #observations by #features
TrainingFeatures = reshape(Features,[trialCount*digitCount,3]); 
%assign appropriate label to each observation (i.e., 0 or 1)
TrainingLabels = repmat(digits, [1, trialCount]); 
%perform LDA
LDA = fitcdiscr(TrainingFeatures,TrainingLabels); 

%% 8. Plot features & LDA
% The features and LDA are plotted in 3-D.  You can click on the 
% "Rotate 3-D" button on the top right side of the plot to view it from 
% different angles.  Rotate the plot until you can see the hyperplane that 
% separates the gesture features.  Ideally all of the features for zero are 
% on one side of the hyperplane and all of the features for one are on the 
% other side of the hyperplane.
figure(); hold on; grid on; % create plot
for a = 1:digitCount
    % plot x, y, z values from features 1, 2, and 3 respectively
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled'); 
end
limits = [xlim ylim zlim];
K = LDA.Coeffs(1,2).Const;
L = LDA.Coeffs(1,2).Linear;
f = @(x1,x2,x3) K + L(1)*x1 + L(2)*x2 + L(3)*x3;
h2 = fimplicit3(f, limits, "white"); % plots the LDA
% rotate so we can see that the plot is 3-D and not 2-D
view(-60,60)
title("Features + LDA"); %title plot with the label
xlabel('X'); ylabel('Y'); zlabel('Z'); %label axes
hold off;

%% 9. Run-Time Predictions (Test the Model) 
% (This is essentially a copy of the lab 1 code with determination 
% replaced with LDA code.)
% Perform one gesture and see if the LDA accurately predicts what it is!

% make sure NN exists
if(~exist('LDA'))
    error("You have not yet performed a LDA! Be sure you run this" + ...
        " section AFTER you have performed the LDA.");
end

% clear the old singleLetter and nb
clear nb singleLetter

%%%%%%%%%%%%%%%%%%%%%%%%% RECONNECT TO ARDUINO %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add your port below (same as at the beginning of the code)
% for PC:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nb.ledWrite(0); % turn off the LED

numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
pause(.5);

clc; % clear the command line
countdown("Beginning in", 3);
disp("Make A Gesture!");
nb.ledWrite(1); % Turn on the LED to signify the start of recording data

% Gesture is performed during the segement below
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

nb.ledWrite(0); % Turn the LED off to signify end of recording data

singleLetter = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into LDA input form
LDAinput = zeros(1,3);

%%%%%%%%%%%%%%% CALCULATE FEATURE AGAIN (YOUR CODE GOES HERE) %%%%%%%%%%%%%
% IMPORTANT:  Change only the left side of the expressions below. Don't
% just copy and paste entire lines from earlier, since you're giving values
% to LDAinput here rather than Features(a,b,:).

LDAinput(1,1) = mean(singleLetter(1,:)); % REPLACE THE RIGHT SIDE OF THESE EQUATIONS WITH
LDAinput(1,2) = range(singleLetter(2,:)); % WHAT YOU ENDED UP USING ABOVE FOR THE
LDAinput(1,3) = var(singleLetter(2,:)); % TRAINING (SO THAT THE TRAINING AND TESTING USE


%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Prediction based on NN
LDAprediction = predict(LDA,LDAinput);

% Plot with label
figure(); plot(singleLetter', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(LDAprediction)); %title plot with the label

%% 10. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
close all
clear all
%%
clear; clc; close all; %initialization
