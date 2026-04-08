%%%%%%%%%%%%%
% ECE 3610
% INTEL LAB 3 -- Perceptron Classification
%%%%%%%%%%%%%
% In this lab, we will use a perceptron to classify gestures.  Perceptron 
% classification is advantageous over LDA when the data is not linearly 
% separable (it can use non-linear decision boundaries).  This gives us 
% more flexibility in separating out the different classes (digits). On 
% the other hand, the advantages of LDA are that it is generally faster 
% and it often works better with high-dimensional data.

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or port_detector.m. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!
clear; clc; close all; %initialization

% for Mac:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');

nb.ledWrite(0); % make sure the LED is off

%% 2. Specify initial parameters:
trialCount = 12; % Specify how many times you will gesture each digit.
                 % This needs to be fairly large because we will be
                 % splitting up the data into training and testing data.
                 % Must be divisible by 4 because 3/4 will be used for
                 % training and 1/4 will be used for testing.
if (mod(trialCount,4) ~= 0) % check that trialCount is divisible by 4.
    error("trialCount must be divisible by 4");
end
digits = 0:4; % Specify which digits you will be gesturing.
              % Start with 0 through 4, but you can add more after trying 
              % these!2
numreads = 150; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
vals = zeros(3,numreads);

%% 3. Collect Multiple Gestures
% For all of these intel labs, you can hold the dowel however you want.  
% However, you should try to be consistent in how you hold it from one lab 
% to the next.
digitCount = length(digits); % determine the number of digits
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

        % Gesture is performed during the segement below
        for i = 1:numreads
            val = nb.accelRead();
            vals(1,i) = val.x;
            vals(2,i) = val.y;
            vals(3,i) = val.z;
        end

        nb.ledWrite(0);   %Turn off the LED to signify 
    % end of recording data      
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
% later if desired.  The file will probably be saved in the current folder.
if menu("Would you like to save the dataset you just recorded?", "Yes", "No") == 1
    t = clock;
    filename = sprintf("%d%d%d_%d%d%d_TrainingSet_%dDigits%dTrials", ...
        t(1),t(2),t(3),t(4),t(5),round(t(6)),height(data),width(data)-1);
    save(filename, "data");
end

%% 4. (OPTIONAL, AS NEEDED) 
% Once you have a good set of training data for this lab, as needed you 
% can reload that data from the corresponding file here.  It will look in 
% the current directory for the file.
clear; clc; close all; %initialization
filename = "2026323_175422_TrainingSet_5Digits12Trials.mat";  % add the directory before the filename 
                                 % if needed
data = importdata(filename);

%% 5. Calculate features for each "image"
%determine digitCount and trialCount based on data size
digitCount = height(data); %number of digits is the number of rows (height)
trialCount = width(data)-1; %number of trials is the number of columns (width)
Features = zeros(digitCount, trialCount, 3); % 3 because the accelerometer sends 3 axes of data
for a = 1:digitCount %iterate through all digit
    for b = 1:trialCount %iterate through all trials
        singleLetter = data{a,b+1}; %get the individual gesture data   
        
     Features(a,b,1) = mean(singleLetter(2,:));   % x
    Features(a,b,2) = mean(singleLetter(3,:));   % y
    Features(a,b,3) = var(singleLetter(3,:));     % z
    end
end

%% 6. Plot features
% Now with more digits, is it harder to separate the features of the digits 
% in 3-D space using straight lines as in the last lab?  If so, the 
% perceptron classification we're using in this lab will do a better job 
% of classifying the digits, since it works better on data that is best 
% separated with non-linear boundaries.
figure(); hold on; grid on; % create plot
labels = strings(a,1); % pre-allocate to hold labels for the legend
for a = 1:digitCount
    scatter3(Features(a,:,1), Features(a,:,2), Features(a,:,3), 'filled');
    % rotate so we can see that the plot is 3-D and not 2-D
    view(-60,60)
    labels(a)=string(a); % assign labels to populate the legend
end
title("Features"); %title plot with the label
xlabel('X'); ylabel('Y'); zlabel('Z'); %label axes
legend(labels); % add a legend to the plot
hold off;

%% 7. Store Data as at Stack for Input to Neural Network
% Features are stored as a stack in a 4D array (b/c the MATLAB function 
% requries a 4D array as input; we are only using the 1st and 4 dimensions)
% Initialize to zero.
TrainingFeatures = zeros(3,1,1,digitCount*trialCount); 
%labels are stored as a 1D array, initialize to zero
labels = zeros(1,digitCount*trialCount); 

k=1; %simple counter
for a = 1:digitCount %iterate through digits
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = Features(a,b,:); %put each feature into image stack
        labels(k) = data{a,1}; %put each label into label stack
        k = k + 1; %increment
    end
end
labels = categorical(labels); %convert labels into categorical

%% 8. Split Training and Testing Data
%selection is an array that will hold the value of 1 when that data is 
%selected to be training data and a 0 when that data is selected to be 
%testing data
selection = ones(1,digitCount*trialCount); %allocate logical array
                                             %initialize all to 1 at first
selectionIndices = []; %initialization
for b = 1:digitCount %pick 1/4 of the data for testing
    selectionIndices = [selectionIndices,  round(linspace(1,trialCount,...
        round(trialCount/4))) + (trialCount*(b-1))];
end
selection(selectionIndices) = 0; %set logical to zero to indicate testing 
                                 %data

%training data
xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on
%testing data
xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on

%% 9. Define Neural Network

[inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

%%%%%%%%%%%%%%%%%%%%% YOU SHOULD MODIFY THESE PARAMETERS %%%%%%%%%%%%%%%%%%%

learnRate = 6; % how quickly network makes changes and learns
maxEpoch = 100; % how long the network learns (how many times all the data 
               % is passed through the CNN)

%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%%

layers= [ ... %NN architecture for a simple perceptron
    imageInputLayer([inputsize1,inputsize2,1])
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

options = trainingOptions('sgdm','InitialLearnRate', learnRate, ...
    'MaxEpochs', maxEpoch, 'Shuffle','every-epoch','Plots', ...
    'training-progress', 'ValidationData',{xTest,yTest}); %options for NN

%% 10. Train Neural Network
% This section will create a plot of the training progress.  Before going 
% on to the next section, check the validation accuracy. If the results 
% don't look good yet, go back to the previous section and adjust the 
% learnRate and maxEpoch. 
[myNeuralNetwork,info] = trainNetwork(xTrain,yTrain,layers,options); 
                         %output is the trained NN

%% 11. Test Neural Network
%  Now we will test the neural network. Check that the confusion matrix is
%  good enough.
t = 1:length(info.TrainingAccuracy);
figure();
subplot(2,2,1);
plot(info.TrainingAccuracy,'LineWidth',2,'Color',"#0072BD");
hold on;
plot(t(~isnan(info.ValidationAccuracy)), ...
    info.ValidationAccuracy(~isnan(info.ValidationAccuracy)),'--k', ...
    'LineWidth',2,'Marker','o');
title("Training Accuracy")
legend("Training Accuracy","Validation Accuracy");
xlabel("Iterations");
ylabel("Accuracy (%)");

subplot(2,2,3);
plot(info.TrainingLoss,'LineWidth',2,'Color',"#D95319");
hold on;
plot(t(~isnan(info.ValidationLoss)), ...
    info.ValidationLoss(~isnan(info.ValidationLoss)),'--k', ...
    'LineWidth',2,'Marker','o');
title("Training Loss")
legend("Training Loss","Validation Loss");
xlabel("Iterations");
ylabel("Root Mean Square Error (RMSE)");

predictions = classify(myNeuralNetwork, xTest)'; %classify testing data using NN
disp("The Neural Network Predicted:"); disp(predictions); %display predictions
disp("Correct Answers"); disp(yTest); % display correct answers
subplot(2,2,[2,4]); confusionchart(yTest,predictions); % plot a confusion matrix
title("Confusion Matrix")

%% 12. View Neural Network

figure(); plot(myNeuralNetwork); % visualize network connections
disp(myNeuralNetwork.Layers); % view layers
disp(myNeuralNetwork.Layers(2)); % view fully connect layer
disp(myNeuralNetwork.Layers(2).Weights); % view weights for each layer
disp(myNeuralNetwork.Layers(2).Bias); % view offset for each layer

%% 13. Run-Time Predictions 
% (This is essentially a copy of the lab 1 code with determination 
% replaced with NN code.)
% Perform a gesture and see if the perceptron classification works! 
% Rerun this part of the code and try testing all digits you are trying 
% to classify.

% make sure NN exists
if(~exist('myNeuralNetwork'))
    error("You have not yet created your neural network! Be sure you" + ...
        " run this section AFTER your neural network is created.");
end

% clear the old singleLetter and nb
clear nb singleLetter;

%%%%%%%%%%%%%%%%%%%%%%%%% RECONNECT TO ARDUINO %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add your port below (same as at the beginning of the code)
% for PC:
nb = nanobot('/dev/cu.usbmodem1101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%%%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nb.ledWrite(0); % turn off the LED

numreads = 200; % about 2 seconds (on serial); adjust as needed, but we 
                % will be using a value of 150 for Labs 4 and 5
pause(.5);

clc; % clear the command line
countdown("Beginning in", 3);
disp("Make A Gesture!");
nb.ledWrite(1);  % Turn on the LED to signify the start of recording data

% Gesture is performed during the segement below
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

nb.ledWrite(0); % Turn the LED off to signify end of recording data

singleLetter = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,1,1,1); %allocate the size

%%%%%%%%%%%%%%%%%%%%% YOU SHOULD MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%

xTestLive(1,:,:,1) = mean(singleLetter(2,:));
xTestLive(2,:,:,1) = mean(singleLetter(3,:));
xTestLive(3,:,:,1) = var(singleLetter(3,:));

% xTestLive(1,:,:,1) = mean(singleLetter(1,:)); % YOU SHOULD MODIFY THIS LINE TO MATCH THE
%                                 % TYPE OF TRAINING USED ABOVE (FEATURES)
% xTestLive(2,:,:,1) = range(singleLetter(2,:)); % YOU SHOULD MODIFY THIS LINE TO MATCH THE
%                                 % TYPE OF TRAINING USED ABOVE (FEATURES)
% xTestLive(3,:,:,1) = range(singleLetter(2,:)); % YOU SHOULD MODIFY THIS LINE TO MATCH THE
                                % TYPE OF TRAINING USED ABOVE (FEATURES)

%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%%

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);

% Plot with label
figure(); plot(singleLetter', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
title("Classification:", string(prediction)); %title plot with the label

%% 14. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
close all

%%
nb.livePlot('accel')