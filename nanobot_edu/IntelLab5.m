%%%%%%%%%%%%%
% ECE 3610
% INTEL LAB 5 -- Data Augmentation and Software Engineering 2.0
%%%%%%%%%%%%%
% Today we will crowd source data from everyone in the class and
% augment it to classify all 10 digits with near perfect accuracy 
% (>90% at least).

clear; clc; close all; %initialization

%% 1. Load in Merged Training Sets
% When you run this section, it will ask you to select the file you want to
% load.  A file has been prepared for you that combines all of the data
% from everyone in the class.

[file, path] = uigetfile('10DigitsTrainingSet_MergedData.mat');
load(fullfile(path,file));

% Figure out how many gestures there are.
[h, w] = size(data);
digitCount = h;
trialCount = w-1;
digits = [data{:,1}];

%% Data Augmentation
% NOTE:  Uncomment this part only after you have run through the code 
% at least once using today's larger data set.  Use this section to add 
% even more data and further imporve the results. 

% 2nd NOTE:  If you get an error when you run this part of the code (with 
% "awgn", you need to install the communications toolbox.

% Instead of collecting even more gestures from even more people to 
% improve the accuracy of the classifications, we will create new 
% versions of the data that we already have.  We will add these to the 
% original set of data to create an even larger set of data for training 
% the CNN.

% NOTE!!! Once you uncomment this part of the code, the data set will
% be much larger, and thus the training will take much longer. Therefore,
% parameter you should change when including this section is maxEpoch.  
% You will probably want to set it equal to a value of 10 or less.
%
% Augmenting the data in this manner is especially useful whenever it's 
% not easy to collect more data.

dataaug = data;
for a = 1:digitCount
    c = 2;
    for b = 1:trialCount
        temp = data{a,b+1};
        % augment by exchanging the axes (e.g. move z data to x data and 
        % x to z) to compensate for someone holding the "wand" with a 
        % different orientation
        dataaug{a,trialCount+c} = temp([1,3,2],:);
        dataaug{a,trialCount+c+1} = temp([2,1,3],:);
        dataaug{a,trialCount+c+2} = temp([2,3,1],:);
        dataaug{a,trialCount+c+3} = temp([3,2,1],:);  % plotting when c = 2
        dataaug{a,trialCount+c+4} = temp([3,1,2],:);
        % augment by inverting axes to compensate for someone drawing the 
        % numbers backwards 
        dataaug{a,trialCount+c+5} = bsxfun(@times,temp,[-1;1;1]);
        dataaug{a,trialCount+c+6} = bsxfun(@times,temp,[1;-1;1]);
        dataaug{a,trialCount+c+7} = bsxfun(@times,temp,[1;1;-1]);
        dataaug{a,trialCount+c+8} = bsxfun(@times,temp,[1;-1;-1]);
        dataaug{a,trialCount+c+9} = bsxfun(@times,temp,[-1;-1;1]);
        dataaug{a,trialCount+c+10} = bsxfun(@times,temp,[-1;-1;-1]); % plotting when c = 2
        % augment by shifting time (add an artificial delay in case 
        % someone doesn't immediately start drawing the letter)
        dataaug{a,trialCount+c+11} = circshift(temp,5,2);
        dataaug{a,trialCount+c+12} = circshift(temp,10,2);
        dataaug{a,trialCount+c+13} = circshift(temp,15,2);
        dataaug{a,trialCount+c+14} = circshift(temp,20,2);
        dataaug{a,trialCount+c+15} = circshift(temp,25,2); % plotting when c = 2
        % augment by skewing time to compensate for people writing letters 
        % at different rates (i.e. elongate the data by trimming the ends)
        dataaug{a,trialCount+c+16} = imresize(temp(:,5:end-5), [3 150], 'bilinear');
        dataaug{a,trialCount+c+17} = imresize(temp(:,10:end-10), [3 150], 'bilinear');
        dataaug{a,trialCount+c+18} = imresize(temp(:,15:end-15), [3 150], 'bilinear'); % plotting when c = 2
        dataaug{a,trialCount+c+19} = imresize(temp(:,20:end-20), [3 150], 'bilinear');
        dataaug{a,trialCount+c+20} = imresize(temp(:,25:end-25), [3 150], 'bilinear');
        % augment by amplifying movements to compensate for people writing
        % letters of different sizes
        dataaug{a,trialCount+c+21} = temp*1.1;
        dataaug{a,trialCount+c+22} = temp*1.2; % plotting when c = 2
        dataaug{a,trialCount+c+23} = temp*1.3;
        dataaug{a,trialCount+c+24} = temp*1.4;
        dataaug{a,trialCount+c+25} = temp*1.5;
        % augment by adding (Gaussian) noise to compensate for shaking,
        % vibrations, etc.
        dataaug{a,trialCount+c+26} = awgn(temp,30,'measured');
        dataaug{a,trialCount+c+27} = awgn(temp,25,'measured');
        dataaug{a,trialCount+c+28} = awgn(temp,20,'measured');
        dataaug{a,trialCount+c+29} = awgn(temp,18,'measured');
        dataaug{a,trialCount+c+30} = awgn(temp,15,'measured'); % plotting when c = 2
        c = c + 31;
    end
end

% Plot an example of each (as  marked above) to see the impact
figure();
ax(1) = nexttile; plot(data{1,2}', 'LineWidth', 1.5); title("Original Data");
ax(2) = nexttile; ax2 = plot(dataaug{1,trialCount+5}', 'LineWidth', 1.5); title("Swapped Axis");
ax(3) = nexttile; ax3 = plot(dataaug{1,trialCount+12}', 'LineWidth', 1.5); title("Inverted Axis");
ax(4) = nexttile; ax4 = plot(dataaug{1,trialCount+17}', 'LineWidth', 1.5); title("Time Shift");
ax(5) = nexttile; ax5 = plot(dataaug{1,trialCount+20}', 'LineWidth', 1.5); title("Time Skew");
ax(6) = nexttile; ax6 = plot(dataaug{1,trialCount+24}', 'LineWidth', 1.5); title("Amplitude Amplification");
ax(7) = nexttile; ax7 = plot(dataaug{1,trialCount+32}', 'LineWidth', 1.5); title("Gaussian Noise");
linkaxes(ax, 'xy'); % plot all on the same scale, so don't autoscale to bigger waveforms
sgtitle("Example of Data Augmentation")
% shuffle the order of the data to prevent the CNN from assuming the order 
% is important during training
justData = dataaug(:,2:end);
for a = 1:digitCount
    order = randperm(length(justData));
    justData(a,order) = justData(a,:);
end
dataaug(:,2:end) = justData;
data = dataaug;

%% 2. Store Data for Neural Network
%determine digitCount and trialCount based on data size
digitCount = height(data); %number of digits is the number of rows (height)
trialCount = width(data)-1; %number of trials is the number of columns (width)
%features are stored as a stack of 3D images (4D array), initialize to zero
TrainingFeatures = zeros(3,150,1,digitCount*trialCount); 
%labels are stored as a 1D array, initialize to zero
labels = zeros(1,digitCount*trialCount); 

k=1; %simple counter
for a = 1:digitCount %iterate through digits
    for b = 1:trialCount %iterate through trials
        TrainingFeatures(:,:,:,k) = data{a,b+1}; %put data into image stack
        labels(k) = data{a,1}; %put each label into label stack
        k = k + 1; %increment
    end
end
labels = categorical(labels); %convert labels into categorical

%% 3. Split Training and Testing Data
%selection is an array that will hold the value of 1 when that data is 
%selected to be training data and a 0 when that data is selected to be 
%testing data
selection = ones(1,digitCount*trialCount); %allocate logical array
                                             %initialize all to 1 at first
selectionIndices = []; %initialization
for b = 1:digitCount %save 1/4 of the data for testing
    selectionIndices = [selectionIndices,  round(linspace(1,trialCount,round(trialCount/4))) + (trialCount*(b-1))];
end
selection(selectionIndices) = 0; %set logical to zero

%training data
xTrain = TrainingFeatures(:,:,:,logical(selection)); %get subset (3/4) of features to train on
yTrain = labels(logical(selection)); %get subset (3/4) of labels to train on
%testing data
xTest = TrainingFeatures(:,:,:,~logical(selection)); % get subset (1/4) of features to test on
yTest = labels(~logical(selection)); %get subset (1/4) of labels to test on

%% 4. Define Neural Network
% NOTE:  When working with a larger data set as you are in this lab, you'll
% find that you will want to use different values for the learnRate, 
% maxEpoch, etc. than what worked for the smaller data sets in the earlier 
% labs.

[inputsize1,inputsize2,~] = size(TrainingFeatures); %input size is defined by features
numClasses = length(unique(labels)); %output size (classes) is defined by number of unique labels

%%%%%%%%%%%%%%%%%%%%%% YOUR MODIFICATIONS GO HERE %%%%%%%%%%%%%%%%%%%%%%%%%%

% In this section, adjust learnRate and maxEpoch based on what you learned
% last time.  Also change the size of the convolution from [1,1] to
% something that works better.  You can also change the number of filters.

learnRate = 0.003; % how quickly network makes changes and learns
maxEpoch = 40; % how long the network learns (how many times all the data 
               % is passed through the CNN)

layers= [ ... %NN architecture for a conv net
    imageInputLayer([inputsize1,inputsize2,1])
    convolution2dLayer([3,9],16) % [1,1] is the size of the convolution, 
                                 % i.e. the size of the filters/kernels 
                                 % that sweep across the data.  
                                 % Change [1,1] to be your desired size for 
                                 % the filter / convolution. (Note: A 
                                 % filter of size [3x150] would extend
                                 % across all x,y,z accelerometers 
                                 % and all 150 time steps. In this case, 
                                 % there is no longer a convolution because 
                                 % the filter is the same size as the data!
                                 % The last value (initially 20) is the
                                 % number of filters (i.e. features) you
                                 % want the CNN to use.  Adjust this number
                                 % as needed.
    batchNormalizationLayer  % normalize the inputs to the next layer
    reluLayer % activation layer
    convolution2dLayer([1,7],32) % [1,1] is the size of the convolution 
                                 % Change [1,1] to be your desired size for 
                                 % the convolution.
                                 % The second value (initially 20) is the
                                 % number of filters (i.e. features) you
                                 % want the CNN to use.
    batchNormalizationLayer % normalize the inputs to the next layer
    reluLayer  % activation layer
    fullyConnectedLayer(64)
    dropoutLayer(0.20) % Rate of dropout (e.g., 0.01 = 1%). This value is 
                       % reasonable, but feel free to change it!
    fullyConnectedLayer(numClasses)
    softmaxLayer
    classificationLayer
    ];

%%%%%%%%%%%%%%%%%%%%%%%%% END OF YOUR MODIFICATIONS %%%%%%%%%%%%%%%%%%%%%

% Options for NN
% Mini-batch size is changed from 128 to 248 due to the larger data sets.
options = trainingOptions('sgdm','InitialLearnRate', learnRate, ...
    'MaxEpochs', maxEpoch,'Shuffle','every-epoch', ...
    'MiniBatchSize',248,'Plots','training-progress', ...
    'ValidationData',{xTest,yTest}); 

%% 5. Train Neural Network
% Check the accuracy here or go to the next section and check that both the 
% accuracy and the confusion matrix together are good enough before going 
% on. 

% A note about how long it takes to train your CNN in this lab:
% If maxEpoch is too small, the CNN won't be trained very well and the
% results will be poor.  Howwever, if maxEpoch is too large, the number of 
% iterations will be very large and the CNN will take a very long time to 
% train since you are working with more data this time, and the default 
% mini-batch size is 248.  If you find that the total number of iterations 
% is too large, you can decrease the number of epochs or you can set the 
% mini-batch size to something other than 248.
% From last time:
%     Epochs = number of times all of the training data passes through 
%              the network 
%     Iterations = number of times that both a mini-batch of data (i.e. 
%              mini-batch of "images") is passed through the network and 
%              the network parameters are updated
% MATLAB's default mini-batch size is 128 (i.e. there are 128 "images" in 
% each mini-batch). Also, since:
%     Iterations per epoch = (total number of training "images") / 
%                                  (number of "images" in each mini-batch)
% And, in this code:
%     Total number of training samples ("images") = trialCount * digitCount 
% Using the above, calculate the number of iterations per epoch, and check
% the total number of iterations:
%      Total Number of iterations = maxEpoch * Iterations per epoch

[myNeuralNetwork, info] = trainNetwork(xTrain,yTrain,layers,options); 
% output is the trained NN

%% 6. Test Neural Network

t = 1:length(info.TrainingAccuracy);
figure();
subplot(2,2,1);
plot(info.TrainingAccuracy,'LineWidth',2,'Color',"#0072BD");
hold on;
plot(t(~isnan(info.ValidationAccuracy)),info.ValidationAccuracy(~isnan(info.ValidationAccuracy)),'--k','LineWidth',2,'Marker','o');
title("Training Accuracy")
legend("Training Accuracy","Validation Accuracy");
xlabel("Iterations");
ylabel("Accuracy (%)");

subplot(2,2,3);
plot(info.TrainingLoss,'LineWidth',2,'Color',"#D95319");
hold on;
plot(t(~isnan(info.ValidationLoss)),info.ValidationLoss(~isnan(info.ValidationLoss)),'--k','LineWidth',2,'Marker','o');
title("Training Loss");
legend("Training Loss","Validation Loss");
xlabel("Iterations");
ylabel("RMSE");

predictions = classify(myNeuralNetwork, xTest)'; %classify testing data using NN
%disp("The Neural Network Predicted:"); disp(predictions); %display predictions
%disp("Correct Answers"); disp(yTest); % display correct answers
subplot(2,2,[2,4]); confusionchart(yTest,predictions); % plot a confusion matrix
title("Confusion Matrix");

%% 7. View Neural Network

figure(); plot(myNeuralNetwork); % visualize network connections

%% 8. Run-Time Predictions 
% (This is essentially a copy of the lab 1 code with determination 
% replaced with CNN code.)

% make sure NN exists
if(~exist('myNeuralNetwork'))
    error("You have not yet created your neural network! Be sure you run this section AFTER your neural network is created.");
end

% clear nb
clear nb;

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
nb.ledWrite(1);  % Turn on the LED to signify the start of recording data

% Gesture is performed during the segement below
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end

nb.ledWrite(0); % Turn the LED off to signify end of recording data

rtdata = [vals(1,:);vals(2,:);vals(3,:)];

% put accelerometer data into NN input form
xTestLive = zeros(3,150,1,1);
xTestLive(:,:,1,1) = rtdata;

% Prediction based on NN
prediction = classify(myNeuralNetwork,xTestLive);

% Plot with label
figure(); plot(rtdata', 'LineWidth', 1.5); %plot accelerometer traces
legend('X','Y','Z'); ylabel('Acceleration'); xlabel('Time') %label axes
set(gca,'FontSize',16); %make it easier to read the font
title("Classification:", string(prediction), 'FontSize',24); %title plot with the label

%% 9. Augment the data
% If you haven't already, make sure to go back to section 1 and uncomment
% the part called "Data Augmentation."  The run through all of the sections
% of this lab again with the larger amount of data (when you get to it, 
% tweak the learnRate, maxEpoch, and convolution, etc. as needed).  See if 
% you can further improve the run-time performance of your CNN!

%% 10. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all
close all