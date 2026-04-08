% Vet and Merge Trainings
% This code allows you to select all the downloaded .mat files from the
% class and run a few different operations on it. After selecting all the
% files, a window will pop up (perhaps very small in the upper left corner 
% of your screen) asking if you'd like to review the files. If
% you select yes, this will draw small plots of each individual training
% for every file. If a file has 100 trainings in it, then the figure will
% show 100 small plots. The main purpose of this is to vet the trainings
% and see if there are any obvious bad trainings that you'd like to
% exclude. If there's something you don't like, the filename is listed at
% the top of the figure to facilitate finding and deleting the file.

% After reviewing the files, the next option will be to perform a size
% check on each of the trainings. If there are inproperly sized trainings
% (too many or too few samples) it will be flagged and reported.

% After checking the training sizes, there will be another window pop up
% asking if you'd like the merge the files. If you reviewed files and and
% found that you don't want all of them included, select 'no' and come back
% later after you've re-run this script selecting the proper files. If
% you're confident in the files, you can skip the 'review' step, as it
% takes a minute to load all the plots.

% Finally, the code will prompt the user for a filename to save the data.


clear;clc;close all;

[files, path] = uigetfile("MultiSelect",'on','.mat');

Training_0 = [];
Training_1 = [];
Training_2 = [];
Training_3 = [];
Training_4 = [];
Training_5 = [];
Training_6 = [];
Training_7 = [];
Training_8 = [];
Training_9 = [];

option = menu("Would You like to Review these files?", {"Yes", "No"});
if option == 1
    for i = 1:length(files)
        figure();
        load(fullfile(path,files{i}));
        [h,w] = size(data);

        for h_inx = 1:h
            for w_inx = 2:w
                nexttile;
                plot(data{h_inx,w_inx}')
                title(string(data(h_inx,1)));
            end
        end
        sgtitle(replace(files{i}, "_", "\_"));
        clc;
        fprintf("%d of %d\n", i, length(files));
    end
end



option = menu("Proceed with Data Size Check?", {"Yes", "No"});
if option == 1
    clc;close all;
    for i = 1:length(files)
        load(fullfile(path,files{i}));
        [h,w] = size(data);

        for h_inx = 1:h
            for w_inx = 2:w
                if length(data{h_inx,w_inx}) ~= 150
                    fprintf("Bad File (length %d): %s\n",length(data{h_inx,w_inx}),string(files{i}));
                end
            end
        end
    end
end



option = menu("Proceed with Automatic Merge?", {"Yes", "No"});
if option == 1

    for i = 1:length(files)
        load(fullfile(path, files{i}));
        [h,~] = size(data);
        for h_inx = 1:h
            gesture = data{h_inx,1};
            cmd = sprintf("Training_%d = [Training_%d, data(h_inx,2:end)];", gesture, gesture);
            eval(cmd);
            clear cmd;
        end
    end
    len_array = [];
    for i = 0:9
        if ~isempty(eval(sprintf("Training_%d", i)))
            len_array = [len_array, length(eval(sprintf("Training_%d", i)))];
        end
    end
    minlength = min(len_array);
    k=1;
    data = cell(length(len_array), minlength+1);
    for i = 0:9
        if ~isempty(eval(sprintf("Training_%d", i)))
            data{k,1} = i;
            data(k,2:minlength+1) = eval(sprintf("Training_%d(1:minlength)", i));
            k = k + 1;
        end
    end
end

filename = inputdlg('Enter filename to save data:', 'Save File'); 
if ~isempty(filename) % Check if user entered a filename
    save(filename{1}, 'data');
end
