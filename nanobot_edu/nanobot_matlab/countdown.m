% This function will be used in intel labs to have a countdown to start
% making the gestures.
function countdown(message, seconds)
for i = flip(1:seconds)
    fprintf("%s\t%d\n", message, i);
    pause(1);
end
