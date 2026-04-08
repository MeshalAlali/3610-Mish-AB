%%%%%%%%%%%%%
% ECE 3610
% LAB 5 -- Inertial Measurement Lab
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% Tilt detection using an IMU (inertial measurement unit) is critical for a 
% huge number of products - the Wii-mote, quadrotors, and commercial 
% aircraft, to name a few. The three-axis MEMS accelerometer in your 
% Arduino is a critical component of the IMU. In this lab, we will find out 
% how the measurements from the IMU can be used to calculate and visualize 
% the tilt angle of the Arduino in real time.
% 
% Deliverables:
% - Fill in the correct orientation of the accelerometer axes at the end 
% of section 2.
% - Demonstrate the 3D tilt matching game. 
% - Fill in your answers to section 5 (impact of changing the pause).
% - Be able to discuss your results for section 6 (impact of changing 
% numReads, angle_threshold, pause).
%
% Extensions:
% - Change the color of the 3D box being displayed every time you press a
% pushbutton.
% - Change the color of the 3D box being displayed every time you tap 
% the accelerometer.

%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE. Note that the
%  clc and clear all will clear your command window and all saved
%  variables!

clc
clear all

% for PC:
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% 2. Testing the onboard IMU in the presence of gravity
% For this part, position the Arduino flat to the horizon, the Arduino's 
% pins oriented downwards towards the floor, and the Arduino's microUSB 
% port oriented towards you.

% Your Arduino board has a built-in accelerometer that can be used to
% observe the effect of gravity as well as its own movement (i.e. the 
% acceleration of the Arduino in the x, y, and z directions).  If an 
% accelerometer is at rest on a surface, it will register acceleration 
% due to gravity (~9.8 m/s^2).

% Let's first examine the effect of gravity on the output of the IMU. Try 
% writing (single or averaged) accelerometer values to the screen and/or 
% creating a live plot.  What are the x, y, and z values when the board 
% is flat vs. other orientations?   
%
% HINT; 'accel' in nanobot_demo.m stands for accelerometer 

% FILL IN code here to create a live plot 


nb.livePlot('accel');


% FILL IN code to take a (single or averaged) reading and writing the 
% value to the screen.

% tic
% vals = [];
% while (toc < 1)
%     vals(length(vals)+1) = nb.analogRead('accel');
% end
% meanval2 = mean(vals);
% fprintf('mean val = %.0f\n', meanval)




% What is the accelerometer's approximate x value when the chip is flat to 
% the horizon? (record the closest integer value, i.e. -2, -1, 0, 1, 2, 
% etc.)
x_flat = 0;
% What is the accelerometer's approximate y value when the chip is flat to 
% the horizon? (record the closest integer value, i.e. -2, -1, 0, 1, 2, 
% etc.)
y_flat = 0;
% What is the accelerometer's approximate z value when the chip is flat to 
% the horizon? (record the closest integer value, i.e. -2, -1, 0, 1, 2, 
% etc.)
z_flat = 1;

% What are the accelerometer's values when the chip is on its left side?
x_left_side = 0;
y_left_side = 1;
z_left_side = 0;

% What are the accelerometer's values when the chip is on its right side?
x_right_side = 0;
y_right_side = -1;
z_right_side = 0;

% What are the accelerometer's values when the microUSB is oriented 
% straight upwards?
x_microUSB_up = 1;
y_microUSB_up = 0;
z_microUSB_up = 0;

% What are the accelerometer's values when the microUSB is oriented 
% straight downwards?
x_microUSB_dwn = -1;
y_microUSB_dwn = 0;
z_microUSB_dwn = 0;

% Position the Arduino flat to the horizon again and with its microUSB port 
% oriented towards you.
% Using your results obtained earlier in this section, the x, y, and z-axes 
% of the accelerometer on board the Arduino are oriented:  (1) vertically 
% up/down; (2) left/right; (3) front/back (in line with the microUSB port).  
% Fill in the blanks below.  
x_axis_orientation = "up/down"; %keep the quotes to keep this a string
y_axis_orientation = "left/right"; %keep the quotes to keep this a string
z_axis_orientation = "front/back"; %keep the quotes to keep this a string

%% 3. Visualizing IMU tilt
% Next, let's move the Arduino and use gravity and the accelerometer values 
% to visualize the movement of the Arduino (roll and pitch) in a plot 
% created by MATLAB.

% First, this section of code will set up a 3D box that will represent 
% the Arduino in the plot. This section will not create a plot yet.  Just 
% run it to set the features of the 3D box (i.e. the dimensions, colors,
% and corner locations of each side of the box.

% Initialize the cube 
xc=0; yc=0; zc=0;    % cube center coordinates
L=2;                 % cube size (length of an edge)
alpha=0.8;           % transparency (max=1=opaque)

% define the X, Y, and Z coordinates of each corner of the box; use to 
% plot each face of the box
X = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Y = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C= [0.1 0.5 0.9 0.9 0.1 0.5];   % each face of the box will have a 
                                % different color
                          
X = L/1.5.*(X-0.5) + xc; %define the board length in the X-direction
Y = L*(Y-0.5) + yc;  %this is the long dimension of the board
Z = L/3*(Z-0.5) + zc; %this is the smallest dimension of the board
V=[reshape(X,1,24); reshape(Y,1,24); reshape(Z,1,24)]; %reshape takes all 
% of the elements of the 3D X matrix (and Y and Z matrices) and puts them 
% all into one column of dimension [1,24].  So, the first dimension of V 
% holds all of the X elements, the second dimension of V holds all of the 
% Y elements, and the third dimension of V holds all of the Z elements.

%% 4. Track IMU pose (track and plot the roll and pitch of the board)
% For this part, start with the Arduino flat to the horizon and with its 
% microUSB port oriented towards you.

% Today, we will only calculate the pitch and roll of the Arduino  
% because the Arduino Nano 33 IoT does not include a magnetometer.  
% Without a magnetometer, determining the yaw is more involved because 
% it requires an integration of the angular velocity (obtained from the 
% IMU's gyroscope) over time.  This can be left to a future exercise.  

% In this section, once you figure out all of the '?' parts, the 3D block 
% you defined in the last section will be plotted and should replicate the 
% roll and pitch movements of your board. In addition, a game has 
% been set up in which you will be given an orientation and you have to 
% try to match it (this type of programming is used inside of drones!).  
% When playing the game, make sure to uncomment the two game-related parts 
% of the code.

% Note:  If you get an error when trying to run "angle2dcm," then you
% need to install the Aerospace Toolbox (available for free through the 
% University Matlab license).

% Offset Calibration:
fprintf('Check the following and then press enter once:\n');
fprintf('1. The Arduino is lying flat (IMU chip parallel to horizon); and\n');
calib1 = input('2. The Arduino''s microUSB port is oriented towards you.');
% Now calibrate
% Here is an example of taking 10 accelerometer readings, then averaging
% each axis:
numreads = 10;
vals = zeros(3,numreads);
for i = 1:numreads
    val = nb.accelRead();
    vals(1,i) = val.x;
    vals(2,i) = val.y;
    vals(3,i) = val.z;
end
% find the mean of each of the x,y,z accelerometer values
meanOffx = mean(vals(1,:)); 
meanOffy = mean(vals(2,:));
meanOffz = mean(vals(3,:));
% As we tilt the board, we want to know the change in position relative 
% to it being flat (use your results from Part 2 of this lab to fill in the 
% blanks below):
xOff = meanOffx - x_flat; % Fill in the variable name for the x value when 
                         % the chip is flat
yOff = meanOffy - y_flat; % Fill in the variable name for the y value when 
                         % the chip is flat
zOff = meanOffz - z_flat; % Fill in the variable name for the z value when 
                       % the chip is flat

% IF PLAYING GAME, UNCOMMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pitchT = randi([-60, 60]);
pitchT = pitchT * pi/180;
rollT = randi([-60, 60]);
rollT = rollT * pi/180;

dcm_targ = angle2dcm(0, pitchT, rollT);
V_targ = dcm_targ*V;
X_targ=reshape(V_targ(1,:),4,6);
Y_targ=reshape(V_targ(2,:),4,6);
Z_targ=reshape(V_targ(3,:),4,6);

figure(1)

fill3(X_targ,Y_targ,Z_targ,C,'FaceAlpha',alpha);
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,0);
box on;
drawnow
movegui(1,'west')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic; %to count the seconds

while(toc<20) % stop after this many seconds

    numreads = 10; %take three readings and average
    IMUvals = zeros(3,numreads);
    for i = 1:numreads
        val = nb.accelRead();
        IMUvals(1,i) = val.x;
        IMUvals(2,i) = val.y;
        IMUvals(3,i) = val.z;
    end
    % find the mean of each of the x,y,z accelerometer values
    meanx = mean(IMUvals(1,:));
    meany = mean(IMUvals(2,:));
    meanz = mean(IMUvals(3,:));

    % Find the x-,y-, and z- acceleration values due to gravity and 
    % relative to the starting position
    ax = meanx + xOff; 
    ay = meany + yOff; 
    az = meanz + zOff; 

    % Convert the x-, y- and z-axis accelerometer values to angles of 
    % inclination (here, roll and pitch).  Find this in the readings!
    theta = atan(ax/sqrt(ay.^2 +az.^2));   % roll angle (radians); equation 11
    psi = atan(ay/sqrt(ax.^2 +az.^2)); % pitch angle (radians); equation 13

    % Create a direction of cosine matrix from the angles of inclination.
    % The first term (yaw) is zero since it can't be found using the
    % accelerometer alone (a magnetometer would be helpful here).
    dcm_acc = angle2dcm(0,psi,theta); % (yaw,pitch,roll) 

    % IF PLAYING GAME, UNCOMMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%
    pitchCheck = psi * 180/pi;
    rollCheck = theta * 180/pi;
    angle_threshold = 10;
    if((pitchCheck > (rad2deg(pitchT) - angle_threshold)) & ...
            (pitchCheck < (rad2deg(pitchT) + angle_threshold)) & ...
            (rollCheck > (rad2deg(rollT) - angle_threshold)) & ...
            (rollCheck < (rad2deg(rollT) + angle_threshold)))
        fprintf('MATCHING DESIRED ORIENTATION!\n');
        figure(1); title(['\color{red}MATCHING!']);
        figure(2); title(['\color{red}MATCHING!']);
        break
    else
        fprintf('Not matching desired orientation...\n');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % multiply the direction of cosine matrix by the box to rotate it
    V_rot=dcm_acc*V;
    % extract the X, Y, and Z vectors for plotting
    X_rot=reshape(V_rot(1,:),4,6);
    Y_rot=reshape(V_rot(2,:),4,6);
    Z_rot=reshape(V_rot(3,:),4,6);


    figure(2)

    fill3(X_rot,Y_rot,Z_rot,C,'FaceAlpha',alpha);
    xlim([-2 2]); % make the limits in all directions the same so the box
    ylim([-2 2]); % is plotted on the same scale in all directions
    zlim([-2 2]);
    xlabel('X'); 
    ylabel('Y');
    zlabel('Z');
    view([0,0]); % start by looking straight at the USB port
    box on;
    drawnow
   
    pause(0.5);
  
end

%% 5. Impact of the pause
% What is the impact of changing the length of the pause at the end of 
% Section 4?  Why does this happen?

% Try changing the pause to 0.01. In the blanks below, briefly describe
% what happens and why.
  shorter_pause = ("Smooth");  %keep the quotes to keep this a string
% Then try changing the pause to 0.2
  longer_pause = ("Laggy");  %keep the quotes to keep this a string
% Then try changing the pause to 0.5
  even_longer_pause = ("Unplayable");  %keep the quotes to keep this a string

%% 6. Impact of numReads and angle_threshold
% The default numReads was 3, the default pause was 0.1 seconds, and the 
% default angle_threshold was 10 degrees.  Try increasing / decreasing 
% these values to determine the impact (e.g. Reduce the pause and increase 
% numReads. Can you decrease angle_threshold?)

%% 7. EXTENSION (optional)
% - Change the color of the 3D box being displayed every time you press a
% pushbutton.
% - Change the color of the 3D box being displayed every time you tap 
% the accelerometer.

%% 7. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all