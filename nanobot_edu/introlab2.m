%%%%%%%%%%%%%
% ECE 3610
% LAB 2 -- Toolchain Intro 2 -- Experimenting with Other Peripherals
%%%%%%%%%%%%%

%%%%%%%%%%%%%
% In this lab, we will be learning how to connect external components to
% our microcontroller, such as motors, a potentiometer, and a push button. 
% We will also continue to explore the nanobot class and its functions as 
% detailed in nanobot.m.
%
% Deliverables:
% Develop a MATLAB script (this one) which successfully:
% - Moves a DC motor
% - Moves a servo motor
% - Shows live plotting of a variable input voltage (analog data)
% - Toggles an LED using a pushbutton (digital data)
%
% Possible Extensions:
% - Make the LED blink when the input voltage increases above its midpoint, 
%   and go steady when the voltage returns below the midpoint.
% - Move the servo in 10-degree increments every time the pushbutton is
%   pressed. Return to home position when the voltage reading increases 
%   above its midpoint.
%%%%%%%%%%%%%%

%% 1. CONNECT TO YOUR NANOBOT
%  Remember to replace the first input argument with text corresponding to
%  the correct serial port you found through the Arduino IDE or 
%  port_detector.m. Note that the clc and clear all will clear your command 
%  window and all saved variables!

clc
clear all

% for PC:
nb = nanobot('/dev/cu.usbmodem101', 115200, 'serial');
% for Mac:
%nb = nanobot('/dev/cu.usbmodem14301', 115200, 'serial');

%% AN IMPORTANT NOTE:  
% The communication between MATLAB and the nanobot will work much of the 
% time.  However, every now and then (seemingly randomly) a JSON packet 
% will get dropped and/or communication will be lost.  If you find this 
% has happened (you will likely see an error show up in MATLAB), try the 
% following: 
% - Rerun Section 1 above to re-establish a connection and then return to 
% what you were working on.  
% - You can try power cycling the Arduino (unplug it so it doesn't 
% receive any power and plug it back in again).
% - Push the reset button on the top of the Arduino.
% - Rerun the port_detector.m code (while double checking the port 
% number) and then rerun Section 1 above to reconnect to it.
% - Try to connect to your Arduino while it is not plugged into the motor
% carrier.


%% 2.  Plug the Arduino into the motor carrier
% If you plug the Arduino the wrong way into the motor carrier, it will 
% immediately burn out the Arduino when you turn the battery on!

% Check the image on the Canvas help page, and plug the Arduino correctly 
% into the motor carrier.


%% 3. Connect and run the DC motor
%  First, plug a LiPo battery into the motor carrier so that we can use it 
%  to drive the DC motor.  
%
%  There is an ON/OFF switch on your carrier board that enables the LiPo 
%  battery to drive the motors. Set the switch to the ON position.
%
%  Locate where the Motor 1 connections are on your motor carrier 
%  board (they should be marked as M1+ and M1-).
%
%  Take your DC motor and connect one of the leads to M1+ and the other to 
%  M1-. To do this, you will need to use a screwdriver to first open both 
%  ports a bit.  Insert the leads into the M1+ and M1- ports, and then use 
%  the screwdriver to close the ports again (there should be a good 
%  metal-to-metal connection between each wire leads and the metal inside 
%  of the ports). The polarity does not matter right now (we will learn why 
%  later).
%
%  Is is VERY IMPORTANT that we use the motor carrier and battery to drive
%  the motors. The Arduino can only output a very small amount of current
%  through its pins, so we want the motor carrier to do the heavy lifting
%  while the Arduino supplies the control signal. Using the Arduino itself
%  to power the motor could potentially damage your board and render it
%  nonfunctional.
%
%  Next, we want to drive the motor. Find the appropriate code in
%  nanobot_de2mo.m to make our DC motor move and copy it here:



nb.setMotor(1,25) % (motor number, duty cycle)
pause(1)
nb.setMotor(1,0)

%% 4. Connect and run the servo motor
%  Similarly to the DC motor hookup, locate the servo 1 output pins. Note,
%  if you check the underside of the carrier board, you will see that 
%  one of the the servo 1 output pins has a white stripe over it. This pin
%  corresponds to the brown wire connection for the servo motor.
% 
%  Connect the servo motor to Servo1, making sure to match the
%  brown wire to the white stripe pin.
%
%  Now find the appropriate code in nanobot_demo.m and copy it here to make
%  the servo motor move. Make sure to keep the LiPo battery plugged in and 
%  turned ON. 

angle = 0;
while (angle <= 180)
    nb.setServo(1,angle)
    angle = angle+20;
    pause(0.25)
end
clear angle



%% 5. Create a live plot of analog input voltages
% Each analog input pin of the Arduino detects a voltage and converts that 
% voltage to a digital value using an analog to digital converter (ADC). 
% (You'll see more info about this in the vidoes for the next lab). 

% First, connect a wire to the A1 pin of the Arduino. (To do this, use 
% the Arduino Nano 33 IoT pinout diagram provided in Canvas.)  Do not 
% connect the other end of the wire to anything yet. 

% Create a live plot of the analog values detected at the A1 pin on the 
% Arduino (find the appropriate code in nanobot_demo.m and copy it here).
% Make sure to specify the correct pin (A1), and make sure the code sets 
% the A1 pin to be an analog input.


%%This starts a dynamically updating plot of the accelerometer values. Click
%%the "stop" button and wait for the figure to close to exit.
%%nb.livePlot('accel');

%%You can also plot an analog read value. Note that I have to initialize the pin as an analog input ("ainput")
%%first:
nb.pinMode('A1','ainput');
nb.livePlot('analog','A1');


% Run this section.  What values do you observe at the A1 pin of the 
% Arduino?  Do the values change in time?  Right now, the A1 pin on the
% Arduino is not connected to anything, so it is a floating pin with 
% an unknown voltage.  You may observe noisy values, and the values may 
% even change in time if you move the wire connected to the A1 pin.

% Next, use the wire to connect the A1 pin on the Arduino to the 3.3V pin 
% on the Arduino (the Arduino Nano 33 IoT operates at 3.3V).  Now what 
% values do you see in the live plot?  The  Arduino uses 10-bit 
% analog-to-digital converter (ADC), which means when it can only display 
% 1024 different values.

A1_value_for_3pt3V = 1000;%1000

% Lastly, disconnect the wire from the 3.3V pin and instead use it to 
% connect the A1 pin on the Arduino to the GND pin on the Arduino.  
% What values do you now observe in the live plot?

A1_value_for_GND = 0;%0


%% 6. Live plot of analog input values adjusted by a potentiometer (pot)
% For this part, you will need your breadboard and the blue pot.
% Insert the pot into the breadboard so that the three pins are
% not electrically connected (i.e. plug the three leads of the pot into
% three different rows of the breadboard).
%
% Use three wires to connect: 
% (1) one of the outside leads of the pot (lead #1 or #3 in the image 
%     shown in Canvas) to the Arduino's GND (it's good practice to connect 
%     GND in a circuit first);  
% (2) the other outside lead of the pot to the Arduino's +3.3V; and 
% (3) the middle pin of the pot to the Arduino's A1 pin.

% Copy some code here to create another live plot of the A1 pin values.  
% How do the values in the plot change as you turn the knob on the pot?

nb.pinMode('A1','ainput');
nb.livePlot('analog','A1');



%% 7. Read digital values with a pushbutton
%  Grab the pushbutton and place it into the breadboard so that its pins
%  enter different rows (i.e. don't create a short circuit between the two
%  pins). Then use two wires to connect: (1) one pin of the pushbutton to 
%  the Arduino's GND; and (2) the other pin of the pushbutton to Digital 
%  Pin 10 (i.e. D10). 
%
%  Grab code from the DIGITAL READ section of nanobot_demo.m and modify it
%  as necessary to select the correct pin for digital reading.
%
%  Next, add some code so that the onboard LED turns on while the button is 
%  pressed and turns off again whenever the button is not being pressed.
%
%  HINT: If you wrap an if statement in a while(true) loop, the condition
%  will be checked continuously. You can also use tic and toc to have the
%  loop run continuously for a finite period of time.

nb.pinMode('D10','dinput');

while(true)
    if nb.digitalRead('D10') == 0 
            nb.ledWrite(1)
    else
            nb.ledWrite(0)

    end
end

%% 8. EXTENSION (optional)
%  Try these following tasks if you're up to the challenge:
% - Make the LED blink when the input voltage increases above its midpoint, 
%   and go steady when the voltage returns below the midpoint.
% - Move the servo in 10-degree increments every time the pushbutton is
%   pressed. Return to home position when the voltage reading increases 
%   above its midpoint.

%% X. DISCONNECT
%  Clears the workspace and command window, then
%  disconnects from the nanobot, freeing up the serial port.

clc
delete(nb);
clear('nb');
clear all