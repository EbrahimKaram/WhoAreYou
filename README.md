# Who are you? _An Artistic Piece_
This was the code used to drive the heart of our piece. The project was part of the robotics for Creative Pratice course at Carnegie mellon. 

The code was built on top of the StepperWinch Program developed by Garth Zeglin. 
That code can be found in the link below
https://courses.ideate.cmu.edu/16-376/s2020/ref/text/software/StepperWinch.html

In essence, this is a Zeotrope that starts and changes color based on the input of two different ultrasound sensors.

You can find the youtube Videos showing the progressive build of the piece in the link below. 
https://youtube.com/playlist?list=PLgK8ktcGDGs_JM1s6qz76z2ol__JUwTJW


Final Video trailer
https://youtu.be/fU3-fOXpoGg


# Daily Logs
Below are the daily notes taken into account as this project was developed. 


## Saturday November 13,2021
We want to modify the ultrasound sensor to use SPindle Direction and enable
now we would also like to have this not use a delay.

https://roboticsbackend.com/arduino-pulsein-with-interrupts/

We need to measure the high end of the echo pin

https://www.arduino.cc/en/Reference.PulseIn

So for the arduino uno only pin 2 and 3 can have an interrupt attached to them

that is why we are using the y step

we need have the echo be on something to detect 40 cm or less

1 microseconds would detect 20 cm


## Thursday November 11,2021
We got most of this to work with the RGB
The light isn't as strong

We can set the color to whatever we like by using the code
color R G B
where each value is from 0 to 255

Now we need to setup the Ultrasound Sensors
We have two ultra sound sensors
https://create.arduino.cc/projecthub/abdularbi17/ultrasonic-sensor-hc-sr04-with-arduino-tutorial-327ff6

Supply voltage needs to be 5V.
There is GND and Triger and echo
They didn't work with A4 and A5

it does work with digital pins.
We need to turn it on when value is less than 20 cm

Both Ultrasound sensors seem to work in this scenario

A reading will take 12 microseconds
We can do a reading every second and the system seems to work fine

Let's try to see if we can get the on and off system to work.

SDA And SCL can be for the first ultrasound
We can do a check every 5 seconds if someone is less than 10 cm away

This will cause the zeotrope to turn on and off respectively.

### Issue
The pulse in function is blocking so we need to add timer interrupt on the system to read the value
First need to check if this works.
For the turn on and turn off.
We can shorten the delay and that wouldn't cause to many issues for the delay in and out.

You need to stay looking at it for 5 seconds. if the user steps away it will stay on for 50 seconds

### Turning the zeotrope on and off

## Sunday, November 7,2021
This is an absolute move.
a x 50			move the X axis to +50
this would move it to a +50 location

d x 50 would just move x 50 plus

There seems to be some oscillation. We might need to improve the PIDs here.

set all channels to 0.1 Hz natural frequency and underdamping
g xyza 0.1 0.5

This seems to work
g xyza 0.1 1

is the best result at 200 micro steps.
g xyza 0.5 1

We have 12 figures.
We want the strobe to flash at certain marks
We need to mod 200 what ever position we need to have it to strobe on the 16.666 mark

we have 360 degrees per revolution. in 200 counts that would be 1.8 degree per count.

Each of the slots is 10 degrees. how many counts would that be.?
10/1.8= 5.555 steps
It's more like 5 degrees per slot.
5/1.8= 3
So we need 3 micro steps or smaller to flash
5*12 would mean 60 degrees are covered by slots. 60/360 that would mean something like 16 percent of the circle.
which is like 3 steps of 16.66 which is around 18 percent.



## Saturday, October 30,2021
We might need to use smaller micro steps
  https://courses.ideate.cmu.edu/16-375/f2021/text/hardware/cnc-shield.html#id17

Micro stepping suggests that we need to be to remove the things below the driver
We are now setting this to eighth step.

2X, 4X, 8X, or 16X the full-step resolution

The stepper motor runs at
KL17H248-15-4A NEMA17 stepper motor with the CNC Shield. This is a bipolar 200 counts/rev

So we had 3200 micro steps at 16x


We are using driver x
v x

We removed M2 which caused an eighth step.
200 * 8
1600 per revolution

No pins attached makes this a full step revolution
So 200 steps per revolution

We have 6 figures on the zeotrope.
if we want to do 12 frames a second.
We need to do 2 revolutions per second.
The time period for 12 Hz is



## Thursday October 28, 2021
The strobe light was set to a 50/50 duty cycle.

We need to shorten that duty cycle

https://electronics.stackexchange.com/questions/159098/experimental-strobelight-duty-cycle#:~:text=Typical%20strobe%20lights%2C%20such%20as,the%20duty%20cycle%20is%20only%20.

So we need to set the interval between these two to get a better desired effect.

We might need to shorten our test phase and see how we can get there faster.
