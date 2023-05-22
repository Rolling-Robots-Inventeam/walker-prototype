# walker-prototype
Arduino programs
Last updated 2/18/23

Hi!

Welcome to Alex's Commandrunning documentation!
I'll keep this short, by just explaining the conceit of the CW_Master and CW_Motor programs: taking and receiving 6-character commands.

Bear in mind:
  - Each byte gets more and more context-specific the further down you go. 
    This example covers the currently functional code, but cannot cover the wide range of potential applications in further development.
  - This code is meant to run several Vesc motors, which require some careful setup. Our guide, with pictures and example code, is available here:
    https://docs.google.com/document/d/1rYXp6uxJL8TSdy2wzScx4im5B_NEkK_osdKSdBAQtKA/edit
  - All these rules are arbitrary. They need only be internally consistent. 
  - This was not written with pure optimization nor pure readability in mind. I would avoid the temptation to lean too far towards either.
  - Have fun! :D

***---***

Byte by Byte Analysis, with Example:

            Example Command: "11+030"
            
        1            1            +        030
  1x Purpose + 1x Location + 1x Action + 3x Data


---Byte 0: Purpose
 This byte indicates the job of the whole rest of the command. 
 It's an integer counter starting at 1. It's read on a case-by-case basis in Motor, cases being:
  1 -> Send instructions (to motor to drive it, perhaps to other attached devices)
  2 -> Recieve data (from sensor, or perhaps motor)
  
 IN THIS EXAMPLE, Purpose = 1 -> This command directs a motor.
 
 
---Byte 1: Location 
 This byte indicates which device in the above group the command refers to.
 It's an integer counter, whose range depends on Purpose. For motors, it ranges 1-4 for the 4 drive motors.
 
 IN THIS EXAMPLE, Location = 1 -> This command directs Motor #1.
 
 
---Byte 2: Action 
 This byte describes the exact process that is going to happen to our particular device.
 This byte could be any character, and it depends entirely on the device in question.
 The current list of proposed motor actions is as follows:
  '+' -> Set a positive motor duty
  '-' -> Set a negative motor duty (Read by the Motor chip as part of the Data number)
  '/' -> Disengage motors (for free rolling as walker)
  '!' -> Engage or brake motors (whatever at this point)
 
 IN THIS EXAMPLE, Action = + -> This command will set the duty of Motor 1 to some positive value.
 
 
---Bytes 3, 4, and 5: Data
 These bytes are the block of information that's being passed from master to motor device.
 These bytes are, for now, all numbers.
 In the case of motor controls, they range from -100 to 100 representing possible motor duties (-1.00 to 1.00) x 100.
 In the case of motor controls, they are formatted to the right decimal before being plugged directly into a setDuty command.
 
 IN THIS EXAMPLE, Data = 030 -> This command will pase 030 into 0.30, and set the duty of Motor 1 to that.
 
***---***

 That is all for this documentation! Hope it helps :D
 
 -Alex
