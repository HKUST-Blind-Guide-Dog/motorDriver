# RMD X8 motor driver with RS485 serial
 
 ## **QUICK START**
 After clone the repo into your computer, directly cd to the downloaded repo folder. And running the following command in the terminal.
 ```python
 mkdir build
 cd build
 cmake ..
 make
 sudo ./motorTest
 ```
 Then it will require you to input your password. If the password is correct, the program will be successfully executed. 

 ## **Notice**
 You might find the command buffer message in the termianl, it is just for the debugging. You could comment it out from the code. Also you might find some message like "serial read failed", it is because the control frequency is too high for the current hardware. This is also the problem I encountered and it need to be resolved as soon as possible.