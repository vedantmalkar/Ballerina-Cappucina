## Video 1:
---
##### Types of control
- Passive (passively changing the output favourably)
- Active (Energy used up to manipulate system behaviour)
	- Open-loop control (control action is determined by a predefined input, without using feedback to adjust for changes or disturbances in the output, lots of energy used)
	- Closed-loop feedback control (control action is continuously adjusted based on feedback from the output to maintain the desired performance)

##### Why feedback?
1. Uncertainty: any pre-planned input is not always correct and can be sub optimal, however when given a feedback by observing output it can be corrected
2. Instability: actual dynamics of system can be corrected  
3. Disturbances: any unexpected external disturbance can be predicted  
4. Efficiency 

An example of a system is:
	x˙(t)=Ax(t)
	
After control:
	x˙(t)=Ax(t) + Bu 

where u = -Kx

u is chosen by the control system to nullify errors as much as possible

## Video 2:
---
in a control system given by:

x˙=Ax

x is a vector and A is a nxn matrix which tells you how variables react with each other 

the basic solution of this equation is x(t) = e^(At) x(0)

to find the exponential value of a matrix just plug in the value of the matrix into the taylor series 

![[Screenshot from 2025-06-29 22-58-18.png]]


this can sometimes be very complex hence we use eigen values instead 

eigenvectors of a matrix are non-zero vectors that, when multiplied by the matrix, result in a new vector that is a scalar multiple of the original vector

![[Screenshot from 2025-06-29 23-03-20.png]]

we can say that:

## AT = TD
where A is system matrix, T is matrix of eigen vectors and D is diagonal matrix of eigen values 

T^-1 AT = A

![[Screenshot from 2025-06-29 23-11-49.png]]

this is a lot easier to solve than taylors
when we solve this we get:

![[Screenshot from 2025-06-29 23-15-34.png]]


we know that A = TDT^-1

A^2 = TDT^-1 TAT^-1 = TD^2 ^-1


![[Screenshot from 2025-06-29 23-24-05.png]]

putting this expression into the original equation:

![[Screenshot from 2025-06-29 23-28-30.png]]

