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

### AT = TD
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



## Video 3:
---
#### Stability:
we know that 
### $$
x(t) = T e^{Dt} T^{-1} x(0)
$$
The exponential of a diagonal matrix is easy to write and can be written as: 

$$
e^{Dt} = \begin{pmatrix}
e^{\lambda_1 t} & 0 & \cdots & 0 \\
0 & e^{\lambda_2 t} & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
0 & 0 & \cdots & e^{\lambda_n t}
\end{pmatrix}
$$

if system is unstable i.e when a value in RHS tends to infinity then so will LHS

### $$
 \lambda = a + i b
$$
this is known as Euler's formula
### $$
e^{\lambda t} = e^{(a + ib)t} = e^{at} \left( \cos(bt) + i \sin(bt) \right)
$$

when we have a complex eigen value this is how it breaks up

The value $e^{at}$ tells us whether the solution is **growing** or **decaying** in time:

- If $a > 0$, the solution is **growing** over time because $e^{at}$ increases as time progresses.
- If $a < 0$, the solution is **decaying** over time because $e^{at}$ decreases as time progresses.


![[Pasted image 20250706195120.png]]

### the system will be stable if and only if all real parts of all eigen values are negative (in continuous time)

in control systems we may have some positive values of eigen vectors what we do is that we add some +Bu which drives that system to stability 

![[Pasted image 20250706195555.png]]

we measure the system at time intervals of $\Delta t$

![[Pasted image 20250706200056.png]]


![[Pasted image 20250706201402.png]]


### if the eigen values of $\tilde{A}$ are inside unite circle then the system is stable (in discrete time)

## Video 4:
---
Linearizing Around a Fixed Point using pendulum as example, steps:
1. Find fixed points, i.e. $\bar{x}$, such that $f(\bar{x}) = 0$
2. linearize about $\bar{x}$ (find gacobian at $\bar{x}$)

Gacobian:

![[Pasted image 20250706225820.png]]


![[Pasted image 20250706231503.png]]

USING INVERTED PENDULUM AS EXAMPLE:

  $\displaystyle \ddot{\theta} = -\frac{g}{l} \sin(\theta) - \delta \dot{\theta}$

where $\delta \dot{\theta}$ is friction 

this is a non linear equation

fixed points at 0,pie,2pie ....
![[Pasted image 20250706234841.png]]

we represented the basic physics of the system in the given format

1. fixed point $\bar{x}$ = origin i.e d/dt(x1 x2) = 0


2. compute jacobian and place $\bar{x}$ values into matrix 


![[Pasted image 20250706235708.png]]

finding egien values using matlab

![[Pasted image 20250707000020.png]]

negative eigen values shows pendulum is dampening i.e stable


## Video 5:
---
New A matrix with sensor based feedback control:

![[Pasted image 20250707001807.png]]

in a control system normally A and B are fixed and you get to control 'u'

example of bad B for controllability:

![[Pasted image 20250707002608.png]]


example of good B for controllability:
![[Pasted image 20250707002946.png]]

another example of good controllability:

![[Pasted image 20250707003604.png]]

here x1 can be controlled by values of u2

![[Pasted image 20250707004304.png]]


![[Pasted image 20250707004720.png]]

the command ctrb(A,B) in matlab gives you the value of "curly" C and we can determine controllability by checking rank

## Video 6:
---
#### Equivalences:
1. system is controllable  
2. Arbitrary eigen values (pole) placements   
	$u = -kx$ -> $\dot{x} = (A-Bk)x$
3. Reachability 

if one of these are true then all others are also true 

![[Pasted image 20250707153854.png]]




## Video 7:
---
when we hit the system with an impulse in u and it rings through the system and there are some directions in state system which are not touched by the input B then we cannot reach them with control ![[Pasted image 20250707160233.png]]


