# Video 1
## Introduction
- Understood the different types of control systems, like active and passive.
- Active is further divided into open looped and closed looped systems.

Space State differential equations are represented as:
x'=Ax, where x is a vector denoting all quantities of interest.

The closed loops system is also known as feedback loop, as the output is analyzed, changes are made and the input is further updated. 
#### It has 4 main advantages:
- Uncertainty
- Instability
- Disturbances
- Efficiency

The feedback system has one major advantage, it changes the inherent stability of the system, showing it as 
x'=Ax+Bu, where u is like the control knob, and B is how the changes we make will change x and its derivatives. 

Basically, a brief overview to control systems, why the feedback loop system is so important and useful. 

# Video 2
                        x'=Ax
This is how linear differential systems are denoted. Integrating it, we get
                         x(t)=$e^{At}$  x(0)
                          By Taylor series expansion,
                          $e^{At}$= I+$(At)^2$/2!+$(At)^3$/3!+...

By the concept of eigen vectors and eigen values, we know that for any matrix A,
					     Aξ=Λξ
			where ξ denotes the Eigen vectors, and Λ the eigen values.  $T^{-1}$    
  T=[ξ1, ξ2,....ξN] is a matrix of all eigen vectors, and D is a diagonal matrix, containing the eigen values Λ1,Λ2,...Λn as the diagonal elements.
  Thus, we get the relation:
   $$
T^{-1} A T = D
$$

x=Tz, where z is a state vector in the eigen coordinate system, easier to calculate.
				  
				  x'=Tz'=Ax
				  Tz'=ATz
				  z'=T-1ATz
				  z'=Dz
Putting A=TD$T^{-1}$ in expansion of $e^{At}$ = I+At+$(At)^2$ /2!+...
We get after simplifying
				 $e^{At}$=T$e^{Dt}$$T^{-1}$
which is much easier to calculate.


