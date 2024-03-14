//
//  NumericalMethods.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath on 12/3/24.
//

#include "NumericalMethods.hpp"


double factorial(int x)
{
   int factorial=1;
   for (int i = 1;i <= x;i++)
       factorial*=i;

   return factorial;
}


double taylor(double x, int n)
{
    double output,sum;
    double value = 0.0;
    
    for (int i=1; i<=n; i++)
    {
        sum=((pow(x,i))/factorial(i));
        value+=sum;
    }
    output=value+1;
    return output;
}

//  Author:
//
//    John Burkardt, Niran (Modified)
//
//  Parameters:
//
//    Input, double T0, the current time.
//
//    Input, double U0, the solution estimate at the current time.
//
//    Input, double DT, the time step.
//
//    Input, double F ( double T, double U ), a function which evaluates
//    the derivative, or right hand side of the problem.
//
//    Output, double RK4, the fourth-order Runge-Kutta solution estimate
//    at time T0+DT.
//
double forwardEuler(double x0,double dx,double dt){
    return x0 + dt*dx;
}
double RK2(double t0, double x0,double f(double t,double x),double dt){
    double f0 =f(t0,x0);
    double t1 =t0+dt/2.0;
    double x1 = x0 + dt * f0/2.0;
    double f1 = f(t1,x1);
    double output = forwardEuler(x0, f1, dt);
    return output;
    
}
double RK4(double t0, double x0,double f(double t,double x),double dt){

    
    double f0 =f(t0,x0);
    double t1 =t0+dt/2.0;
    double x1 = x0 + dt * f0 / 2.0;
    double f1=f(t1,x1);
    double t2 = t0+dt/2.0;
    double x2 = x0+ dt * f1/2.0;
    double f2 = f(t2,x2);

    double t3 = t0 + dt;
    double x3 = x0 + dt * f2;
    double f3 = f(t3,x3);
    double output = forwardEuler(x0, ( f0 + 2.0 * f1 + 2.0 * f2 + f3 ) / 6.0, dt);
    return output;
    
};



