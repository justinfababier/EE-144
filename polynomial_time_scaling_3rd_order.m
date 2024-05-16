clc;
clear;

syms x_0 x_T x_dot_0 x_dot_T T a_3 a_2 a_1 a_0;

x = [x_0; 
     x_T; 
     x_dot_0; 
     x_dot_T];

T = [0      0   0 1;
     T^3    T^2 T 1;
     0      0   1 0;
     3*T^2  2*T 1 0];

a = transpose(T).*x;

%{
 
a_3 = [0, T^3, 0, 3*T^2][x_0]
a_2 = [0, T^2, 0,   2*T][x_T]
a_1 = [0,   T, 1,     1][x_dot_0]
a_0 = [1,   1, 0,     0][x_dot_T]

%}