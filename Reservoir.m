clear all
close all
clc
%% Physical parameters of the three tanks
S1 = 1.3e-3;                            % Cross-sectional area of tank 1(m²)
S2 = S1;                                % Cross-sectional area of tank 2(m²)
S3 = S2;                                % Cross-sectional area of tank 3(m²)
d1 = 7.2e-3;                            % Diameter of the first tank's orifice(mm)
d2 = 5.4e-3;                            % Diameter of the second tank's orifice(mm)
ds2 = 3.5e-3;                           % Diameter of the output orifice of the second tank(mm)
ds3 = ds2;                              % Diameter of the output orifice of the third tank(mm)
alphd = 0.72;                           % Flow coefficient
qee = 2.78e-5;                          % Equilibrium flow (L/h)
qem = 1.4*2.78e-5;                      % Maximum realizable flow
g = 10;                                 % Gravity acceleration
phi1 = 4.2135e-05;                      % Orifice cross sectional area(mm²)
phi2 = 2.3487e-5;                       % Orifice cross sectional area(mm²)
phis2 = 9.9085e-6;                      % Orifice cross sectional area(mm²)
phis3 = 1.0252e-5;                      % Orifice cross sectional area(mm²)

a12 = alphd*phi1*sqrt(2*g);
as2 = alphd*phis2*sqrt(2*g);
a23 = alphd*phi2*sqrt(2*g);
as3 = alphd*phis3*sqrt(2*g);
%% Equilibrium points for each height x1,x2,x3(50cm>x10>x20>x30)
x10 = 0.2606;
x20 = 0.2148;
x30 = 0.1803;
xe = [x10;x20;x30];
                        %%%%%% Linear Control %%%%%%
%% State space representation of linearized model at the operating point x10,x20,x30,qee
A = [-0.2415 0.2415 0;0.2415 -0.4244 0.1566;0 0.1566 -0.1865]; % State matrix
B = [795.7747;0;0];                                            % Input matrix
C = [0 0 1];                                                   % Output matrix
D = zeros(1,1);                                                % Coupling matrix 
sys = ss(A,B,C,D);
%% From state space to transfer function
[b,a] = ss2tf(A,B,C,D);
Hol = tf(b,a);
%% Open loop stability of the linearized system
eigol = eig(A);                                                % Open loop eigenvalues of the system
%% Controllability and Observability
Com = ctrb(A,B);
rcom = rank(Com);
Obs = obsv(A,C);
robs = rank(Obs);
if rcom == length(A)
    disp('The system is controllable')
else
    disp('The system is not controllable')
end
if robs == length(A)
    disp('The system is observable')
else
    disp('The system is not observable')
end
%% Pole placement and feedback dynamic control law
P = 1/3*eigol(1)*ones(1,3);
K = acker(A,B,P);
%% From state space to transfer function after Feedback control
[bc,ac] = ss2tf(A-B*K,B,C,D);
Hcl = bc/ac;
%% Feedforward control
N = inv(bc(4)/ac(4));
%% Augmented state space model and feedback control with integral action
M = C;
Aa = [A zeros(3,1);-M 0];
Ba = [B;0];
Ca = [C 0];
Pa = [P P(1)];
Ka = acker(Aa,Ba,Pa);
%% Full Order Luenberger Observer
Po = 2*P;
L = acker(A',C',Po);
%% Reduced Order Luenberger Observer : zdot = (A11-L2*A21)z+((A11-L2*A21)L2+A12-L2A22)x3+(B1-L2B2)u
A11 = [A(1,1) A(1,2);A(2,1) A(2,2)];
A12 = [A(1,3);A(2,3)];
A21 = [A(3,1) A(3,2)];
A22 = A(3,3);
B1 = B(1:2);
B2 = B(2:end);
L2 = L(1:2);









