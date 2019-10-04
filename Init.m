%Initialization file for some Parameters: 
%Sample time for the MPC: 
Ts=1;
%Simulation/Implementation time:
N=40;
%Horizon time for MPC: 
H=5;
%Generate reference trajectory: 
ref=[-0.8+0.5*cos(0.2*(0:Ts:(N+H))+pi/4);...
-0.4+0.5*sin(0.2*(0:Ts:(N+H))+pi/4)];
