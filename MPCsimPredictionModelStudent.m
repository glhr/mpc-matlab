clear all;close all;clc;
%Simulation file: 
%Run the initialize script for reference generation:
Init
%Simulation time step: 
Tsim=Ts;
%Time vector for simulation:
tsim=0:Tsim:N;
%initialize vectors for simulation:
X=zeros(2,length(tsim)); %States [x;y] 
vx=zeros(1,length(tsim)); %MPC output of velocity on x
vy=zeros(1,length(tsim)); %MPC output of velocity on y
%Initial states: 
X0=[0;0];
X(:,1)=X0;
%Start the Simulation:
for k=1:length(tsim)-1
u=MPCcodeStudent(ref(:,k:k+(H-1)),X(:,k),Ts,H); %Pass a reference that matches the sample time of
%MPC.
vx(k)=u(1,1);
vy(k)=u(2,1);
%Apply model: 
X(1,k+1)=X(1,k)+vx(k)*Tsim;
X(2,k+1)=X(2,k)+vy(k)*Tsim;
end
%Plotting:
h1=plot(X(1,:),X(2,:));
hold on 
h2=plot(ref(1,:),ref(2,:),'g');
hold on 
h3=plot(ref(1,:),-0.6*ones(1,length(ref(1,:))),'r');
xlabel('x','interpreter','latex')
ylabel('y','interpreter','latex')
title('MPC simulation','interpreter','latex')
grid on 
leg=legend('Actualt Trajectory',...
'Desired Trajectory','Constraint');
set(leg,'interpreter','latex');
figure 
subplot(2,1,1)
%Vx:
plot(tsim,vx)
xlabel('Time [s]','interpreter','latex')
ylabel('Velocity $v_{x}$ [m/s]','interpreter','latex')
leg=legend('$v_{x}$');
set(leg,'interpreter','latex');
grid on 
subplot(2,1,2)
%Vy
plot(tsim,vy)
xlabel('Time [s]','interpreter','latex')
ylabel('Velocity $v_{x}$ [m/s]','interpreter','latex')
leg=legend('$v_{x}$');
set(leg,'interpreter','latex');
grid on 