% Braden Guthrie and Noe Avila
% EENG 350 - Section B
% Mini Project Assignment
% This file runs the motor control simulink file given data from the motors.
% It will output graphs of velocity and voltage of the motors.

close all;

K=2.2; % DC gain [rad/Vs]
sigma=10; % time constant reciprocal [1/s]
Kp = 11;
open_system('motor_control')

% run the simulation

out=sim('motor_control');


%load data from arduino
load('stepdata.mat')

%graph each plot of voltage and velocity
figure
subplot(2,1,1)
plot(out.Voltage,'--','linewidth',2)
hold on
plot(data(:,1),data(:,2),'linewidth',2)
legend('Simulated','Experimental','location','southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.DesiredVelocity,'--','linewidth',2)
hold on
plot(out.Velocity,'linewidth',2)
hold off
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
