% modified in Dec 14, 2022
close all, clear all, clc, format long,

disp(' --- this procedure can take some minutes to complete ---')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  RLC elements 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R1 = 14;
L = 1.54e-3;
C1 = 0.1e-6;
D = 4.5;
h = 19*10^(-6);  %sampling time of the 'Arduino Due' board


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CLOSED-LOOP data: RLC oscillator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 20221211-0001.mat

D = 4.5;

%
G = 5 + 80/5;  % gain factor set in the INA126 amplifier
x1real = -inv(R1*G)*10*B;   % factor of 10X from the measurement cable
x2real = 10*C + 0.38;
vecU = 10*A;  % factor of 10X from the measurement cable

TS = Tinterval;
t = TS*[1:max(size(A))];

figure(61),

subplot(3,1,1)
hold on
plot(TS*[1:max(size(x2real))],x2real,'k')
hold off
ylabel('x2(t)')
legend('x2real'); grid;

subplot(3,1,2)
hold on
plot(TS*[1:max(size(x1real))],x1real,'r')
hold off
ylabel('x1(t)')
legend('x1real'); grid;

subplot(3,1,3)
hold on
plot(TS*[1:max(size(vecU))],vecU,'b')
%plot(t,sine_x1,'k')
hold off
legend('vecU'); grid;
ylabel('u(t)')
xlabel('time (sec.)');


savefile = 'data_experiment_RLC_closedloop_deadbeat.mat';
save(savefile,'x1real','x2real','vecU','TS','-v7');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  procedure to compute the long-run
%  average cost (LRAC)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A = [-R1/L  -1/L;
       1/C1     0;];

B = D*[1/L ;0];   

Nit=1000;

G = -1*[0.5*21 -0.1515];

Acl = (eye(2,2)+h*A) + h*B*G;
H = zeros(2,2);
H(1,1) = 1.72497*10^(-3); H(2,2) = 4.53976*10^(-2);

%interval1 = [-14.5:0.025:-13.25];
%interval2 = [0.17:0.00025:0.19];

%interval1 = [-13.9860:0.00001:-13.9850];
%interval2 = [0.1783:0.000001:0.1787];

% part 1

interval1 = [-34:0.05:-5];
interval2 = [0.06:0.005:0.24];

[G1,G2] = meshgrid(interval1,interval2);

[lin,col]=size(G1);

for i=1:lin
    for j=1:col
        G = [G1(i,j) G2(i,j) ];
        LRAC(i,j) = computeLRAC_Deadbeat(A,B,[G1(i,j) G2(i,j)], H, h, Nit);
        if LRAC(i,j)<10^4
            Spec(i,j)=0;
        else
            Spec(i,j)=1;
        end
    end
end

figure(51)
hold on
for i=1:lin
    for j=1:col
        if Spec(i,j)==0
            plot(G1(i,j), G2(i,j), 'go');
        else
            plot(G1(i,j), G2(i,j), 'r*');
        end
    end
end
hold off

% part 2

interval1 = [-14.5:0.025:-13.25];
interval2 = [0.17:0.00025:0.19];
[G1,G2] = meshgrid(interval1,interval2);

[lin,col]=size(G1);

for i=1:lin
    for j=1:col
        G = [G1(i,j) G2(i,j) ];
        LRAC(i,j) = computeLRAC_Deadbeat(A,B,[G1(i,j) G2(i,j)], H, h, Nit);
        if LRAC(i,j)<10^4
            Spec(i,j)=0;
        else
            Spec(i,j)=1;
        end
    end
end

vecG1=[]; vecG2=[]; vecCost=[];
for i=1:lin
    for j=1:col
        if Spec(i,j)==0
            vecG1 = [vecG1 G1(i,j)];
            vecG2 = [vecG2 G2(i,j)];
            vecCost = [vecCost LRAC(i,j)];
        end
    end
end
hold off

figure(52)
[xq,yq] = meshgrid(interval1,interval2);
vq = griddata(vecG1,vecG2,vecCost,xq,yq);
surf(xq,yq,vq,'EdgeColor','none');
hold on
surfc(xq,yq,vq,'EdgeColor','none')
hold off
xlabel('G1'), ylabel('G2'), zlabel('cost');

min_cost = min(vecCost)
for i=1:lin
    for j=1:col
         if (LRAC(i,j)==min_cost)
             index_min = [i j];
             G1_min = G1(i,j)
             G2_min = G2(i,j)
         end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data processing to evaluate noise 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

texto = sprintf('listaNoiseSensorOpenLoop.txt');

fid = fopen(texto);
tline = fgetl(fid);
count = 1;
while ischar(tline)
    nome{count} = sprintf('%s',tline);
    tline = fgetl(fid);
    eval(sprintf('load %s',nome{count}));
    
    G = 5 + 80/5;  % gain factor set in the INA126 amplifier
    x1real = -inv(R1*G)*10*B;   % factor of 10X from the measurement cable
    x2real = 10*A+0.38;  % factor of 10X from the measurement cable
    
    % clean some 'Inf' values measured by error in the digital oscilloscope
    heap_x1{count} = clearInfValuesVector(x1real,100);
    heap_x2{count} = clearInfValuesVector(x2real,100);  
    
    count = count+1;
    Ts = Tinterval;
end
fclose(fid);


Rep=max(size(heap_x2));

vecX1=[];  vecX2=[];
for j=1:Rep
    vecX1=[vecX1; heap_x1{j}];
    vecX2=[vecX2; heap_x2{j}];
end

mean_x1 = mean(vecX1);
mean_x2 = mean(vecX2)-0.02;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot to see noise for a sample
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load 20221202-0001_01.mat

G = 5 + 80/5;  % gain factor set in the INA126 amplifier
x1real = -inv(R1*G)*10*B;   % factor of 10X from the measurement cable
x2real = 10*A+0.38;  % factor of 10X from the measurement cable

TS = Tinterval;
t = TS*[1:max(size(A))];

sine_x1 = mean_x1 + 0.0056*sin( 3228*t+1.23);  %ideal sine wave of x1
sine_x2 = mean_x2 + 1.54*sin( 3228*t-0.23);    %ideal sine wave of x2

figure(10),
subplot(2,1,1)
hold on
plot(TS*[1:max(size(x2real))],x2real,'b')
plot(t,sine_x2,'k')
hold off
legend('x2real','ideal x2'); grid;

subplot(2,1,2)
hold on
plot(TS*[1:max(size(x2real))],x1real,'r')
plot(t,sine_x1,'k')
hold off
legend('x1real','ideal x1'); grid;


Rep=max(size(heap_x2));

vecDevX1=[];  vecDevX2=[];
for j=1:Rep
    vecDevX1=[vecDevX1; (sine_x1'- heap_x1{j})];
    vecDevX2=[vecDevX2; (sine_x2'- heap_x2{j})];
end

mean_x1 = mean(vecDevX1)
mean_x2 = mean(vecDevX2)

std_x1 = std(vecDevX1)
std_x2 = std(vecDevX2)

savefile = 'data_RLC_Oscillator_deadbeat_noise.mat';
save(savefile,'vecDevX1','vecDevX2','-v7');

%return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data from open-loop RLC experiment deadbeat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 20221202-op0002_03.mat

D = 4.5;

%
G = 5 + 80/5;  % gain factor set in the INA126 amplifier
x1real = -inv(R1*G)*10*C(5300:end-1000);   % factor of 10X from the measurement cable
x2real = B(5300:end-1000) + 0.38;
vecU = 10*A(5300:end-1000);  % factor of 10X from the measurement cable

TS = Tinterval;
t = TS*[1:max(size(A))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% deadbeat verification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear A, clear B, clear C,

A = [-R1/L  -1/L;
       1/C1     0;];

B = D*[1/L ;0];   

h = 19*10^(-6)  %sampling time of the 'Arduino Due' board
Ad = (eye(2,2) + h*A);
Bd = h*B;

Ad2 = [0.8272727  -0.01233766;
        190   1];
Bd2 = [0.05551948052;
          0];
K2 = acker(Ad2,Bd2,[0 0]) ; % gain to ensure deadbeat      

K = acker(Ad,Bd,[0 0])  % gain to ensure deadbeat


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x{1}=[x1real(1);
      x2real(1)];

for k=1:max(size(x1real))
    x{k+1} = (eye(2,2)+TS*A)*x{k} + TS*B*vecU(k);
end

vecX1=[];
vecX2=[];
for k=1:max(size(x1real))
    vecX1=[vecX1 x{k}(1)];
    vecX2=[vecX2 x{k}(2)];
end

figure(1),
hold on
plot(TS*[1:max(size(x2real))],vecU,'k')
plot(TS*[1:max(size(x2real))],vecX2,'b')
plot(TS*[1:max(size(x2real))],x2real,'r')
legend('vecU','simul','x2real');
grid
hold off

figure(2),
hold on
plot(TS*[1:max(size(x2real))],vecX1,'b')
plot(TS*[1:max(size(x2real))],x1real,'r')
legend('simul','x1real');
grid
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% noise in practice upon the diode D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load 20221130-0002_02.mat 

vecD2 = 10*A;
vecD1 = B;
TS = Tinterval;
t = TS*[1:max(size(vecD1))];

figure(3),
hold on
plot(t,vecD1-vecD2,'r')
legend('Voltage upon diode D');
grid
hold off


savefile = 'data_simulation_RLC_openloop_deadbeat.mat';
save(savefile,'-v7');

return
