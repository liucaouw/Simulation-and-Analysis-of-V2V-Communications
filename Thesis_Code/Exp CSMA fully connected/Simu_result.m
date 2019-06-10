global Ntr;
global lambda;
global data_rate;
global Packet_size;
global repeat_times;
global T;
global slot_size;
global nc;
repeat_times=5;
slot_size = 16* (10^-6); % 16 us %
T = round(10000 * 10^-3/slot_size); % simulation time%
d_ratio_1=zeros(1,40);
d_ratio_2=zeros(1,40);
d_ratio_3=zeros(1,40);
d_ratio_4=zeros(1,40);
delay_1=zeros(1,40);
delay_2=zeros(1,40);
delay_3=zeros(1,40);
delay_4=zeros(1,40);
stand_deviation_1=zeros(1,40);
stand_deviation_2=zeros(1,40);
stand_deviation_3=zeros(1,40);
stand_deviation_4=zeros(1,40);
a=1;
b=1;
c=1;
d=1;

m=1;
while(m<2)
lambda=2;
data_rate=12*10^6;
Packet_size=250*8;
Y1=exp_Simu_CSMA();
for Ntr=10:10:400
nc=Y1(7,a);
if(nc==0)
nc=2;
end
[x,fval]=fsolve(@exp_analytic,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
d_ratio_1(a)=d_ratio_1(a)+x(5);
delay_1(a)=delay_1(a)+x(15)*1000;
stand_deviation_1(a)=stand_deviation_1(a)+x(14)*1000;
a=a+1;
end
m=m+1;
end

m=1;
while(m<2)
lambda=10;
data_rate=24*10^6;
Packet_size=250*8;
Y2=exp_Simu_CSMA();
for Ntr=10:10:400
nc=Y2(7,b);
if(nc==0)
nc=2;
end
[x,fval]=fsolve(@exp_analytic,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
d_ratio_2(b)=d_ratio_2(b)+x(5);
delay_2(b)=delay_2(b)+x(15)*1000;
stand_deviation_2(b)=stand_deviation_2(b)+x(14)*1000;
b=b+1;
end
m=m+1;
end

m=1;
while(m<2)
lambda=10;
data_rate=24*10^6;
Packet_size=450*8;
Y3=exp_Simu_CSMA();
for Ntr=10:10:400
nc=Y3(7,c);
if(nc==0)
nc=2;
end
[x,fval]=fsolve(@exp_analytic,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
d_ratio_3(c)=d_ratio_3(c)+x(5);
delay_3(c)=delay_3(c)+x(15)*1000;
stand_deviation_3(c)=stand_deviation_3(c)+x(14)*1000;
c=c+1;
end
m=m+1;
end

m=1;
while(m<2)
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;
Y4=exp_Simu_CSMA();
for Ntr=10:10:400
nc=Y4(7,d);
if(nc==0)
nc=2;
end
[x,fval]=fsolve(@exp_analytic,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
d_ratio_4(d)=d_ratio_4(d)+x(5);
delay_4(d)=delay_4(d)+x(15)*1000;
stand_deviation_4(d)=stand_deviation_4(d)+x(14)*1000;
d=d+1;
end
m=m+1;
end

figure(1)
N=10:10:400;%vehicle density
grid on;
plot(N,d_ratio_1,'o-',N,d_ratio_2,'d-',N,d_ratio_3,'v-',N,d_ratio_4,'*-')
hold on;
errorbar(N,Y1(1,:),Y1(2,:),'+-')
hold on;
errorbar(N,Y2(1,:),Y2(2,:),'s-')
hold on;
errorbar(N,Y3(1,:),Y3(2,:),'p-')
hold on;
errorbar(N,Y4(1,:),Y4(2,:),'^-')
xlabel('Vehicle Density(vehicles/km)')
ylabel('Packet Delivery Ratio')
axis([0 400 0.1 1])
legend('Analysis(12,2,200)','Analysis(24,10,200)',...
'Analysis(24,10,400)','Analysis(6,10,200)',...
'Simulation(12,2,200)','Simulation(24,10,200)','Simulation(24,10,400)',...
'Simulation(6,10,200)')
figure(2)
N=10:10:400;%vehicle density
grid on;
plot(N,delay_1,'o-',N,delay_2,'d-',N,delay_3,'v-',N,delay_4,'*-')
hold on;
errorbar(N,Y1(3,:),Y1(4,:),'+-')
hold on;
errorbar(N,Y2(3,:),Y2(4,:),'s-')
hold on;
errorbar(N,Y3(3,:),Y3(4,:),'p-')
hold on;
errorbar(N,Y4(3,:),Y4(4,:),'^-')
xlabel('Vehicle Density(vehicles/km)')
ylabel('Mean Delay (ms)')
axis([0 400 0 4])
legend('Analysis(12,2,200)','Analysis(24,10,200)',...
'Analysis(24,10,400)','Analysis(6,10,200)',...
'Simulation(12,2,200)','Simulation(24,10,200)','Simulation(24,10,400)',...
'Simulation(6,10,200)')
figure(3)
N=10:10:400;%vehicle density
grid on;
plot(N,stand_deviation_1,'o-',N,stand_deviation_2,'d-',N,stand_deviation_3,...
    'v-',N,stand_deviation_4,'*-')
hold on;
errorbar(N,Y1(5,:),Y1(6,:),'+-')
hold on;
errorbar(N,Y2(5,:),Y2(6,:),'s-')
hold on;
errorbar(N,Y3(5,:),Y3(6,:),'p-')
hold on;
errorbar(N,Y4(5,:),Y4(6,:),'^-')
xlabel('Vehicle Density(vehicles/km)')
ylabel('Standard Deviation (ms)')
axis([0 400 0 3])
legend('Analysis(12,2,200)','Analysis(24,10,200)',...
'Analysis(24,10,400)','Analysis(6,10,200)',...
'Simulation(12,2,200)','Simulation(24,10,200)','Simulation(24,10,400)',...
'Simulation(6,10,200)')