%Results of CIC in different arguments (Corresponding to Fig.14)
global data_rate;
global Packet_size;
global repeat_times2;%Contention Intensity control 
global T;
global TT;
global slot_size;
global zz;
global lambda;
global Ntr;

repeat_times2=40;
slot_size = 16* (10^-6); % 16 us %
zz=2;
T = round(12000 * 10^-3/slot_size); % simulation time%
DIFS=64*10^(-6);

delay_1=zeros(1,20);
delay_2=zeros(1,20);
delay_3=zeros(1,20);
delay_4=zeros(1,20);
a=1;
b=1;
c=1;
d=1;

m=1;
while(m<2)
lambda=2;
data_rate=12*10^6;
Packet_size=250*8;
TT=Packet_size/data_rate+DIFS+2*slot_size;
Y1=Simu_CIC();
for Ntr=10:10:200
[x,fval]=fsolve(@analytic1,[0,0,0]);%Mean delay=x(1)
delay_1(a)=delay_1(a)+x(1)*1000;
a=a+1;
end
m=m+1;
end

m=1;
while(m<2)
lambda=10;
data_rate=12*10^6;
Packet_size=250*8;
TT=Packet_size/data_rate+DIFS+2*slot_size;
Y2=Simu_CIC();
for Ntr=10:10:200
[x,fval]=fsolve(@analytic1,[0,0,0]);%Mean delay=x(1)
delay_2(b)=delay_2(b)+x(1)*1000;
b=b+1;
end
m=m+1;
end


m=1;
while(m<2)
lambda=10;
data_rate=24*10^6;
Packet_size=250*8;
TT=Packet_size/data_rate+DIFS+2*slot_size;
Y3=Simu_CIC();
for Ntr=10:10:200
[x,fval]=fsolve(@analytic1,[0,0,0]);%Mean delay=x(1)
delay_3(c)=delay_3(c)+x(1)*1000;
c=c+1;
end
m=m+1;
end

m=1;
while(m<2)
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;
TT=Packet_size/data_rate+DIFS+2*slot_size;
Y4=Simu_CIC();
for Ntr=10:10:200
[x,fval]=fsolve(@analytic1,[0,0,0]);%Mean delay=x(1)
delay_4(d)=delay_4(d)+x(1)*1000;
d=d+1;
end
m=m+1;
end


N=10:10:200;%vehicle density
plot(N,delay_1,'o-',N,delay_2,'d-',N,delay_3,'v-',N,delay_4,'*-')
hold on;
errorbar(N,Y1(3,:),Y1(4,:),'+-')
hold on;
errorbar(N,Y2(3,:),Y2(4,:),'s-')
hold on;
errorbar(N,Y3(3,:),Y3(4,:),'p-')
hold on;
errorbar(N,Y4(3,:),Y4(4,:),'^-')
grid on;
xlabel('Vehicle Density(vehicles/km)')
ylabel('Mean Delay (ms)')
%axis([0 200 0 2.5])
legend('Analysis(12,2,200)','Analysis(24,10,200)',...
'Analysis(24,10,400)','Analysis(6,10,200)',...
'Simulation(12,2,200)','Simulation(24,10,200)','Simulation(24,10,400)',...
'Simulation(6,10,200)')