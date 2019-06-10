global Ntr;
global lambda;
global data_rate;
global Packet_size;
global repeat_times;
global T;
global slot_size;
slot_size = 16 * (10^-6); % 16 us %
repeat_times=20;
T = round(12000 * 10^-3/slot_size); % simulation time%
d_ratio=zeros(1,20);
d_ratio1=zeros(1,20);
d_ratio2=zeros(1,20);
d_ratio3=zeros(1,20);

for Ntr=10:10:200
lambda=2;
data_rate=12*10^6;
Packet_size=250*8;
[x,fval]=fsolve(@exp_hidden_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
d_ratio(Ntr/10)=d_ratio(Ntr/10)+x(5);
end
Y1=exp_hidden_Simu();

for Ntr=10:10:200
lambda=10;
data_rate=24*10^6;
Packet_size=250*8;
[x,fval]=fsolve(@exp_hidden_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
d_ratio1(Ntr/10)=d_ratio1(Ntr/10)+x(5);
end
Y2=exp_hidden_Simu();

for Ntr=10:10:200
lambda=10;
data_rate=24*10^6;
Packet_size=450*8;
[x,fval]=fsolve(@exp_hidden_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
d_ratio2(Ntr/10)=d_ratio2(Ntr/10)+x(5);
end
Y3=exp_hidden_Simu();

for Ntr=10:10:200
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;
[x,fval]=fsolve(@exp_hidden_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
d_ratio3(Ntr/10)=d_ratio3(Ntr/10)+x(5);
end
Y4=exp_hidden_Simu();

N=10:10:200;%vehicle density
grid on;
plot(N,d_ratio,'o-',N,d_ratio1,'d-',N,d_ratio2,'v-',N,d_ratio3,'*-')
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
axis([0 200 0 1])
legend('Analysis(12,2,200)','Analysis(24,10,200)',...
'Analysis(24,10,400)','Analysis(6,10,200)',...
'Simulation(12,2,200)','Simulation(24,10,200)','Simulation(24,10,400)',...
'Simulation(6,10,200)')