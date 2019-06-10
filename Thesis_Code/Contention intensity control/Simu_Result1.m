%ART under CSMA and CICRC (Corresponding to Fig.18)
global data_rate;
global Packet_size;
global repeat_times;%Contention control with rate control
global repeat_times1;%CSMA with rate control
global repeat_times2;%CSMA with large contention window with rate control
global T;
global slot_size;
global zz;
global rg;
global lambda;

rg=0.85;%load thresold
repeat_times=100;
repeat_times1=30;
repeat_times2=30;
slot_size = 16* (10^-6); % 16 us %
zz=2;
T = round(12000 * 10^-3/slot_size); % simulation time%
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;

Y1=Simu_CICRC();
Y2=Simu_CSMARC();
Y3=Simu_CSMAwLCWwRC();

n=10:10:300;%vehicle density
plot(n,Y1(5,:),'o-')
hold on;
plot(n,Y2(8,:),'*-')
hold on;
plot(n,Y3(8,:),'s-')
grid on;
xlabel('Vehicle Density(vehicles/km)')
ylabel('Average Successful Reception Time(ms)')
%axis([0 400 100 200])
legend('Contention Intensity Control with Rate Control',...
    'CSMA-CW=16 with Rate Control','CSMA-CW=128 with rate control')