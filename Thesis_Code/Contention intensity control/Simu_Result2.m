% Mean delay under CSMA and CIC (Corresponding to Fig.15)
global data_rate;
global Packet_size;
global repeat_times;%CSMA (fully connected vehicular network)
global repeat_times1;%CSMA with large Contention Window
global repeat_times2;%Contention Intensity control 
global T;
global slot_size;
global zz;
global lambda;

repeat_times=5;
repeat_times1=5;
repeat_times2=15;
slot_size = 16* (10^-6); % 16 us %
zz=2;
T = round(12000 * 10^-3/slot_size); % simulation time%
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;

Y1=Simu_CSMA();
Y2=Simu_CSMAwLCW();
Y3=Simu_CIC();

figure(1)
n=10:10:200;%vehicle density
errorbar(n,Y1(1,:),Y1(2,:),'o-')
hold on;
errorbar(n,Y2(1,:),Y2(2,:),'*-')
hold on;
errorbar(n,Y3(1,:),Y3(2,:),'s-')
grid on;
xlabel('Vehicle Density(vehicles/km)')
ylabel('Packet Delivery Ratio')
axis([0 200 0.8 1])
legend('CSMA-CW=16','CSMA-CW=128',...
'Contention Intensity Control')
figure(2)
n=10:10:200;%vehicle density
errorbar(n,Y1(3,:),Y1(4,:),'o-')
hold on;
errorbar(n,Y2(3,:),Y2(4,:),'*-')
hold on;
errorbar(n,Y3(3,:),Y3(4,:),'*-')
grid on;
xlabel('Vehicle Density(vehicles/km)')
ylabel('Mean Delay(ms)')
legend('CSMA-CW=16','CSMA-CW=128',...
'Contention Intensity Control')
