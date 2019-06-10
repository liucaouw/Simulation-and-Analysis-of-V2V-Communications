% Simulation on PDR under CSMA and CIC (Corresponding to Fig.17)
global data_rate;
global Packet_size;
global repeat_times;%CSMA (fully connected vehicular network)
global repeat_times1;%CSMA with large Contention Window
global repeat_times2;%Contention Intensity control 
global T;
global slot_size;
global zz;
global lambda;
global TT;
global Ntr;

lambda=10;
DIFS=64*10^(-6);
slot_size = 16 * (10^-6); % 16 us %
data_rate =6*10^6;% 6 Mbps %
Packet_size = 250*8;% get the packet size in bits % 
TT=Packet_size/data_rate+DIFS+2*slot_size;%Duration of transmitting a packet
repeat_times=60;
repeat_times1=40;
repeat_times2=150;
zz=2;
T = round(12000 * 10^-3/slot_size); % simulation time%
pdr=zeros(1,191);
dd=ones(1,191);
d=ones(1,191);
E=zeros(1,191);
Pc=ones(1,191);
Num=1;

Y1=exp_PDR1();
Y2=exp_PDR3();
Y3=exp_PDR4();




for Ntr=10:1:200
   m=0;
   w=2;
     while w<=Ntr/2
       p=w;
       d(Num)=1;
       m=0;
       x=1;
       while m<Ntr
           if m<Ntr-w
             d(Num)=d(Num)*(6250-m)/6250;
             m=m+1;
           else
             d(Num)=d(Num)*((6250-m-1)/6250)*(1/6250);
             m=m+2;
           end
       end
       while p>0
         x=x*(p-1);
         p=p-2;
       end
       d(Num)=d(Num)*prod(Ntr-w+1:Ntr)/prod(1:w)*x;
       E(Num)=E(Num)+w*d(Num);
       w=w+2;
     end
       Pc(Num)=Pc(Num)*E(Num)/Ntr;
   [x,fval]=fsolve(@analytic1,[0,0,0]);%Pck(0)=x(3)
   a1=(1-x(3))*(1-(1-lambda*slot_size)^(Ntr-E(Num)/2));
   ak=(1-x(3))*(1-(1-lambda*TT)^(Ntr-E(Num)/2));
   c=-a1*(1-lambda*(Ntr-E(Num)/2)*(TT-slot_size))-(ak-a1)*lambda*(Ntr-E(Num)/2)*slot_size/(1-x(3));
   b=1-lambda*(Ntr-E(Num)/2)*(TT-slot_size)-a1;
   a=1;
   pdr(Num)=pdr(Num)+1-((1-Pc(Num))*(-b+(b^2-4*a*c)^0.5)/2+Pc(Num));
   Num=Num+1;
end

n=10:10:200;%vehicle density
errorbar(n,Y1(1,:),Y1(2,:),'o-')
hold on;
errorbar(n,Y2(1,:),Y2(2,:),'*-')
hold on;
errorbar(n,Y3(1,:),Y3(2,:),'s-')
hold on;
n=10:1:200;%vehicle density
plot(n,pdr,'*-')
grid on;
xlabel('Vehicle Density(vehicles/km)')
ylabel('Packet Delivery Ratio')
axis([0 200 0.8 1])
legend('CSMA-CW=16','CSMA-CW=128',...
'Contention Intensity Control','Lower Bound for Contention Intensity Control')