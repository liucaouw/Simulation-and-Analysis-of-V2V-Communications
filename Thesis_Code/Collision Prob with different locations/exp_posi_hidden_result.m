global Ntr;
global Nht;
global lambda;
global data_rate;
global Packet_size;
global slot_size;
slot_size = 16 * (10^-6); % 16 us %
V1=100;
V2=200;
V3=400;
p_collision1=zeros(1,V1*4);
p_collision2=zeros(1,V2*4);
p_collision3=zeros(1,V3*4);

Nht=V1/2;
Ntr=V1/2;
for i=1:4*V1
lambda=10;
data_rate=24*10^6;
Packet_size=450*8;
[x,fval]=fsolve(@hidd_posi_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
p_collision1(i)=p_collision1(i)+x(11);
if(i<=V1/2)
Ntr=Ntr+1;
end
if(i>V1/2&&i<V1)
    Nht=Nht+1;
end
if(i>3*V1&&i<7*V1/2)
    Nht=Nht-1;
end
if(i>7*V1/2)
    Ntr=Ntr-1;
end
end

Nht=V2/2;
Ntr=V2/2;
for i=1:4*V2
lambda=10;
data_rate=24*10^6;
Packet_size=450*8;
[x,fval]=fsolve(@hidd_posi_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
p_collision2(i)=p_collision2(i)+x(11);
if(i<=V2/2)
Ntr=Ntr+1;
end
if(i>V2/2&&i<V2)
    Nht=Nht+1;
end
if(i>3*V2&&i<7*V2/2)
    Nht=Nht-1;
end
if(i>7*V2/2)
    Ntr=Ntr-1;
end
end


Nht=V3/2;
Ntr=V3/2;
for i=1:4*V3
lambda=10;
data_rate=24*10^6;
Packet_size=450*8;
[x,fval]=fsolve(@hidd_posi_analytic,[0,0,0,0,0,0,0,0,0,0,0]);
p_collision3(i)=p_collision3(i)+x(11);
if(i<=V3/2)
Ntr=Ntr+1;
end
if(i>V3/2&&i<V3)
    Nht=Nht+1;
end
if(i>3*V3&&i<7*V3/2)
    Nht=Nht-1;
end
if(i>7*V3/2)
    Ntr=Ntr-1;
end
end

location=0:(1/V1):4-(1/V1);%vehicle density
plot(location,p_collision1,'*-')
hold on;
location=0:(1/V2):4-(1/V2);%vehicle density
plot(location,p_collision2,'o-')
hold on;
location=0:(1/V3):4-(1/V3);%vehicle density
plot(location,p_collision3,'+-')
grid on;
xlabel('Location')
ylabel('Packet Collision Ratio')
axis([0 4 0 1])
legend('Vehicle Desnsity 100','Vehicle Desnsity 200','Vehicle Desnsity 400')