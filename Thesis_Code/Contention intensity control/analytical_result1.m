% Compare contention intensity between CSMA(CW=16) and CIC (Corresponding to Fig.16)
global Ntr;
global lambda;
global slot_size;
global T;

ci_CSMA=zeros(1,191);
ci_low=zeros(1,41);
ci_high=zeros(1,41);

slot_size = 16* (10^-6); % 16 us %
zz=2;%protocol parameter for CIC model
lambda=10;
data_rate=6*10^6;
Packet_size=250*8;
DIFS=64*10^(-6);
T=Packet_size/data_rate+DIFS+2*slot_size;
window_size=16;
p_backoff_size=1/((window_size-1)/2+1);
ave_U=(window_size-1)/2;

Num=1;
for Ntr=10:1:200
    [x,fval]=fsolve(@analytic2,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
    ci_CSMA(Num)=ci_CSMA(Num)+ave_U*(1-(1-x(1)*p_backoff_size)^(Ntr-1));
    Num=Num+1;
end

Num=1;
for Ntr=160:1:200
    ci_high(Num)=ci_high(Num)+lambda*(Ntr-1)*(T/2+slot_size*zz)/(1-lambda*(Ntr-1)*(T+slot_size*(zz-1)));
    Num=Num+1;
end

Num=1;
for Ntr=10:1:50
    ci_low(Num)=ci_low(Num)+lambda*(Ntr-1)*(T+slot_size*zz)/(1-lambda*(Ntr-1)*(T+slot_size*(zz-1)));
    Num=Num+1;
end

n=10:1:200;
plot(n,ci_CSMA,'*-')
hold on;
n=10:1:50;
plot(n,ci_low,'o-')
hold on;
n=160:1:200;
plot(n,ci_high,'s-')
grid on;
xlabel('Vehicle Density (vehicles/km)')
ylabel('Contention Intensity')
legend('Contention Intensity of CSMA','Contention Intensity of CIC in Low Vehicle Load',...
    'Contention Intensity of CIC in High Vehicle Load')