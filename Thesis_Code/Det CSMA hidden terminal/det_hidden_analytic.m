function F=det_hidden_analytic(x)
global Ntr;
global lambda;
global data_rate;
global Packet_size;
global slot_size;
window_size=16;
DIFS=64*(10^-6);
T=Packet_size/data_rate+DIFS+2*slot_size;
p_backoff_size=1/((window_size-1)/2+1);
ave_T=T;
ave_Tres=T/2;
ave_U=(window_size-1)/2;
%p,pb,ave_s,
F=[x(1)-lambda*x(2);%p=x(1),ave_S=x(2)
x(3)-(1-(1-x(1))*(1-x(4)))*(1-(1-x(1)*p_backoff_size)^(Ntr-1));%pdc=x(3),pb=x(4)
x(4)-(Ntr-1)*lambda*T*(1-x(3)/2);
x(2)-x(6)-ave_T;%ave_A=x(6)
x(6)-((1-x(1))*x(4)*(x(7)+ave_Tres)-(2*x(1)-x(1)^2)/(1-x(1)))*0.1-x(1)*x(7);%ave_B=x(7)
x(7)-(slot_size+x(8))*ave_U;%ave_Y=x(8)
x(8)-(1-(1-x(1)*p_backoff_size)^(Ntr-1))*T;
x(9)-1+Ntr*lambda*T*(1-x(3)/2);%P(H1)=x(9)
x(10)-(1-lambda*(T-2*DIFS))^Ntr;%P(H2)=x(10)
x(11)-1+(1-x(3))*x(9)*x(10);%x(11)=pc
x(5)-1+x(11)];%PDR=x(5)