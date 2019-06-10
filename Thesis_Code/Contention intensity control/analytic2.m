% CSMA analytical model (deterministic packet interarrival)
function F=analytic2(x)
global Ntr;
global lambda;
global slot_size;
global T;
window_size=16;
p_backoff_size=1/((window_size-1)/2+1);
ave_T=T;
ave_Tres=T/2;
ave_U=(window_size-1)/2;
%Çóp,pb,ave_s,
F=[x(1)-lambda*x(2);%p=x(1),ave_S=x(2)
x(3)-(1-(1-x(1))*(1-x(4)))*(1-(1-x(1)*p_backoff_size)^(Ntr-1));%pdc=x(3),pb=x(4)
x(5)-1+x(3);%PDR=x(5)
x(4)-(Ntr-1)*lambda*(T)*(1-(2-1)*x(3)/2);
x(2)-x(6)-ave_T;%ave_A=x(6)
x(6)-((1-x(1))*x(4)+(2*x(1)-x(1)^2)/(1-x(1)))*(x(7)+ave_Tres);%ave_B=x(7)
x(7)-(slot_size+x(8))*ave_U;%ave_Y=x(8)
x(8)-(1-(1-x(1)*p_backoff_size)^(Ntr-1))*T;
x(9)-(1-x(1))*(1-x(4))*x(6)^2-((1-x(1))*x(4)+(4*x(1)-3*x(1)^2+x(1)^3)/(1-x(1))^2)*...
(x(10)+x(11)+(x(6)-x(7)-ave_Tres)^2);%var_A=x(9),var_B=x(10),var_Tres=x(11)
x(11)-T^2/12;
x(10)-x(12)*(ave_U)-(slot_size+x(8))^2*x(13);%var_Y=x(12),var_U=x(13)
x(13)-(window_size-1)^2/12;
x(12)-(1-(1-x(1)*p_backoff_size)^(Ntr-1))*((1-x(1)*p_backoff_size)^(Ntr-1))*T^2;
x(14)-sqrt(x(9));
x(15)-x(2)];