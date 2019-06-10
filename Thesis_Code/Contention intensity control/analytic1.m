%This function is bulit to get the mean packet delay,the expected number of
%contending packets and the probability that no packet is contending for the channel
%in CICRC analytical model.

function F=analytic1(x)
global Ntr;
global lambda;
global slot_size;
global TT;
global zz;

F=[x(1)-(x(2)+1-0.5*(1-x(3)))*TT-(zz*(x(2)+1)-x(2))*slot_size %x(1):The mean packet delay
   x(2)-(Ntr-1)*lambda*x(1) %x(2):The expected number of contending packets
   x(3)-(1-x(2)/(Ntr-1))^(Ntr-1)]; %x(3):The probability that no packet is contending for the channel