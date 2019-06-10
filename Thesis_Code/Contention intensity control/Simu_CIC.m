%Contention Intensity control 
function A1=Simu_CIC()
    global lambda;
    global data_rate;
    global Packet_size;
    global repeat_times2;
    global T;
    global slot_size;
    global zz;
    
    N =10; % get vehicle density
    Nm=200;%maximum vehicle density
    DIFS=64*(10^-6)/slot_size;
    packet_time = round(Packet_size / (data_rate*slot_size))+2; % get packet time 
    d_ratio=zeros(1,Nm/10);%initial delivery ratios under different vehicle density%
    ave_delay=zeros(1,Nm/10);%initial mean delay under different vehicle density%
    standard_deviation=zeros(1,Nm/10);%initial standard deviation under different vehicle density%
    delay_standard_deviation=zeros(1,Nm/10);
    
while N<=Nm %vehicle density%
    k=1;
    every_delivery_ratio=zeros(1,repeat_times2);%used to calculate standard deviation%
    every_ave_delay=zeros(1,repeat_times2);
    
   while k<=repeat_times2  % repeating times for one density%
    good_transfer=0;%initial packet number with no collision%
    bad_transfer=0;%initial packet number with collision%
    bad_transfer1=0;%initial packet number with collision_long delay%
    total_time = 0;%initial current time%
    r = round(((randi([0,round((1/lambda)*10^6)],1,N))*10^-6/slot_size)+DIFS);% generate N counters randomly and put them into an array %
    %r = round(((randi([0,round(1*10^6)],1,N))*10^-6/slot_size)+DIFS);% generate N counters randomly and put them into an array %
    backoff_flag=zeros(1,N);%set the backoff flag for each vehicle%
    delay=zeros(1,N);%initial vehicle delay%
    delay_total=zeros(1,N);
    to_delay=0;
    q=1/lambda*ones(1,N);% generate next N counters randomly and put them into an array %
    aaaaa=0;
    bb=zeros(1,N);
    long_delay=zeros(1,N);
    long_delay_flag=zeros(1,N);
    
    while total_time < T
        num=1;
        dd=[];
        ddd=[];
        count = 0;
        count1=0;
        cont_int=0;
        
        M = min(r); % find the vehicle with the minimum counter %
       
        for i = 1:N   % check if there are more than one vehicle with same minimum counter %
            if (M == r(i)&&backoff_flag(i)>0)
                count = count + 1;
            elseif (M == r(i)&&backoff_flag(i)==0)
                count1=count1+1;
            end
        end
        
        if count1>1
          aaaaa=aaaaa+count1;
        end
        
        if count==1
            good_transfer=good_transfer+count;%count=1%
        elseif count>1
            bad_transfer=bad_transfer+count;
        end
        
        for i=1:N
              if(backoff_flag(i)>0)
                 cont_int=cont_int+1;
              end
        end
        
       for i=1:N
           if (M == r(i))
               if (backoff_flag(i)>0)
                   delay(i)=delay(i)+M+packet_time;
                   r(i) = round(q(i)/slot_size)-delay(i)+DIFS; 
                   q(i)=1/lambda;
                   backoff_flag(i)=0;
                   delay_total(i)=delay_total(i)+delay(i);
                   delay(i)=0;
                   if(long_delay_flag(i)>0)
                       delay_total(i)=delay_total(i)+long_delay(i);
                       long_delay_flag(i)=0;
                       long_delay(i)=0;
                   end
               elseif (backoff_flag(i)==0)
                    if(count>0)
                       r(i)=zz*(cont_int+1)+DIFS;
                       delay(i)=delay(i)+packet_time+DIFS;
                       backoff_flag(i)=backoff_flag(i)+1;  
                    else
                        r(i)=zz*(cont_int+1);
                        delay(i)=delay(i)+DIFS;
                        backoff_flag(i)=backoff_flag(i)+1;
                    end
               end
           else
               if (backoff_flag(i)==0)
                   if(count>0)
                       if (r(i)-M-packet_time>DIFS)
                          r(i)=r(i)-M-packet_time;
                       elseif (r(i)-M<=packet_time+DIFS)  
                          dd(num)=r(i);
                          ddd(num)=i;
                          num=num+1;
                          if(r(i)-M-packet_time>0)
                              bb(i)=1;
                          end
                       end
                   else
                     r(i)=r(i)-M; 
                   end
               elseif (backoff_flag(i)>0)
                   if(count>0)
                      r(i)=r(i)-M+DIFS;
                      delay(i)=delay(i)+M+packet_time;
                      if delay(i)>round(1/lambda/slot_size)
                         long_delay_flag(i)=long_delay_flag(i)+1;
                         long_delay(i)=long_delay(i)+round(q(i)/slot_size);
                         bad_transfer1=bad_transfer1+1;
                         r(i)=round((1/lambda/slot_size))-(delay(i)-(M+packet_time));
                         backoff_flag(i)=0;
                         dd(num)=r(i);
                         ddd(num)=i;
                         num=num+1;
                      end
                   else
                      r(i)=r(i)-M;
                      delay(i)=delay(i)+M;
                   end
               end            
           end
       end
       cont_int=cont_int+count1;
 
       ss=0;
       Check = min(dd);
        for i=1:num-1
            if (dd(i)==Check)
                ss=ss+1;
            end
        end
        
       while (Check<100)
               for i=1:num-1
                   if (dd(i)==Check)
                      r(ddd(i))=zz*(cont_int+1)+DIFS;
                      if(bb(ddd(i))==1)
                          delay(ddd(i))=delay(ddd(i))+DIFS;
                          bb(ddd(i))=0;
                      else
                          delay(ddd(i))=delay(ddd(i))+packet_time-(dd(i)-M)+DIFS;
                      end
                      backoff_flag(ddd(i))=1; 
                      dd(i)=dd(i)+100;
                   end
               end
               cont_int=cont_int+ss;
           ss=0;
           Check = min(dd);
           for i=1:num-1
              if (dd(i)==Check)
                 ss=ss+1;
              end
           end
       end

       
       if (count>0)
        total_time = total_time+M+packet_time;
       else
        total_time = total_time+M;
       end
     end 
    
    for i=1:N
       to_delay=to_delay+delay_total(i);
    end
     
    %PDR
    d_ratio(N/10)=d_ratio(N/10)+good_transfer/(good_transfer+bad_transfer+bad_transfer1);
    every_delivery_ratio(k)=good_transfer/(good_transfer+bad_transfer+bad_transfer1);
    %delay
    ave_delay(N/10)=ave_delay(N/10)+to_delay/(good_transfer+bad_transfer+bad_transfer1);
    every_ave_delay(k)=to_delay*slot_size*1000/(good_transfer+bad_transfer+bad_transfer1);
    k=k+1; 
    end
   ave_delay(N/10)=ave_delay(N/10)*slot_size*1000/repeat_times2;%delay in ms
   d_ratio(N/10)=d_ratio(N/10)/repeat_times2;
    for i=1:repeat_times2
      standard_deviation(N/10)=standard_deviation(N/10)+(every_delivery_ratio(i)-d_ratio(N/10))^2/repeat_times2;
      delay_standard_deviation(N/10)=delay_standard_deviation(N/10)+(every_ave_delay(i)-ave_delay(N/10))^2/repeat_times2;
    end
    standard_deviation(N/10)=sqrt(standard_deviation(N/10));
    delay_standard_deviation(N/10)=sqrt(delay_standard_deviation(N/10));
   N=N+10;
end

A1(1,:)=d_ratio;
A1(2,:)=standard_deviation;
A1(3,:)=ave_delay;
A1(4,:)=delay_standard_deviation;