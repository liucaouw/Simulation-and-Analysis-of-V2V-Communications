%CSMA with large Contention Window(CW=127)
function A1=Simu_CSMAwLCW()
    global lambda;
    global data_rate;
    global Packet_size;
    global repeat_times1;
    global T;
    global slot_size;

    N =10; % get vehicle density
    Nm=200;%maximum vehicle density
    packet_time = round(Packet_size / (data_rate*slot_size))+2; % get packet time %
    DIFS=64*(10^-6)/slot_size;
    CW = 127;%window size%
    d_ratio=zeros(1,Nm/10);%initial delivery ratios under different vehicle density%
    ave_delay=zeros(1,Nm/10);%initial mean delay under different vehicle density%
    standard_deviation=zeros(1,Nm/10);%initial standard deviation under different vehicle density%
    delay_standard_deviation=zeros(1,Nm/10);
    ave_stand_delay=zeros(1,Nm/10);
    mmmm=zeros(1,Nm/10);
    collision_number=zeros(1,Nm/10);
    reception_time=zeros(1,Nm/10);
    
while N<=Nm %vehicle density%
    count = 0;
    k=1;
    every_delivery_ratio=zeros(1,repeat_times1);%used to calculate standard deviation%
    every_ave_delay=zeros(1,repeat_times1);
    every_stan_delay=zeros(1,repeat_times1);
    total_collision_number=0;
    gg=0;
    
   while k<=repeat_times1  % repeating times for one density%
    good_transfer=0;%initial packet number with no collision%
    bad_transfer=0;%initial packet number with collision%
    bad_transfer1=0;%initial packet number with collision_long delay%
    total_time = 0;%initial current time%
    collision_flag = 0;
    r = round(((randi([0,(1/lambda)*10^6],1,N))*10^-6/slot_size)+DIFS);% generate N counters randomly and put them into an array %
    DIFS_number=zeros(1,N); %set the number of DIFS which each vehicle experiences(doesn't include 1st DIFS)%
    backoff_flag=zeros(1,N);%set the backoff flag for each vehicle%
    delay=zeros(1,N);%initial vehicle delay%
    delay_total=zeros(1,N);
    to_delay=0;
    dev_delay=0;
    m=1;
    q=1/lambda*ones(1,N);% generate next N counters randomly and put them into an array %
    detect=zeros(1,N);%detect the delay in the first DIFS% 
    num=0;
    stan_dev_delay=[];
    long_delay=zeros(1,N);
    long_delay_flag=zeros(1,N);
    
    while total_time < T
        aaaa=0;%initial number of jumping packets happening collision in every transmission round%
        cccc=0;%initial number of backetoff packets happening collision in every transmission round%
        l=1;%initial number of jumping packets%
     if m<2 %first transmission%
        M= min(r); % find the vehicle with the minimum counter %
       
        for i = 1:N   % check if there are more than one nodes with same minimum counter %
            if (M == r(i))
                count = count + 1;         
            end
        end
        
        if count > 1
            collision_flag = 1; % possible collision occured %
        end
        
        
        if collision_flag ~= 1 % no collision occured %
            good_transfer=good_transfer+count;%here, count=1%
        else
            for i=1:N
                if (M == r(i)) 
                    if(DIFS_number(i)==0)%jumping
                       aaaa=aaaa+1;
                    end
                end
            end
            if(aaaa>0)%the case where collision occured only between jumping packets%
                good_transfer=good_transfer+1;
            end
        end
        
        for i=1:N
             if (M == r(i))                
                 if(aaaa>0)%only jumping
                    if(l<aaaa)
                       delay(i)=delay(i)+packet_time+DIFS; 
                       r(i) = DIFS;
                       DIFS_number(i)=DIFS_number(i)+1;
                       l=l+1;
                    else
                       delay(i)=delay(i)+DIFS+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;%the new counter of vehicle i
                       q(i)=1/lambda;%generate the next counter of vehicle i  
                       delay_total(i)=delay_total(i)+delay(i);
                    end
                 
                 else%collision_flag ~=1,aaaa=0
                       delay(i)=delay(i)+DIFS+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;
                       q(i)=1/lambda;
                       delay_total(i)=delay_total(i)+delay(i);
                end
                 
             else 
                if (r(i)-M-packet_time>DIFS)
                    r(i) = r(i)-M-packet_time; 
                elseif(r(i)-M-packet_time<=DIFS&&r(i)-M-packet_time>=0)
                    delay(i)=delay(i)+r(i)-M-packet_time;
                    r(i)=DIFS+DIFS-(r(i)-M-packet_time);
                    DIFS_number(i)=DIFS_number(i)+1;
                    detect(i)=1;
                elseif(r(i)-M-packet_time<0)
                    delay(i)=delay(i)+packet_time+M-r(i)+DIFS; 
                    r(i) = DIFS;
                    DIFS_number(i)=DIFS_number(i)+1;
                end
             end
         end
            
        Check_DIFS = min(r);%Check if DIFS is minimum%
        for i = 1:N
            if (Check_DIFS == r(i)&&DIFS_number(i)>0&&detect(i)==0&&backoff_flag(i)==0)
                r(i)=r(i)+(randi([0,CW],1,1));
                backoff_flag(i)=backoff_flag(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r);
            for i = 1:N
                if (Check_DIFS1 == r(i)&&DIFS_number(i)>0&&detect(i)>0&&backoff_flag(i)==0)
                   r(i)=r(i)+(randi([0,CW],1,1));
                   backoff_flag(i)=backoff_flag(i)+1;%start backoff%
                end
            end
        end
        
        
     elseif(m>=2)
        M = min(r); % find the vehicle with the minimum counter %
       
        for i = 1:N   % check if there are more than one vehicle with same minimum counter %
            if (M == r(i))
                count = count + 1;         
            end
        end
        
        if count > 1
            collision_flag = 1; % possible collision occured %
        end
        
        if collision_flag ~= 1 % no collision occured %
            good_transfer=good_transfer+count;%count=1%
        else
            for i=1:N
                if (M == r(i)) 
                    if (backoff_flag(i)>0)%backoff%
                       cccc=cccc+1;
                    elseif(DIFS_number(i)==0)%jumping%
                       aaaa=aaaa+1;
                    end
                end
            end
            if(cccc>1&&aaaa>0)%the case where probable collision occured between jumping and backoff packets%
                bad_transfer=bad_transfer+cccc;
                total_collision_number=total_collision_number+cccc;
                gg=gg+1;
            elseif(cccc==1&&aaaa>0)
                good_transfer=good_transfer+1;
            end
            if(cccc>1&&aaaa==0)%the case where probable collision occured only between backoff packets
                bad_transfer=bad_transfer+cccc;
                total_collision_number=total_collision_number+cccc;
                gg=gg+1;
            end
            if(cccc==0&&aaaa>1)
                good_transfer=good_transfer+1;
            end
        end
                               
            for i=1:N
              if (M == r(i))
                if(collision_flag == 1)
                   if(cccc>0&&aaaa>0)%jumping and backoff
                      if(DIFS_number(i)==0)
                       delay(i)=delay(i)+packet_time+DIFS; 
                       r(i) = DIFS;
                       DIFS_number(i)=DIFS_number(i)+1; 
                      elseif(backoff_flag(i)>0)
                       delay(i)=delay(i)+M+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS; 
                       q(i)=1/lambda;
                       DIFS_number(i)=0;
                       backoff_flag(i)=0;
                       detect(i)=0;
                       delay_total(i)=delay_total(i)+delay(i);
                       if(long_delay_flag(i)>0)
                           delay_total(i)=delay_total(i)+long_delay(i);
                           long_delay_flag(i)=0;
                           long_delay(i)=0;
                       end
                      end
                   elseif(cccc==0&&aaaa>0)%only jumping
                      if(l<aaaa)
                       delay(i)=delay(i)+packet_time+DIFS; 
                       r(i) = DIFS;
                       DIFS_number(i)=DIFS_number(i)+1;
                       l=l+1;
                      else
                       delay(i)=delay(i)+DIFS+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;%the new counter of vehicle i
                       q(i)=1/lambda;%generate the next counter of vehicle i  
                       delay_total(i)=delay_total(i)+delay(i);
                      end
                   elseif(cccc>0&&aaaa==0)%only backoff
                       delay(i)=delay(i)+M+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;%the new counter of vehicle i
                       q(i)=1/lambda;%generate the next counter of vehicle i  
                       DIFS_number(i)=0;
                       backoff_flag(i)=0;
                       detect(i)=0;
                       delay_total(i)=delay_total(i)+delay(i);
                       if(long_delay_flag(i)>0)
                           delay_total(i)=delay_total(i)+long_delay(i);
                           long_delay_flag(i)=0;
                           long_delay(i)=0;
                       end
                   end       
                else
                     if (DIFS_number(i)==0)%jump
                           delay(i)=delay(i)+DIFS+packet_time;
                           r(i) = round(q(i)/slot_size)-delay(i)+DIFS;
                           q(i)=1/lambda;
                           delay_total(i)=delay_total(i)+delay(i);
                     elseif (backoff_flag(i)>0)
                           delay(i)=delay(i)+M+packet_time;
                           r(i) = round(q(i)/slot_size)-delay(i)+DIFS; 
                           q(i)=1/lambda;
                           DIFS_number(i)=0;
                           backoff_flag(i)=0;
                           detect(i)=0;
                           delay_total(i)=delay_total(i)+delay(i);
                       if(long_delay_flag(i)>0)
                           delay_total(i)=delay_total(i)+long_delay(i);
                           long_delay_flag(i)=0;
                           long_delay(i)=0;
                       end
                     end
                 end
              else
                    if (DIFS_number(i)==0&&r(i)-M-packet_time>DIFS)
                       r(i) = r(i)-M-packet_time; 
                       
                    elseif(DIFS_number(i)==0&&r(i)-M-packet_time>=0&&r(i)-M-packet_time<=DIFS)
                       delay(i)=delay(i)+DIFS-(r(i)-M-packet_time);        
                       r(i)=DIFS+r(i)-M-packet_time;
                       DIFS_number(i)=DIFS_number(i)+1;
                       detect(i)=1;

                    elseif(DIFS_number(i)==0&&r(i)-M-packet_time<0)
                       delay(i)=delay(i)+packet_time+M-r(i)+DIFS; 
                       r(i) = DIFS;
                       DIFS_number(i)=DIFS_number(i)+1;

                    elseif(DIFS_number(i)>0&&backoff_flag(i)==0&&detect(i)==1)
                       delay(i)=delay(i)+packet_time+M;%ONLY occured when packet with detect(i)=1%
                       r(i) = DIFS;
                       DIFS_number(i)=DIFS_number(i)+1;

                    elseif(backoff_flag(i)>0)
                        delay(i)=delay(i)+M+packet_time;
                        if(delay(i)>=round(q(i)/slot_size))
                           long_delay_flag(i)=long_delay_flag(i)+1;
                           long_delay(i)=long_delay(i)+round(q(i)/slot_size);
                           g=delay(i)-round(q(i)/slot_size);
                           if(g<=DIFS)
                              delay(i)=0;
                              DIFS_number(i)=0;
                              delay(i)=delay(i)+g;
                              r(i)=DIFS+DIFS-g;
                              q(i)=1/lambda;
                              backoff_flag(i)=0;
                              DIFS_number(i)=DIFS_number(i)+1;
                              detect(i)=1;
                           elseif(g>DIFS)
                              delay(i)=0;
                              DIFS_number(i)=0;
                              delay(i)=delay(i)+g; 
                              r(i)=DIFS;
                              q(i)=1/lambda;
                              backoff_flag(i)=0;
                              DIFS_number(i)=DIFS_number(i)+1;
                              detect(i)=0;
                           end
                              bad_transfer1=bad_transfer1+1;
                        elseif(delay(i)<round(q(i)/slot_size))
                          r(i)=r(i)-M+DIFS;
                          backoff_flag(i)=backoff_flag(i)+1;%this code is not necessary%
                        end
                    end                 
              end
            end
            
        Check_DIFS = min(r);
        for i = 1:N
            if (Check_DIFS == r(i)&&DIFS_number(i)>0&&detect(i)==0&&backoff_flag(i)==0)
                r(i)=r(i)+(randi([0,CW],1,1));
                backoff_flag(i)=backoff_flag(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r);
            for i = 1:N
                if (Check_DIFS1 == r(i)&&DIFS_number(i)>0&&detect(i)>0&&backoff_flag(i)==0)
                   r(i)=r(i)+(randi([0,CW],1,1));
                   backoff_flag(i)=backoff_flag(i)+1;%start backoff%
                end
            end
        end

     end  

     for i=1:N
          if(DIFS_number(i)==0&&backoff_flag(i)==0&&delay(i)>0) 
              num=num+1;
              stan_dev_delay(num)=delay(i);
              delay(i)=0;
          end
     end
        
     m=m+1;
     total_time = total_time+M+packet_time;    
     count = 0;
     collision_flag = 0;  
    end
 
    
    for i=1:N
       to_delay=to_delay+delay_total(i);
    end
    
    for i=1:num
        dev_delay=dev_delay+stan_dev_delay(i);
    end
        dev_delay=dev_delay/num;
        
    %PDR
    d_ratio(N/10)=d_ratio(N/10)+good_transfer/(good_transfer+bad_transfer+bad_transfer1);
    every_delivery_ratio(k)=good_transfer/(good_transfer+bad_transfer+bad_transfer1);
    %delay
    ave_delay(N/10)=ave_delay(N/10)+to_delay/(good_transfer+bad_transfer+bad_transfer1);
    every_ave_delay(k)=to_delay*slot_size*1000/(good_transfer+bad_transfer+bad_transfer1);
    %standard deviation of delay
    for i=1:num
    every_stan_delay(k)=every_stan_delay(k)+((dev_delay-stan_dev_delay(i))*slot_size*1000)^2/num;
    end
    every_stan_delay(k)=sqrt(every_stan_delay(k));
    k=k+1; 
   end
   for i=1:repeat_times1
       ave_stand_delay(N/10)=ave_stand_delay(N/10)+every_stan_delay(i);
   end
   ave_stand_delay(N/10)=ave_stand_delay(N/10)/repeat_times1;
   ave_delay(N/10)=ave_delay(N/10)*slot_size*1000/repeat_times1;%delay in ms
   d_ratio(N/10)=d_ratio(N/10)/repeat_times1;
  for i=1:repeat_times1
      standard_deviation(N/10)=standard_deviation(N/10)+(every_delivery_ratio(i)-d_ratio(N/10))^2/repeat_times1;
      delay_standard_deviation(N/10)=delay_standard_deviation(N/10)+(every_ave_delay(i)-ave_delay(N/10))^2/repeat_times1;
      mmmm(N/10)=mmmm(N/10)+(ave_stand_delay(N/10)-every_stan_delay(i))^2/repeat_times1;
  end
  standard_deviation(N/10)=sqrt(standard_deviation(N/10));
  delay_standard_deviation(N/10)=sqrt(delay_standard_deviation(N/10));
  mmmm(N/10)=sqrt(mmmm(N/10));
  if(gg==0)
  collision_number(N/10)=0;
  else
  collision_number(N/10)=collision_number(N/10)+total_collision_number/gg;
  end
  reception_time(N/10)=reception_time(N/10)+1000/(lambda*d_ratio(N/10))+ave_delay(N/10);
  N=N+10;
end
  
A1(1,:)=d_ratio;
A1(2,:)=standard_deviation;
A1(3,:)=ave_delay;
A1(4,:)=delay_standard_deviation;
A1(5,:)=ave_stand_delay;
A1(6,:)=mmmm;
A1(7,:)=collision_number;
A1(8,:)=reception_time;