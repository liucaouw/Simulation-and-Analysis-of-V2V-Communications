function A1=exp_hidden_Simu()
    global lambda;
    global data_rate;
    global Packet_size;
    global repeat_times;
    global T;
    global slot_size;
    N =10; % get vehicle density
    Nm=200;%maximum vehicle density
    packet_time = round(Packet_size / (data_rate*slot_size))+2; % get packet time %
    DIFS=64*(10^-6)/slot_size;
    CW = 15;%window size%
    d_ratio=zeros(1,Nm/10);%initial delivery ratios under different vehicle density%
    ave_delay=zeros(1,Nm/10);%initial mean delay under different vehicle density%
    standard_deviation=zeros(1,Nm/10);%initial standard deviation under different vehicle density%
    
while N<=Nm %vehicle density%
    count = 0;
    k=1;
    every_delivery_ratio=zeros(1,repeat_times);%used to calculate standard deviation%
    count_1=0;
    count_2=0;

   while k<=repeat_times  % repeating times for one density%
    good_transfer=0;%initial packet number with no collision%
    bad_transfer=0;%initial packet number with collision%
    total_time = 0;%initial current time%
    collision_flag = 0;
    r = round(exprnd(1/lambda,1,N)/slot_size+DIFS);% generate N counters randomly and put them into an array %
    DIFS_number=zeros(1,N); %set the number of DIFS which each vehicle experiences(doesn't include 1st DIFS)%
    backoff_flag=zeros(1,N);%set the backoff flag for each vehicle%
    delay=zeros(1,N);%initial vehicle delay%
    delay_total=zeros(1,N);
    to_delay=0;
    m=1;
    q=exprnd((1/lambda),1,N);% generate next N counters randomly and put them into an array %
    detect=zeros(1,N);%detect the delay in the first DIFS% 
    
    total_time_1 = 0;
    collision_flag_1 = 0;
    r_1 = round(exprnd(1/lambda,1,N)/slot_size+DIFS);% generate N counters randomly and put them into an array %
    DIFS_number_1=zeros(1,N); %set the number of DIFS which each vehicle experiences(doesn't include 1st DIFS)%
    backoff_flag_1=zeros(1,N);%set the backoff flag for each vehicle%
    delay_1=zeros(1,N);%initial vehicle delay%
    delay_total_1=zeros(1,N);
    m_1=1;
    q_1=exprnd((1/lambda),1,N);% generate next N counters randomly and put them into an array %
    detect_1=zeros(1,N);%detect the delay in the first DIFS% 
    hidden_time_record_1=[];
    transmission_flag1=zeros(1,N);
    
    total_time_2 = 0;
    collision_flag_2 = 0;
    r_2 = round(exprnd(1/lambda,1,N)/slot_size+DIFS);% generate N counters randomly and put them into an array %
    DIFS_number_2=zeros(1,N); %set the number of DIFS which each vehicle experiences(doesn't include 1st DIFS)%
    backoff_flag_2=zeros(1,N);%set the backoff flag for each vehicle%
    delay_2=zeros(1,N);%initial vehicle delay%
    delay_total_2=zeros(1,N);
    m_2=1;
    q_2=exprnd((1/lambda),1,N);% generate next N counters randomly and put them into an array %
    detect_2=zeros(1,N);%detect the delay in the first DIFS% 
    hidden_time_record_2=[];
    transmission_flag2=zeros(1,N);
  
    
    %hidden terminal district1
     while total_time_1 < T
        aaaa=0;%initial number of jumping packets happening collision in every transmission round%
        cccc=0;%initial number of backetoff packets happening collision in every transmission round%
        l=1;%initial number of jumping packets%
     if (m_1<2) %first transmission%
        M_1= min(r_1); % find the vehicle with the minimum counter %
       
        for i = 1:N  % check if there are more than one nodes with same minimum counter %
            if (M_1 == r_1(i))
                count_1 = count_1 + 1;         
            end
        end
        
        if count_1 > 1
            collision_flag_1 = 1; % possible collision occured %
        end
        
        
        if (collision_flag_1 == 1)
            for i=1:N
                if (M_1 == r_1(i)) 
                    if(DIFS_number_1(i)==0)%jumping
                       aaaa=aaaa+1;
                    end
                end
            end
        end
        
        for i=1:N
             if (M_1 == r_1(i))                
                 if(aaaa>0)%only jumping
                    if(l<aaaa)
                       delay_1(i)=delay_1(i)+packet_time+DIFS; 
                       r_1(i) = DIFS;
                       DIFS_number_1(i)=DIFS_number_1(i)+1;
                       l=l+1;
                    else
                       delay_1(i)=delay_1(i)+DIFS+packet_time;
                       r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS;%the new counter of vehicle i
                       q_1(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total_1(i)=delay_total_1(i)+delay_1(i);
                       transmission_flag1(i)=1;
                    end
                 
                 else%collision_flag ~=1,aaaa=0
                       delay_1(i)=delay_1(i)+DIFS+packet_time;
                       r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS;
                       q_1(i)=exprnd((1/lambda),1,1);
                       delay_total_1(i)=delay_total_1(i)+delay_1(i);
                       transmission_flag1(i)=1;
                end
                 
             else 
                if (r_1(i)-M_1-packet_time>DIFS)
                    r_1(i) = r_1(i)-M_1-packet_time; 
                elseif(r_1(i)-M_1-packet_time<=DIFS&&r_1(i)-M_1-packet_time>=0)
                    delay_1(i)=delay_1(i)+r_1(i)-M_1-packet_time;
                    r_1(i)=DIFS+DIFS-(r_1(i)-M_1-packet_time);
                    DIFS_number_1(i)=DIFS_number_1(i)+1;
                    detect_1(i)=1;
                elseif(r_1(i)-M_1-packet_time<0)
                    delay_1(i)=delay_1(i)+packet_time+M_1-r_1(i)+DIFS; 
                    r_1(i) = DIFS;
                    DIFS_number_1(i)=DIFS_number_1(i)+1;
                end
             end
         end
            
        Check_DIFS = min(r_1);%Check if DIFS is minimum%
        for i = 1:N
            if (Check_DIFS == r_1(i)&&DIFS_number_1(i)>0&&detect_1(i)==0&&backoff_flag_1(i)==0)
                r_1(i)=r_1(i)+(randi([0,CW],1,1));
                backoff_flag_1(i)=backoff_flag_1(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r_1);
            for i = 1:N
                if (Check_DIFS1 == r_1(i)&&DIFS_number_1(i)>0&&detect_1(i)>0&&backoff_flag_1(i)==0)
                   r_1(i)=r_1(i)+(randi([0,CW],1,1));
                   backoff_flag_1(i)=backoff_flag_1(i)+1;%start backoff%
                end
            end
        end
        
        
     elseif(m_1>=2)
        M_1 = min(r_1); % find the vehicle with the minimum counter %
       
        for i = 1:N % check if there are more than one vehicle with same minimum counter %
            if (M_1 == r_1(i))
                count_1 = count_1 + 1;         
            end
        end
        
        if count_1 > 1
            collision_flag_1 = 1; % possible collision occured %
        end
        
        if (collision_flag_1 == 1)
            for i=1:N
                if (M_1 == r_1(i)) 
                    if (backoff_flag_1(i)>0)%backoff%
                       cccc=cccc+1;
                    elseif(DIFS_number_1(i)==0)%jumping%
                       aaaa=aaaa+1;
                    end
                end
            end
        end
                               
            for i=1:N
              if (M_1== r_1(i))
                if(collision_flag_1 == 1)
                   if(cccc>0&&aaaa>0)%jumping and backoff
                      if(DIFS_number_1(i)==0)
                       delay_1(i)=delay_1(i)+packet_time+DIFS; 
                       r_1(i) = DIFS;
                       DIFS_number_1(i)=DIFS_number_1(i)+1; 
                      elseif(backoff_flag_1(i)>0)
                       delay_1(i)=delay_1(i)+M_1+packet_time;
                       r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS; 
                       q_1(i)=exprnd((1/lambda),1,1);
                       DIFS_number_1(i)=0;
                       backoff_flag_1(i)=0;
                       detect_1(i)=0;
                       delay_total_1(i)=delay_total_1(i)+delay_1(i);
                       transmission_flag1(i)=1;
                      end
                   elseif(cccc==0&&aaaa>0)%only jumping
                      if(l<aaaa)
                       delay_1(i)=delay_1(i)+packet_time+DIFS; 
                       r_1(i) = DIFS;
                       DIFS_number_1(i)=DIFS_number_1(i)+1;
                       l=l+1;
                      else
                       delay_1(i)=delay_1(i)+DIFS+packet_time;
                       r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS;%the new counter of vehicle i
                       q_1(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total_1(i)=delay_total_1(i)+delay_1(i);
                       transmission_flag1(i)=1;
                      end
                   elseif(cccc>0&&aaaa==0)%only backoff
                       delay_1(i)=delay_1(i)+M_1+packet_time;
                       r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS;%the new counter of vehicle i
                       q_1(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       DIFS_number_1(i)=0;
                       backoff_flag_1(i)=0;
                       detect_1(i)=0;
                       delay_total_1(i)=delay_total_1(i)+delay_1(i);
                       transmission_flag1(i)=1;
                   end       
                else
                     if (DIFS_number_1(i)==0)%jump
                           delay_1(i)=delay_1(i)+DIFS+packet_time;
                           r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS;
                           q_1(i)=exprnd((1/lambda),1,1);
                           delay_total_1(i)=delay_total_1(i)+delay_1(i);
                           transmission_flag1(i)=1;
                     elseif (backoff_flag_1(i)>0)
                           delay_1(i)=delay_1(i)+M_1+packet_time;
                           r_1(i) = round(q_1(i)/slot_size)-delay_1(i)+DIFS; 
                           q_1(i)=exprnd((1/lambda),1,1);
                           DIFS_number_1(i)=0;
                           backoff_flag_1(i)=0;
                           detect_1(i)=0;
                           delay_total_1(i)=delay_total_1(i)+delay_1(i);
                           transmission_flag1(i)=1;
                     end
                 end
              else
                    if (DIFS_number_1(i)==0&&r_1(i)-M_1-packet_time>DIFS)
                       r_1(i) = r_1(i)-M_1-packet_time; 
                       
                    elseif(DIFS_number_1(i)==0&&r_1(i)-M_1-packet_time>=0&&r_1(i)-M_1-packet_time<=DIFS)
                       delay_1(i)=delay_1(i)+DIFS-(r_1(i)-M_1-packet_time);        
                       r_1(i)=DIFS+r_1(i)-M_1-packet_time;
                       DIFS_number_1(i)=DIFS_number_1(i)+1;
                       detect_1(i)=1;

                    elseif(DIFS_number_1(i)==0&&r_1(i)-M_1-packet_time<0)
                       delay_1(i)=delay_1(i)+packet_time+M_1-r_1(i)+DIFS; 
                       r_1(i) = DIFS;
                       DIFS_number_1(i)=DIFS_number_1(i)+1;

                    elseif(DIFS_number_1(i)>0&&backoff_flag_1(i)==0&&detect_1(i)==1)
                       delay_1(i)=delay_1(i)+packet_time+M_1;%ONLY occured when packet with detect(i)=1%
                       r_1(i) = DIFS;
                       DIFS_number_1(i)=DIFS_number_1(i)+1;

                    elseif(backoff_flag_1(i)>0)
                        delay_1(i)=delay_1(i)+M_1+packet_time;
                        if(delay_1(i)>=round(q_1(i)/slot_size))
                           g=delay_1(i)-round(q_1(i)/slot_size);
                           if(g<=DIFS)
                              delay_1(i)=0;
                              DIFS_number_1(i)=0;
                              delay_1(i)=delay_1(i)+g;
                              r_1(i)=DIFS+DIFS-g;
                              q_1(i)=exprnd((1/lambda),1,1);
                              backoff_flag_1(i)=0;
                              DIFS_number_1(i)=DIFS_number_1(i)+1;
                              detect_1(i)=1;
                           elseif(g>DIFS)
                              delay_1(i)=0;
                              DIFS_number_1(i)=0;
                              delay_1(i)=delay_1(i)+g; 
                              r_1(i)=DIFS;
                              q_1(i)=exprnd((1/lambda),1,1);
                              backoff_flag_1(i)=0;
                              DIFS_number_1(i)=DIFS_number_1(i)+1;
                              detect_1(i)=0;
                           end
                        elseif(delay_1(i)<round(q_1(i)/slot_size))
                          r_1(i)=r_1(i)-M_1+DIFS;
                          backoff_flag_1(i)=backoff_flag_1(i)+1;%this code is not necessary%
                        end
                    end                 
              end
            end
            
        Check_DIFS = min(r_1);
        for i = 1:N
            if (Check_DIFS == r_1(i)&&DIFS_number_1(i)>0&&detect_1(i)==0&&backoff_flag_1(i)==0)
                r_1(i)=r_1(i)+(randi([0,CW],1,1));
                backoff_flag_1(i)=backoff_flag_1(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r_1);
            for i = 1:N
                if (Check_DIFS1 == r_1(i)&&DIFS_number_1(i)>0&&detect_1(i)>0&&backoff_flag_1(i)==0)
                   r_1(i)=r_1(i)+(randi([0,CW],1,1));
                   backoff_flag_1(i)=backoff_flag_1(i)+1;%start backoff%
                end
            end
        end

     end  

        for i=1:N
          if(DIFS_number_1(i)==0&&backoff_flag_1(i)==0)
             delay_1(i)=0;
          end
        end 
        
     kk=0;
     for i=1:1:N/2
         kk=kk+transmission_flag1(i);
     end
     if (kk>0)
     hidden_time_record_1(m_1)=total_time_1+M_1;
     m_1=m_1+1;
     for i=1:1:N/2
         if(transmission_flag1(i)==1)
         transmission_flag1(i)=0;
         end
     end
     end
     total_time_1 = total_time_1+M_1+packet_time;
     count_1 = 0;
     collision_flag_1 = 0;    
     end
     
     
    %hidden terminal district2
     while total_time_2 < T
        aaaa=0;%initial number of jumping packets happening collision in every transmission round%
        cccc=0;%initial number of backetoff packets happening collision in every transmission round%
        l=1;%initial number of jumping packets%
     if (m_2<2) %first transmission%
        M_2= min(r_2); % find the vehicle with the minimum counter %
       
        for i = 1:N  % check if there are more than one nodes with same minimum counter %
            if (M_2 == r_2(i))
                count_2 = count_2 + 1;         
            end
        end
        
        if count_2 > 1
            collision_flag_2 = 1; % possible collision occured %
        end
        
        
        if (collision_flag_2 == 1)
            for i=1:N
                if (M_2 == r_2(i)) 
                    if(DIFS_number_2(i)==0)%jumping
                       aaaa=aaaa+1;
                    end
                end
            end
        end
        
        for i=1:N
             if (M_2 == r_2(i))                
                 if(aaaa>0)%only jumping
                    if(l<aaaa)
                       delay_2(i)=delay_2(i)+packet_time+DIFS; 
                       r_2(i) = DIFS;
                       DIFS_number_2(i)=DIFS_number_2(i)+1;
                       l=l+1;
                    else
                       delay_2(i)=delay_2(i)+DIFS+packet_time;
                       r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS;%the new counter of vehicle i
                       q_2(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total_2(i)=delay_total_2(i)+delay_2(i);
                       transmission_flag2(i)=1;
                    end
                 
                 else%collision_flag ~=1,aaaa=0
                       delay_2(i)=delay_2(i)+DIFS+packet_time;
                       r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS;
                       q_2(i)=exprnd((1/lambda),1,1);
                       delay_total_2(i)=delay_total_2(i)+delay_2(i);
                       transmission_flag2(i)=1;
                end
                 
             else 
                if (r_2(i)-M_2-packet_time>DIFS)
                    r_2(i) = r_2(i)-M_2-packet_time; 
                elseif(r_2(i)-M_2-packet_time<=DIFS&&r_2(i)-M_2-packet_time>=0)
                    delay_2(i)=delay_2(i)+r_2(i)-M_2-packet_time;
                    r_2(i)=DIFS+DIFS-(r_2(i)-M_2-packet_time);
                    DIFS_number_2(i)=DIFS_number_2(i)+1;
                    detect_2(i)=1;
                elseif(r_2(i)-M_2-packet_time<0)
                    delay_2(i)=delay_2(i)+packet_time+M_2-r_2(i)+DIFS; 
                    r_2(i) = DIFS;
                    DIFS_number_2(i)=DIFS_number_2(i)+1;
                end
             end
         end
            
        Check_DIFS = min(r_2);%Check if DIFS is minimum%
        for i = 1:N
            if (Check_DIFS == r_2(i)&&DIFS_number_2(i)>0&&detect_2(i)==0&&backoff_flag_2(i)==0)
                r_2(i)=r_2(i)+(randi([0,CW],1,1));
                backoff_flag_2(i)=backoff_flag_2(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r_2);
            for i = 1:N
                if (Check_DIFS1 == r_2(i)&&DIFS_number_2(i)>0&&detect_2(i)>0&&backoff_flag_2(i)==0)
                   r_2(i)=r_2(i)+(randi([0,CW],1,1));
                   backoff_flag_2(i)=backoff_flag_2(i)+1;%start backoff%
                end
            end
        end
        
        
     elseif(m_2>=2)
        M_2 = min(r_2); % find the vehicle with the minimum counter %
       
        for i = 1:N % check if there are more than one vehicle with same minimum counter %
            if (M_2 == r_2(i))
                count_2 = count_2 + 1;         
            end
        end
        
        if count_2 > 1
            collision_flag_2 = 1; % possible collision occured %
        end
        
        if (collision_flag_2 == 1)
            for i=1:N
                if (M_2 == r_2(i)) 
                    if (backoff_flag_2(i)>0)%backoff%
                       cccc=cccc+1;
                    elseif(DIFS_number_2(i)==0)%jumping%
                       aaaa=aaaa+1;
                    end
                end
            end
        end
                               
            for i=1:N
              if (M_2== r_2(i))
                if(collision_flag_2 == 1)
                   if(cccc>0&&aaaa>0)%jumping and backoff
                      if(DIFS_number_2(i)==0)
                       delay_2(i)=delay_2(i)+packet_time+DIFS; 
                       r_2(i) = DIFS;
                       DIFS_number_2(i)=DIFS_number_2(i)+1; 
                      elseif(backoff_flag_2(i)>0)
                       delay_2(i)=delay_2(i)+M_2+packet_time;
                       r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS; 
                       q_2(i)=exprnd((1/lambda),1,1);
                       DIFS_number_2(i)=0;
                       backoff_flag_2(i)=0;
                       detect_2(i)=0;
                       delay_total_2(i)=delay_total_2(i)+delay_2(i);
                       transmission_flag2(i)=1;
                      end
                   elseif(cccc==0&&aaaa>0)%only jumping
                      if(l<aaaa)
                       delay_2(i)=delay_2(i)+packet_time+DIFS; 
                       r_2(i) = DIFS;
                       DIFS_number_2(i)=DIFS_number_2(i)+1;
                       l=l+1;
                      else
                       delay_2(i)=delay_2(i)+DIFS+packet_time;
                       r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS;%the new counter of vehicle i
                       q_2(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total_2(i)=delay_total_2(i)+delay_2(i);
                       transmission_flag2(i)=1;
                      end
                   elseif(cccc>0&&aaaa==0)%only backoff
                       delay_2(i)=delay_2(i)+M_2+packet_time;
                       r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS;%the new counter of vehicle i
                       q_2(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       DIFS_number_2(i)=0;
                       backoff_flag_2(i)=0;
                       detect_2(i)=0;
                       delay_total_2(i)=delay_total_2(i)+delay_2(i);
                       transmission_flag2(i)=1;
                   end       
                else
                     if (DIFS_number_2(i)==0)%jump
                           delay_2(i)=delay_2(i)+DIFS+packet_time;
                           r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS;
                           q_2(i)=exprnd((1/lambda),1,1);
                           delay_total_2(i)=delay_total_2(i)+delay_2(i);
                           transmission_flag2(i)=1;
                     elseif (backoff_flag_2(i)>0)
                           delay_2(i)=delay_2(i)+M_2+packet_time;
                           r_2(i) = round(q_2(i)/slot_size)-delay_2(i)+DIFS; 
                           q_2(i)=exprnd((1/lambda),1,1);
                           DIFS_number_2(i)=0;
                           backoff_flag_2(i)=0;
                           detect_2(i)=0;
                           delay_total_2(i)=delay_total_2(i)+delay_2(i);
                           transmission_flag2(i)=1;
                     end
                 end
              else
                    if (DIFS_number_2(i)==0&&r_2(i)-M_2-packet_time>DIFS)
                       r_2(i) = r_2(i)-M_2-packet_time; 
                       
                    elseif(DIFS_number_2(i)==0&&r_2(i)-M_2-packet_time>=0&&r_2(i)-M_2-packet_time<=DIFS)
                       delay_2(i)=delay_2(i)+DIFS-(r_2(i)-M_2-packet_time);        
                       r_2(i)=DIFS+r_2(i)-M_2-packet_time;
                       DIFS_number_2(i)=DIFS_number_2(i)+1;
                       detect_2(i)=1;

                    elseif(DIFS_number_2(i)==0&&r_2(i)-M_2-packet_time<0)
                       delay_2(i)=delay_2(i)+packet_time+M_2-r_2(i)+DIFS; 
                       r_2(i) = DIFS;
                       DIFS_number_2(i)=DIFS_number_2(i)+1;

                    elseif(DIFS_number_2(i)>0&&backoff_flag_2(i)==0&&detect_2(i)==1)
                       delay_2(i)=delay_2(i)+packet_time+M_2;%ONLY occured when packet with detect(i)=1%
                       r_2(i) = DIFS;
                       DIFS_number_2(i)=DIFS_number_2(i)+1;

                    elseif(backoff_flag_2(i)>0)
                        delay_2(i)=delay_2(i)+M_2+packet_time;
                        if(delay_2(i)>=round(q_2(i)/slot_size))
                           g=delay_2(i)-round(q_2(i)/slot_size);
                           if(g<=DIFS)
                              delay_2(i)=0;
                              DIFS_number_2(i)=0;
                              delay_2(i)=delay_2(i)+g;
                              r_2(i)=DIFS+DIFS-g;
                              q_2(i)=exprnd((1/lambda),1,1);
                              backoff_flag_2(i)=0;
                              DIFS_number_2(i)=DIFS_number_2(i)+1;
                              detect_2(i)=1;
                           elseif(g>DIFS)
                              delay_2(i)=0;
                              DIFS_number_2(i)=0;
                              delay_2(i)=delay_2(i)+g; 
                              r_2(i)=DIFS;
                              q_2(i)=exprnd((1/lambda),1,1);
                              backoff_flag_2(i)=0;
                              DIFS_number_2(i)=DIFS_number_2(i)+1;
                              detect_2(i)=0;
                           end
                        elseif(delay_2(i)<round(q_2(i)/slot_size))
                          r_2(i)=r_2(i)-M_2+DIFS;
                          backoff_flag_2(i)=backoff_flag_2(i)+1;%this code is not necessary%
                        end
                    end                 
              end
            end
            
        Check_DIFS = min(r_2);
        for i = 1:N
            if (Check_DIFS == r_2(i)&&DIFS_number_2(i)>0&&detect_2(i)==0&&backoff_flag_2(i)==0)
                r_2(i)=r_2(i)+(randi([0,CW],1,1));
                backoff_flag_2(i)=backoff_flag_2(i)+1;%start backoff%
            end
        end
        
        for j=1:4
            Check_DIFS1 = min(r_2);
            for i = 1:N
                if (Check_DIFS1 == r_2(i)&&DIFS_number_2(i)>0&&detect_2(i)>0&&backoff_flag_2(i)==0)
                   r_2(i)=r_2(i)+(randi([0,CW],1,1));
                   backoff_flag_2(i)=backoff_flag_2(i)+1;%start backoff%
                end
            end
        end

     end  

        for i=1:N
          if(DIFS_number_2(i)==0&&backoff_flag_2(i)==0)
             delay_2(i)=0;
          end
        end 
        
     kk=0;
     for i=1:1:N/2
         kk=kk+transmission_flag2(i);
     end
     if (kk>0)
     hidden_time_record_2(m_2)=total_time_2+M_2;
     m_2=m_2+1;
     for i=1:1:N/2
         if(transmission_flag2(i)==1)
         transmission_flag2(i)=0;
         end
     end
     end
     total_time_2 = total_time_2+M_2+packet_time;
     count_2 = 0;
     collision_flag_2= 0;    
     end
     
     
    
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
        
        total_time=total_time+M;
        
        if count > 1
            collision_flag = 1; % possible collision occured %
        end
        
        
        if (collision_flag ~= 1) % no collision occured %
            S_1=min(abs(hidden_time_record_1-total_time));
            S_2=min(abs(hidden_time_record_2-total_time));
            if(S_1>packet_time&&S_2>packet_time)
            good_transfer=good_transfer+1;
            elseif(S_1<=packet_time||S_2<=packet_time)
            bad_transfer=bad_transfer+1;
            end
        else
            for i=1:N
                if (M == r(i)) 
                    if(DIFS_number(i)==0)%jumping
                       aaaa=aaaa+1;
                    end
                end
            end
            if(aaaa>0)%the case where collision occured only between jumping packets%
            S_1=min(abs(hidden_time_record_1-total_time));
            S_2=min(abs(hidden_time_record_2-total_time));
            if(S_1>packet_time&&S_2>packet_time)
            good_transfer=good_transfer+1;
            elseif(S_1<=packet_time||S_2<=packet_time)
            bad_transfer=bad_transfer+1;
            end
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
                       q(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total(i)=delay_total(i)+delay(i);
                    end
                 
                 else%collision_flag ~=1,aaaa=0
                       delay(i)=delay(i)+DIFS+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;
                       q(i)=exprnd((1/lambda),1,1);
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
        
        total_time=total_time+M;
        
        if count > 1
            collision_flag = 1; % possible collision occured %
        end
        
        if (collision_flag ~= 1) % no collision occured %
            S_1=min(abs(hidden_time_record_1-total_time));
            S_2=min(abs(hidden_time_record_2-total_time));
            if(S_1>packet_time&&S_2>packet_time)
            good_transfer=good_transfer+1;
            elseif(S_1<=packet_time||S_2<=packet_time)
            bad_transfer=bad_transfer+1;
            end
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
            if(cccc>1&&aaaa>0)%the case where collision occured between jumping and backoff packets%
                bad_transfer=bad_transfer+cccc;
            elseif(cccc==1&&aaaa>0)
              S_1=min(abs(hidden_time_record_1-total_time));
              S_2=min(abs(hidden_time_record_2-total_time));
              if(S_1>packet_time&&S_2>packet_time)
              good_transfer=good_transfer+1;
              elseif(S_1<=packet_time||S_2<=packet_time)
              bad_transfer=bad_transfer+1;
              end
            end
            if(cccc>1&&aaaa==0)%the case where collision occured only between backoff packets
                bad_transfer=bad_transfer+cccc;
            end
            if(cccc==0&&aaaa>1)
               S_1=min(abs(hidden_time_record_1-total_time));
               S_2=min(abs(hidden_time_record_2-total_time));
               if(S_1>packet_time&&S_2>packet_time)
               good_transfer=good_transfer+1;
               elseif(S_1<=packet_time||S_2<=packet_time)
               bad_transfer=bad_transfer+1;
               end
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
                       q(i)=exprnd((1/lambda),1,1);
                       DIFS_number(i)=0;
                       backoff_flag(i)=0;
                       detect(i)=0;
                       delay_total(i)=delay_total(i)+delay(i);
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
                       q(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       delay_total(i)=delay_total(i)+delay(i);
                      end
                   elseif(cccc>0&&aaaa==0)%only backoff
                       delay(i)=delay(i)+M+packet_time;
                       r(i) = round(q(i)/slot_size)-delay(i)+DIFS;%the new counter of vehicle i
                       q(i)=exprnd((1/lambda),1,1);%generate the next counter of vehicle i  
                       DIFS_number(i)=0;
                       backoff_flag(i)=0;
                       detect(i)=0;
                       delay_total(i)=delay_total(i)+delay(i);
                   end       
                else
                     if (DIFS_number(i)==0)%jump
                           delay(i)=delay(i)+DIFS+packet_time;
                           r(i) = round(q(i)/slot_size)-delay(i)+DIFS;
                           q(i)=exprnd((1/lambda),1,1);
                           delay_total(i)=delay_total(i)+delay(i);
                     elseif (backoff_flag(i)>0)
                           delay(i)=delay(i)+M+packet_time;
                           r(i) = round(q(i)/slot_size)-delay(i)+DIFS; 
                           q(i)=exprnd((1/lambda),1,1);
                           DIFS_number(i)=0;
                           backoff_flag(i)=0;
                           detect(i)=0;
                           delay_total(i)=delay_total(i)+delay(i);
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
                           g=delay(i)-round(q(i)/slot_size);
                           if(g<=DIFS)
                              delay(i)=0;
                              DIFS_number(i)=0;
                              delay(i)=delay(i)+g;
                              r(i)=DIFS+DIFS-g;
                              q(i)=exprnd((1/lambda),1,1);
                              backoff_flag(i)=0;
                              DIFS_number(i)=DIFS_number(i)+1;
                              detect(i)=1;
                           elseif(g>DIFS)
                              delay(i)=0;
                              DIFS_number(i)=0;
                              delay(i)=delay(i)+g; 
                              r(i)=DIFS;
                              q(i)=exprnd((1/lambda),1,1);
                              backoff_flag(i)=0;
                              DIFS_number(i)=DIFS_number(i)+1;
                              detect(i)=0;
                           end
                              bad_transfer=bad_transfer+1;
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
          if(DIFS_number(i)==0&&backoff_flag(i)==0)
             delay(i)=0;
          end
        end  
        
     m=m+1;
     total_time = total_time+packet_time;    
     count = 0;
     collision_flag = 0;  
    end
 
    
    for i=1:N
       to_delay=to_delay+delay_total(i);
    end
    
    %PDR
    d_ratio(N/10)=d_ratio(N/10)+good_transfer/(good_transfer+bad_transfer);
    every_delivery_ratio(k)=good_transfer/(good_transfer+bad_transfer);
    %delay
    ave_delay(N/10)=ave_delay(N/10)+to_delay/(good_transfer+bad_transfer);
    k=k+1; 
   end
   ave_delay(N/10)=ave_delay(N/10)*slot_size*1000/repeat_times;%delay in ms
   d_ratio(N/10)=d_ratio(N/10)/repeat_times;
  for i=1:repeat_times
      standard_deviation(N/10)=standard_deviation(N/10)+(every_delivery_ratio(i)-d_ratio(N/10))^2/repeat_times;
  end
  standard_deviation(N/10)=sqrt(standard_deviation(N/10));
  N=N+10;
end
  

A1(1,:)=d_ratio;
A1(2,:)=standard_deviation;