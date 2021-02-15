clear

Num_step = 1500;
Num_eachphase = 30;
flux = 3000;%veh/h/lane,note that the actual flux is smaller because depart interval < 1 sec is deleted
t_step = 0.1;%communication interval, also the traci control steplength, also the minimum intervel between two FCFS judgement

%% 按泊松分布到达时刻
%depart = generate_depart(flux,Num_eachphase);%depart time by lanes
load('depart_time.mat')
depart = depart - min(min(depart));
%save depart_time depart

% for i =1:2:7
%     depart(i,:) =  2000;
% end

% ti = 0;
% tj = 0;
% depart = [
%     (1:2:59)+ti;
%     (1:2:59)+tj;
%     (1:2:59)+ti;
%     (1:2:59)+tj;
%     (1:2:59)+ti;
%     (1:2:59)+tj;
%     (1:2:59)+ti;
%     (1:2:59)+tj]/2.;

Control_scope = 150;%control range, meter
Start_scope = 300;%departure position, meter
L = ones(8,Num_eachphase+1)*Start_scope;% + 0.5*Start_scope;


%% initialization
% Control_scope = 300;%control range, meter
% Start_scope = 400;%departure position, meter

step_ratio = t_step/0.01;
subscribe_range = 16;
L_edge = 700;
v0 = 10; 
a_max = 3;
decel = 5;
tau = 0.1;
length_car = 4;
width_car = 1.6;
minGap = 3;
Vmin = 10;
Vmax1 = 20;
Vmax2 = 20;

min_distance = ones(1,Num_step)*subscribe_range*2;
V = v0 * ones(8,Num_eachphase+1); 
A = zeros(8,Num_eachphase+1);
T = -1 * ones(8,Num_eachphase+1);
T_pass = zeros(8,Num_eachphase);
arrive = zeros(8,Num_eachphase);
Control_flag = ones(8,Num_eachphase+1);
control_ID = [0 0 0 0 0 0 0 0];
primary_len = [0 0 0 0 0 0 0 0];
subset_num = zeros(1,100); num_of_coordination = 1;
veh_num = ones(8) * Num_eachphase;
t_arrive_last = zeros(1,8);
t_arrive_total = zeros(1,8);
V_opt_last = ones(1,8)*100;
V_opt_total = ones(1,8)*100;
name_dict = cell(8,Num_eachphase);
CLOCK = 0;

check_once = ones(8,Num_eachphase+1);
v_current = zeros(1,8);
Stop_distance = 0;
run_time = zeros(1,Num_step); run_count = 1;
%for plot
t_s_1 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_1 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_2 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_2 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_3 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_3 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_4 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_4 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_5 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_5 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_6 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_6 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_7 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_7 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase

t_s_8 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase
t_v_8 = zeros(1+Num_eachphase,Num_step);%1 row: time; 2-Num_eahphase+1 row: displacement of according phase



%% traci connection
import traci.constants
traci.start(strcat('sumo-gui -c ./data/cross_unsignalized.sumocfg --step-length ',32,num2str(t_step),32,'--collision.action',32,'none'));
center = traci.junction.getPosition('0');
centerx = center(1);
centery = center(2);
traci.junction.subscribeContext('0',  traci.constants.CMD_GET_VEHICLE_VARIABLE,  subscribe_range, {traci.constants.VAR_POSITION,traci.constants.VAR_ANGLE});

for i = 1:8
    for j = 1:Num_eachphase
        switch i
            case 1
                name_dict{i,j} = {strcat('A', num2str(j)),'es','2',num2str(L_edge-L(i,j))};
            case 2
                name_dict{i,j} = {strcat('B', num2str(j)),'ew','1',num2str(L_edge-L(i,j))};
            case 3
                name_dict{i,j} = {strcat('C', num2str(j)),'ne','2',num2str(L_edge-L(i,j))};
            case 4
                name_dict{i,j} = {strcat('D', num2str(j)),'ns','1',num2str(L_edge-L(i,j))};
            case 5
                name_dict{i,j} = {strcat('E', num2str(j)),'wn','2',num2str(L_edge-L(i,j))};
            case 6
                name_dict{i,j} = {strcat('F', num2str(j)),'we','1',num2str(L_edge-L(i,j))};
            case 7
                name_dict{i,j} = {strcat('G', num2str(j)),'sw','2',num2str(L_edge-L(i,j))};
            case 8
                name_dict{i,j} = {strcat('H', num2str(j)),'sn','1',num2str(L_edge-L(i,j))};
        end
    end
end

for i = 1:8
    for j = 1:Num_eachphase
        eval(['traci.vehicle.add(name_dict{',num2str(i),',',num2str(j),'}{1},name_dict{',num2str(i),',',num2str(j),'}{2},''determined'',num2str(',num2str(depart(i,j)),'),name_dict{',num2str(i),',',num2str(j),'}{3},name_dict{',num2str(i),',',num2str(j),'}{4},num2str(',num2str(v0),'));'])
        eval(['traci.vehicle.setLaneChangeMode(name_dict{',num2str(i),',',num2str(j),'}{1},',num2str(0),');'])
    end
end

%% Num_step steps


for i = 1 : Num_step
    
     CLOCK = i * t_step;
     if L(1,veh_num(1))<-20 && L(2,veh_num(2))<-20 && L(3,veh_num(3))<-20 && L(4,veh_num(4))<-20 && ...
         L(5,veh_num(5))<-20 && L(6,veh_num(6))<-20 && L(7,veh_num(7))<-20 && L(8,veh_num(8))<-20
         break
     end


    %% find uncontroled ID of this round
     for j = 1:8
         if size(find(Control_flag(j,:),1),2) == 0
             control_ID(j) = veh_num(j)+1;
             primary_len(j) = 3 * Control_scope;
         else
             control_ID(j) = find(Control_flag(j,:),1);
             primary_len(j) = L(j,control_ID(j));
         end
     end
     if min(primary_len) < Control_scope
        tic
         for j = 1:8
            Control_flag(j,control_ID(j)) = 2;
        end
    %% ad_hoc network
         result = milp(primary_len,Vmin,Vmax1,Vmax2);
         formal_net = ad_hoc(primary_len,result,Vmax1,Vmax2);

         X = [primary_len] .* formal_net;
         for j = 1:8
             if X(j)==3 * Control_scope
                 X(j)=0;
             end
         end
         subset_num(num_of_coordination) = nnz(X);
         num_of_coordination = num_of_coordination+1;
    %% V_profile planning
         for j = 1:8
            v_current(j) = V(j,control_ID(j));
         end
         [t_acc_temp, t_arrive_last_temp, V_opt_last_temp, replanning] = V_profile( v_current(find(X)), a_max, X, result(find(X)),V_opt_total, t_arrive_total, CLOCK);
         
         if replanning == 0%if replanning=1, run as if not optimized, at the original velocity
             t_acc = t_acc_temp;
             t_arrive_last = t_arrive_last_temp;
             V_opt_last = V_opt_last_temp;
             for k = 1:8
                 if t_arrive_last(k) < 20000
                     t_arrive_total(k) = t_arrive_last(k);
                 end
                 if V_opt_last(k) < 100
                     V_opt_total(k) = V_opt_last(k);
                 end
             end

             k = 0;
             for j = find(X)
                 k = k + 1;
                 A(j,control_ID(j)) = (V_opt_last(j) - v_current(j)) / t_acc(k);
                 T(j,control_ID(j)) = CLOCK + t_acc(k);
             end

             for j = find(X)
                Control_flag(j,control_ID(j))=0;
                 
                 if control_ID(j) <= Num_eachphase
                    traci.vehicle.setColor(name_dict{j,control_ID(j)}{1},[0,255,0,500]);
                 end
             end
         end
         
         run_time(i) = toc;

     end


     for j = 1:8
         for k = 1:Num_eachphase
             if  Control_flag(j,k) ~= 0 && check_once(j,k) ~= 0 && L(j,k) < v0*(v0/1)/2+18+Stop_distance
                 T(j,k) = -(0-V(j,k))/1+CLOCK;
                 A(j,k) = -1;
                 check_once(j,k) = 0;
                 traci.vehicle.setColor(name_dict{j,k}{1},[255,0,0,500]);
             end
         end
     end
   
    %% driving stage
    
    CLOCK_dynamic = CLOCK*ones(8,Num_eachphase);
    for j = 1:8
        for k = 1 : Num_eachphase

            if CLOCK >= depart(j,k)
                 if k > 1
                     v_safe = V_safe(decel,tau,V(j,k-1),L(j,k)-L(j,k-1)-length_car-minGap);
                     for step_dynamic = 1:step_ratio
                        if CLOCK_dynamic(j,k) < T(j,k)
                            V(j,k) = max(0,min(v_safe,V(j,k) + A(j,k)*t_step/step_ratio));
                            L(j,k) = L(j,k) - (V(j,k)*t_step/step_ratio + 0.5*A(j,k)*(t_step/step_ratio)^2);
                            CLOCK_dynamic(j,k) = CLOCK_dynamic(j,k) + t_step/step_ratio;
                        else
                            V(j,k) = max(0,min(v_safe,V(j,k)));
                            L(j,k) = L(j,k) - V(j,k) * t_step/step_ratio;
                            CLOCK_dynamic(j,k) = CLOCK_dynamic(j,k) + t_step/step_ratio;
                        end
                     end
                 else
                     for step_dynamic = 1:step_ratio
                        if CLOCK_dynamic(j,k) < T(j,k)
                            V(j,k) = V(j,k) + A(j,k)*t_step/step_ratio;
                            L(j,k) = L(j,k) - (V(j,k)*t_step/step_ratio + 0.5*A(j,k)*(t_step/step_ratio)^2);
                            CLOCK_dynamic(j,k) = CLOCK_dynamic(j,k) + t_step/step_ratio;
                        else
                            L(j,k) = L(j,k) - V(j,k)*t_step/step_ratio;
                            CLOCK_dynamic(j,k) = CLOCK_dynamic(j,k) + t_step/step_ratio;
                        end
                     end
                 end 
             end
            
        end
    end

    for j = 1:8
        for k = 1:Num_eachphase
       
            if L(j,k) > -60 && CLOCK >= depart(j,k)
                switch j
                     case 2
                         eval(['traci.vehicle.moveToXY(strcat(''B'',num2str(',num2str(k),')),''2o'',2,centerx+L(',num2str(j),',',num2str(k),'),centery+6,270.,2);'])
                     case 4
                         %strcat('D',num2str(k))
                         eval(['traci.vehicle.moveToXY(strcat(''D'',num2str(',num2str(k),')),''2o'',2,centerx-6,centery+L(',num2str(j),',',num2str(k),'),180.,2);'])
                     case 6
                         eval(['traci.vehicle.moveToXY(strcat(''F'',num2str(',num2str(k),')),''3o'',2,centerx-L(',num2str(j),',',num2str(k),'),centery-6,90.,2);'])
                     case 8
                         eval(['traci.vehicle.moveToXY(strcat(''H'',num2str(',num2str(k),')),''2o'',2,centerx+6,centery-L(',num2str(j),',',num2str(k),'),0.,2);'])
                     case 1
                         eval(['ans = left_turn((L(',num2str(j),',',num2str(k),')-16)/18,1);'])
                         eval(['traci.vehicle.moveToXY(strcat(''A'',num2str(',num2str(k),')),''2o'',2,centerx+ans(1),centery+ans(2),90.-ans(3)/pi*180.,2);'])                
                     case 3
                         eval(['ans = left_turn((L(',num2str(j),',',num2str(k),')-16)/18,3);'])
                         eval(['traci.vehicle.moveToXY(strcat(''C'',num2str(',num2str(k),')),''2o'',2,centerx+ans(1),centery+ans(2),90.-ans(3)/pi*180.,2);'])
                     case 5
                         eval(['ans = left_turn((L(',num2str(j),',',num2str(k),')-16)/18,5);'])
                         eval(['traci.vehicle.moveToXY(strcat(''E'',num2str(',num2str(k),')),''2o'',2,centerx+ans(1),centery+ans(2),90.-ans(3)/pi*180.,2);'])
                     case 7
                         eval(['ans = left_turn((L(',num2str(j),',',num2str(k),')-16)/18,7);'])
                         eval(['traci.vehicle.moveToXY(strcat(''G'',num2str(',num2str(k),')),''2o'',2,centerx+ans(1),centery+ans(2),90.-ans(3)/pi*180.,2);'])
                     otherwise
                         disp('wrong case in switch!!!');
                end
            end

            if arrive(j,k)==0 && L(j,k) < -20
                arrive(j,k)=1;
                T_pass(j,k)=CLOCK-depart(j,k);
            end

        end 
    end
    
    result = traci.junction.getContextSubscriptionResults('0');
    if result == 'None'
        
    else
        min_distance(i) = collision_check(values(result));
    end
    traci.simulation.step();

    t_s_1(1,i)=CLOCK;
    t_v_1(1,i)=CLOCK;
    t_s_2(1,i)=CLOCK;
    t_v_2(1,i)=CLOCK;
    t_s_3(1,i)=CLOCK;
    t_v_3(1,i)=CLOCK;
    t_s_4(1,i)=CLOCK;
    t_v_4(1,i)=CLOCK;
    t_s_5(1,i)=CLOCK;
    t_v_5(1,i)=CLOCK;
    t_s_6(1,i)=CLOCK;
    t_v_6(1,i)=CLOCK;
    t_s_7(1,i)=CLOCK;
    t_v_7(1,i)=CLOCK;
    t_s_8(1,i)=CLOCK;
    t_v_8(1,i)=CLOCK;
    for veh = 1:Num_eachphase
        t_s_1(1+veh,i)=L(1,veh);
        t_v_1(1+veh,i)=V(1,veh);
        t_s_2(1+veh,i)=L(2,veh);
        t_v_2(1+veh,i)=V(2,veh);
        t_s_3(1+veh,i)=L(3,veh);
        t_v_3(1+veh,i)=V(3,veh);
        t_s_4(1+veh,i)=L(4,veh);
        t_v_4(1+veh,i)=V(4,veh);
        t_s_5(1+veh,i)=L(5,veh);
        t_v_5(1+veh,i)=V(5,veh);
        t_s_6(1+veh,i)=L(6,veh);
        t_v_6(1+veh,i)=V(6,veh);
        t_s_7(1+veh,i)=L(7,veh);
        t_v_7(1+veh,i)=V(7,veh);
        t_s_8(1+veh,i)=L(8,veh);
        t_v_8(1+veh,i)=V(8,veh);
    end
end
%% plot
%toc
steps_launched = i-1;
traci.close()
save ts1_milp t_s_1
save ts2_milp t_s_2
save ts3_milp t_s_3
save ts4_milp t_s_4
save ts5_milp t_s_5
save ts6_milp t_s_6
save ts7_milp t_s_7
save ts8_milp t_s_8
save tv1_milp t_v_1
save tv2_milp t_v_2
save tv3_milp t_v_3
save tv4_milp t_v_4
save tv5_milp t_v_5
save tv6_milp t_v_6
save tv7_milp t_v_7
save tv8_milp t_v_8
save min_distance_milp min_distance
save steps_milp steps_launched
save start_range Start_scope
save run_time_milp run_time
[arrived,~] = size(find(arrive));
arrived
average_time = sum(T_pass(find(arrive)))/arrived