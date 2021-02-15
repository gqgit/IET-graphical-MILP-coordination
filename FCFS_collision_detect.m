function [junc_reserved,confirm_flag,T_acc] = FCFS_collision_detect(junc_occupied_old,reservation_time,CLOCK,t_step,L0,V0,phase,Vmax1,Vmax2,amax,step_ratio)

confirm_flag = 1;
sparse_num = 6;

if mod(phase,2)==0
    Vmax = Vmax2;
else
    Vmax = Vmax1;
end

A = amax;

V_des = Vmax;
T = (V_des-V0)/amax;

%% time calibration
passed_steps = (CLOCK - reservation_time) / t_step;
[length,width,pred_steps] = size(junc_occupied_old);
junc_occupied = ndSparse.build([length,width,pred_steps],0);

index = find(junc_occupied_old);
index_offset = index(index > passed_steps*length*width);
index_offset = round(index_offset - passed_steps*length*width);
if ~isempty(index_offset)
    junc_occupied(index_offset) = 1;
end

junc_reserved = junc_occupied;


%%

for T_acc = T
    confirm_flag = 1;
    CLOCK_dynamic = CLOCK;
    L = L0;
    V = V0;
    junc_occupied = junc_reserved;
    for i = 1:pred_steps
        for k = 1:step_ratio%
            if CLOCK_dynamic < T_acc+CLOCK
                L = L - (V*t_step/step_ratio + 0.5*A*(t_step/step_ratio)^2);%
                V = V + A*t_step/step_ratio;
                CLOCK_dynamic = CLOCK_dynamic + t_step/step_ratio;
            else
                L = L - V*t_step/step_ratio;%
                CLOCK_dynamic = CLOCK_dynamic + t_step/step_ratio;
            end
        end
        %CLOCK_dynamic = CLOCK_dynamic + t_step;
        
        switch phase
             case 2
                 x = L;             y = 6;             a = 270;
             case 4
                 x =-6;             y = L;             a = 180;
             case 6
                 x =-L;             y =-6;             a = 90;
             case 8
                 x = 6;             y =-L;             a = 0;
             case 1
                 res = left_turn((L-16.)/18.,1);
                 x = res(1);             y = res(2);             a = 90.-res(3)/pi*180;
             case 3
                 res = left_turn((L-16.)/18.,3);
                 x = res(1);             y = res(2);             a = 90.-res(3)/pi*180;
             case 5
                 res = left_turn((L-16.)/18.,5);
                 x = res(1);             y = res(2);             a = 90.-res(3)/pi*180;
             case 7
                 res = left_turn((L-16.)/18.,7);
                 x = res(1);             y = res(2);             a = 90.-res(3)/pi*180;
             otherwise
                 disp('wrong case in switch!!!');
        end
        
        if abs(x) > 20 && abs(y) > 20%
            continue
        end
        %%
        [x_all,y_all]=four_points(x,y,a,5,2);
        x12 = linspace(x_all(1),x_all(2),sparse_num+1);
        x23 = linspace(x_all(2),x_all(3),sparse_num+1);
        x34 = linspace(x_all(3),x_all(4),sparse_num+1);
        x41 = linspace(x_all(4),x_all(1),sparse_num+1);
        y12 = linspace(y_all(1),y_all(2),sparse_num+1);
        y23 = linspace(y_all(2),y_all(3),sparse_num+1);
        y34 = linspace(y_all(3),y_all(4),sparse_num+1);
        y41 = linspace(y_all(4),y_all(1),sparse_num+1);
        x_profile = round([x12(1:sparse_num),   x23(1:sparse_num),  x34(1:sparse_num),  x41(1:sparse_num)]) + 16;
        y_profile = round([y12(1:sparse_num),   y23(1:sparse_num),  y34(1:sparse_num),  y41(1:sparse_num)]) + 16;

        for kk = 1:sparse_num*4
            if x_profile(kk) <= length && x_profile(kk) >= 1 && y_profile(kk) <= width && y_profile(kk) >= 1
                index_temp = length*width*(i-1) + (width-y_profile(kk))*length + x_profile(kk);
                if ismember(index_temp,index_offset)
                    confirm_flag = 0;
                    break
                else
                    junc_occupied(index_temp) = 1;
                end
            end
        end
        if confirm_flag == 0
            break
        end
    end
    if confirm_flag == 1
        break
    end

end



if confirm_flag == 1
    junc_reserved = junc_occupied;
end
    


end



