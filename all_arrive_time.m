function [ t_arrive_current ] = all_arrive_time( index,t_arrive_new,num_phase,CLOCK )
t_arrive_all = ones(1,num_phase)*20000;
temp = 1;
for j = index
    t_arrive_all(j) = t_arrive_new(temp)+CLOCK;
    temp = temp + 1;    
end
t_arrive_current = t_arrive_all;
%t_arrive_current = CLOCK + t_arrive_all;
end