function [ t_acc_new, t_arrive_current, V_opt_new, replanning ] = V_profile( v0, a_max, X, V_opt, V_last, t_arrive_last, CLOCK )

num_phase = 8;
replanning = 0;
index = find(X);
X = X(find(X));
num = length(X);
C = 16;
t_acc = zeros(1,num);
t_arrive_new = zeros(1,num);


[~, index_sorted] = sort(V_opt./v0);
V_sorted = V_opt(index_sorted);
t_fastest = ( V_sorted(num) - v0(num) ) / a_max;
%%
for i = 1 : num-1
    eval(['t_',num2str(i),' = t_fastest * V_sorted(i) * (V_sorted(num) - v0(num)) / (V_sorted(num)*(V_sorted(i) - v0(i)));'])
    eval(['t_acc(index_sorted(i)) = t_',num2str(i),';'])
end
t_acc(index_sorted(num)) = t_fastest;
t_acc_new = t_acc;

for i = 1:num
    t_arrive_new(i) = t_acc_new(i) + (X(i)-C-(v0(i)+V_opt(i))*t_acc_new(i)/2) / V_opt(i);

end

t_arrive_current = all_arrive_time( index, t_arrive_new, num_phase, CLOCK );


if inter_loop_check(t_arrive_current, t_arrive_last, all_optimal_velocity( index, V_opt, num_phase), V_last, index) == 1
   replanning = 1;
end

%% 

V_opt_new = all_optimal_velocity(index, V_opt, num_phase);


end



