function [ V_all ] = all_optimal_velocity( index, V_opt, num_phase)
V_all = ones(1,num_phase)*100;
temp = 1;
for j = index
    V_all(j) = V_opt(temp);
    temp = temp + 1;    
end

end