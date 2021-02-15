function [v_safe] = V_safe(b,tau,v_l,g)

v_safe = -b*tau+sqrt((b*tau)^2+v_l^2+2*b*g);

if g < 0
    %fprintf(2,'--------------negative distance!----------\n!')
    %return;
    v_safe = 0;
end

if ~isreal(v_safe)
    v_safe = 0;
    %fprintf(2,'--------------negative distance!----------\n!')
    %return;
end

end

