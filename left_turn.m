function [ ans ] = left_turn( theta, index)

lanewid = 4;
lanenum = 3;
radius = 4;

R = radius + (lanenum+0.5) * lanewid;


switch index
    case 1
        if theta >= 0
            x = 16 + theta * R;
            y = 0.5 * lanewid;
            a = -pi;
        elseif theta < 0 && theta >= -pi/2
            x = 16 + R * sin(theta);
            y = 0.5 * lanewid - (R - R*cos(theta)); 
            a = -pi - theta;
        else
            x = - 0.5 * lanewid;
            y = -16 + (theta + 0.5*pi)*R;
            a = -0.5*pi;
        end
            
    case 3
        if theta >= 0
            x = -0.5 * lanewid;
            y = 16 + theta * R;
            a = -0.5*pi;
        elseif theta < 0 && theta >= -pi/2
            x = -0.5 * lanewid + (R - R*cos(theta));
            y = 16 + R * sin(theta);
            a = -0.5*pi - theta;
        else
            x = 16 - (theta + 0.5*pi)*R;
            y = -0.5 * lanewid;
            a = 0;
        end
        
    case 5
        if theta >= 0
            x = -16 - theta * R;
            y = -0.5 * lanewid;
            a = 0;
        elseif theta < 0 && theta >= -pi/2
            x = -16 - R * sin(theta);
            y = -0.5 * lanewid + (R - R*cos(theta)); 
            a = 0 - theta;
        else
            x = 0.5 * lanewid;
            y = 16 - (theta + 0.5*pi)*R;
            a = 0.5*pi;
        end
        
    case 7
        if theta >= 0
            x = 0.5 * lanewid;
            y = -16 - theta * R;
            a = 0.5*pi;
        elseif theta < 0 && theta >= -pi/2
            x = 0.5 * lanewid - (R - R*cos(theta));
            y = -16 - R * sin(theta);
            a = 0.5*pi - theta;
        else
            x = -16 + (theta + 0.5*pi)*R;
            y = 0.5 * lanewid;
            a = pi;
        end
        
    otherwise
        
end
ans = [x,y,a];        
    
%delete(shape)
end
