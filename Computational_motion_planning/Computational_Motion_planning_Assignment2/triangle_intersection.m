function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = false;
centroid = [(P1(1,1) + P1(2,1) + P1(3,1))/3 , (P1(1,2) + P1(2,2) + P1(3,2))/3 ; (P2(1,1) + P2(2,1) + P2(3,1))/3 , (P2(1,2) + P2(2,2) + P2(3,2))/3 ];
syms e1(x,y);
syms e2(x,y);

eq1(x,y) = [(y - P1(1,2))*(P1(2,1) - P1(1,1)) - (x-P1(1,1))*(P1(2,2)-P1(1,2));...
    (y - P1(1,2))*(P1(3,1) - P1(1,1)) - (x-P1(1,1))*(P1(3,2)-P1(1,2));...
    (y - P1(2,2))*(P1(3,1) - P1(2,1)) - (x-P1(2,1))*(P1(3,2)-P1(2,2))];

eq2(x,y) = [(y - P2(1,2))*(P2(2,1) - P2(1,1)) - (x-P2(1,1))*(P2(2,2)-P2(1,2));...
    (y - P2(1,2))*(P1(3,1) - P2(1,1)) - (x-P2(1,1))*(P2(3,2)-P2(1,2));...
    (y - P2(2,2))*(P2(3,1) - P2(2,1)) - (x-P2(2,1))*(P2(3,2)-P2(2,2))];

cx = centroid(1,1);
cy = centroid(1,2);

val1 = eq1(cx,cy);

cx = centroid(2,1);
cy = centroid(2,2);

val2 = eq2(cx,cy);

syms dist(x1,y1,x2,y2);
dist(x1,x2,y1,y2) = sqrt((x2-x1)^2 + (y2-y1)^2);


for i = 1:3
    px = P1(i,1);
    py = P1(i,2);
    val = eq2(px,py);
    if((sign(val(1,1)) == sign(val2(1,1)) && (sign(val(2,1)) == sign(val2(2,1))&&sign(val(3,1)) == sign(val2(3,1)))))
        %display("here1");
        flag= true; 
        break;
    end
    if(val(1,1)==0)
       if(dist(P2(1,1),P2(2,1),P2(1,2),P2(2,2)) == (dist(P2(1,1),px,P2(1,2),py) + dist(px,P2(2,1),py,P2(2,2)) ))
           flag=true;
       end
       break;
    end
    if(val(2,1)==0 )
       if(dist(P2(1,1),P2(3,1),P2(1,2),P2(3,2)) == (dist(P2(1,1),px,P2(1,2),py) + dist(px,P2(3,1),py,P2(3,2)) ))
           flag=true;
       end
       break;
    end
    if(val(3,1)==0)
       if(dist(P2(2,1),P2(3,1),P2(2,2),P2(3,2)) == (dist(P2(2,1),px,P2(2,2),py) + dist(px,P2(3,1),py,P2(3,2)) ))
           flag=true;
       end
       break;
    end
end
if(flag == false)
    for i = 1:3
        px = P2(i,1);
        py = P2(i,2);
        val = eq1(px,py);
        if((sign(val(1,1)) == sign(val1(1,1)) && (sign(val(2,1)) == sign(val1(2,1))&&sign(val(3,1)) == sign(val1(3,1)))))
            flag= true;  
            break;
        end
        if(val(1,1)==0)
           if(dist(P1(1,1),P1(2,1),P1(1,2),P1(2,2)) == (dist(P1(1,1),px,P1(1,2),py) + dist(px,P1(2,1),py,P1(2,2)) ))
               flag=true;
           end
           break;
        end
        if(val(2,1)==0 )
           if(dist(P1(1,1),P1(3,1),P1(1,2),P1(3,2)) == (dist(P1(1,1),px,P1(1,2),py) + dist(px,P1(3,1),py,P1(3,2)) ))
               flag=true;
           end
           break;
        end
        if(val(3,1)==0)
            %display("here3");
            %dist(P1(2,1),P1(3,1),P1(2,2),P1(3,2))
            %dist(P1(2,1),px,P1(2,2),py)
            %dist(px,P1(3,1),py,P1(3,2))
           if(dist(P1(2,1),P1(3,1),P1(2,2),P1(3,2)) == (dist(P1(2,1),px,P1(2,2),py) + dist(px,P1(3,1),py,P1(3,2)) ))
               flag=true;
           end
           break;
        end
    end
end

% *******************************************************************
end