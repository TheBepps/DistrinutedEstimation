function ob_temp = obstBuild(map,num,detect_R)
    ob_temp=zeros(num,2);
    for i=1:num     %for each obstacle create a random (x,y) position
        flag=true;
        while flag  %check anchor-obstacle overlapping
            flag=false;
            ob_temp(i,1)=randi([1,map.dim]);            
            for k=1:(size(map.anchors_x,2))
                if abs(ob_temp(i,1)-map.anchors_x(k))<detect_R  %inconsistent ancor and avoidance point
                    flag=true;
                end
            end
        end
        % after choosing X valid position
        ob_temp(i,2)=min(map.profile(ob_temp(i,1)*10)+randi([2,30]), 0);  % obstacle Y position (must be under the zero level "water")
    end
end