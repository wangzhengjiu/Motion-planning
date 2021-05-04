 function map = obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y)
%This function returns a map contains random distribution obstacles.
    rand_map = rand(MAX_X,MAX_Y);%为地图中每个格子赋一个概率
    map = [];
    map(1,1) = xStart;
    map(1,2) = yStart;
    k=2;
    obstacle_ratio = 0.25;
    %障碍物生成概率为0.25，如果rand_map中对应的位置（目标点和起点除外）小于0.25，则标记为障碍物
    for i = 1:1:MAX_X
        for j = 1:1:MAX_Y
            if( (rand_map(i,j) < obstacle_ratio) && (i~= xStart || j~=yStart) && (i~= xTarget || j~=yTarget))
                %排除目标点和起点
                map(k,1) = i;
                map(k,2) = j;
                %统计障碍物的数目
                k=k+1;
            end    
        end
    end
    map(k,1) = xTarget;
    map(k,2) = yTarget;
    %返回的map第一行为起点、最后一行为目标点
end

