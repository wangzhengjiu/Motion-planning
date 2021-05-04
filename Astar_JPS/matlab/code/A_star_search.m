function path= A_star_search(map,MAX_X,MAX_Y)
%%
%map的最后一行为goal的横纵坐标，第一行为起点坐标，其他行为生成障碍物的坐标
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1); %返回行列
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1  
    MAP=2*(ones(MAX_X,MAX_Y));%普通路径为2
    
    %Initialize MAP with location of the target    创建终点
    xval=floor(map(size_map, 1)) + X_offset;%floor向下取整
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;%set goal
    
    %Initialize MAP with location of the obstacle 创建障碍物
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point  创建起点
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list   将障碍物加入到closed表中去
    k=1;%Dummy counter   计数器
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;   %在上面最后xval yval的赋值为起点位置
    yNode=yval;
    OPEN_COUNT=1; %open表计数
    goal_distance=distance(xNode,yNode,xTarget,yTarget);   %计算当前点位置和终点距离
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);  %插入起点
     %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val 节点x值 |Y val 节点y值 |Parent X val |Parent Y val |h(n)启发式信息值 即距离终点距离|g(n)从起点走到当前点点路径代价|f(n)=h(n)+g(n)|
    %--------------------------------------------------------------------------
    OPEN(OPEN_COUNT,1)=0;                   %扩展完毕后，从open表中弹出起点
    CLOSED_COUNT=CLOSED_COUNT+1;            %将起点加入close表
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
while(ismember(1,OPEN(:,1))||OPEN_COUNT==1) %若OPEN表非空则循环   ismember判断1是否是OPEN(:,1)中的量
    
    if(xNode==xTarget && yNode==yTarget)     %如果xNode，yNode是目标节点退出
            break;
    end
        exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y); %expand
        
         for i=1:size(exp_array,1)  %i=1:待扩展点的个数
             for j=1:OPEN_COUNT
                 if(exp_array(i,1)==OPEN(j,2)&&exp_array(i,2)==OPEN(j,3))
                     OPEN(j,end)=min(OPEN(j,end),exp_array(i,end));%若待扩展点的f(n)值小于等于open表中的f(n)值，
                                                                   %则更新open表中的f(n)，h(n),g(n)及其父亲节点
                      if OPEN(j,end)== exp_array(i,end)
                         OPEN(j,4)=xNode;
                         OPEN(j,5)=yNode;
                         OPEN(j,6)=exp_array(i,3);
                         OPEN(j,7)=exp_array(i,4);
                      end
                 end
                  
                
             end  
             OPEN_COUNT=OPEN_COUNT+1;
             OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),...
             xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5)); %为open表新的一行赋值
         end
       
       row_index_of_min=min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);         %find the point with lowest fn
       if(row_index_of_min==-1)  %there is no path
           error("ERROR:OPENLIST is empty.There is no path!!!");
           break
       else
           xNode=OPEN(row_index_of_min,2);
           yNode=OPEN(row_index_of_min,3);
           path_cost=OPEN(row_index_of_min,6);
           OPEN(row_index_of_min,1)=0;     %从 OPENLIST中删除
           
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        CLOSED_COUNT=CLOSED_COUNT+1;
       end
     %
     %finish the while loop
     %
     
    end %End of While Loop
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    
    %回溯
   path = [];%空
   path(1,1)=CLOSED(end,1);%从目标点开始
   path(1,2)=CLOSED(end,2);
   xval=path(1,1);
   yval=path(1,2);
   if(xval==xTarget&&yval==yTarget)
       count=2;
       x_parent_node=OPEN(node_index(OPEN,xval,yval),4);%找到目标点的父节点
       y_parent_node=OPEN(node_index(OPEN,xval,yval),5);
       while(1)
           if(x_parent_node==xStart&&y_parent_node==yStart)
               break
           end
           path(count,1)=x_parent_node;
           path(count,2)=y_parent_node;
           index_parent_node=node_index(OPEN,x_parent_node,y_parent_node);
           x_parent_node=OPEN(index_parent_node,4);
           y_parent_node=OPEN(index_parent_node,5);
           count=count+1;
       end
   else
       path=[];
       error("ERROR:There is no path!!!");
   end
   end
