clc;
clf;
clear;
hold on;
max_x = 10; %Maximum number of devision in x axis
max_y = 10; 
node_cost = ones( max_x, max_y); %cost of nodes (goal, obstecle, free space

axis([0 max_x+0.5 0 max_y+0.5]); % axis scaling 
xticks(1:max_x); %dividing x-axis into 1's factor 
yticks(1:max_y); 
xline((1:max_x) + 0.5); %making grids for x-axis
yline((1:max_y) + 0.5);

set(gca,'XTick',(1:20), 'YTick', (1:20));

%Entering Goal coordinate

%Goal input
click = 0;
while (click ~= 1)  %waiting for goal input
    [inrt_cord_x, inrt_cord_y, click] = ginput(1); %getting input from mouse
end
goal_x = round(inrt_cord_x); %converting axis into integer
goal_y = round(inrt_cord_y);
node_cost(goal_x, goal_y) = 0;
plot(goal_x, goal_y, 'go');

%Obstecle input
while (click==1)
    [inrt_cord_x, inrt_cord_y, click] = ginput(1);
    x=round(inrt_cord_x); %coordinate at obstelces
    y=round(inrt_cord_y);
    node_cost(x,y)= 1000; % obstecles
    plot(linspace(x-0.5, x+0.5, 100),y,'k|');
end
clear x y;

%Mobile robot location
click = 0;
while (click ~= 1)
    [inrt_cord_x, inrt_cord_y, click] = ginput(1);
end
rob_pos_x = round(inrt_cord_x); %Robots starting position
rob_pos_y = round(inrt_cord_y);
node_cost(rob_pos_x, rob_pos_y) = 0; %robot position
plot(rob_pos_x, rob_pos_y, 'rs');
clear inrt_cord_x inrt_cord_y;
%close_node
close_node = zeros(max_x*max_y, 2); %for having list of closed node.. Empty Node at starting

%environment maping
hn = hn_func(rob_pos_x, rob_pos_y, goal_x, goal_y);
crnt_node = [1, rob_pos_x rob_pos_y rob_pos_x rob_pos_y hn 0 hn];   %holds current node
open_nodes = -1*ones(max_x* max_y*8, 8); %open status, openx, openy, currentx, currenty, fn, gn,hn
open_nodes(1,:)= crnt_node;
count_opn_nd = 2;   %counter for open node 
status = 1;
pathexist = 1;
counter = 1;
close_counter = 1;  %counter for close node array
help_count = 1;
help_count_x=1;
loop_break = 0;
break_count=0;
while (loop_break == 0)
    break_count = break_count+1;
    Notinclosenode = 1;
    for i=1:-1:-1
        for j=1:-1:-1
            Not_in_close_node =1;
            for k = (1:close_counter-1)
                if (close_node(k,1) == crnt_node(4) && close_node(k,2) == crnt_node(5))
                    Not_in_close_node = 0;
                    Notinclosenode=0;
                    break;
                end
                if (close_node(k,1) == crnt_node(4)+i && close_node(k,2) == crnt_node(5)+j)
                    Not_in_close_node = 0;
                    break;
                end
            end
            if ((i~=0)||(j~=0))              
                if(Not_in_close_node == 1)
                   if ((crnt_node(4)+i)>0 && (crnt_node(5)+j)>0 && (crnt_node(4)+i)<=max_x && (crnt_node(5)+j)<=max_y) 
                      gn_nxt = gn_func(crnt_node(4), crnt_node(5), crnt_node(4)+i, crnt_node(5)+j, crnt_node(7), node_cost);
                      if(gn_nxt <1000)
                         hn = hn_func(crnt_node(4) +i, crnt_node(5)+ j, goal_x, goal_y);
                            open_nodes(count_opn_nd,:) = [1 crnt_node(4)+i crnt_node(5)+j crnt_node(4) crnt_node(5) gn_nxt+hn gn_nxt hn];
                          count_opn_nd = count_opn_nd +1;                        
                      end
                   end
                end
            end
            if((crnt_node(4)+i == goal_x && crnt_node(5)+j == goal_y))
                loop_break = 1;
                break;
            end
        end
        if(loop_break == 1)
            break;
        end
    end
    %buffer = open_nodes(counter:count_opn_nd, :);
    if(Notinclosenode == 1)
        close_node(close_counter,:) = crnt_node(4:5);
        close_counter = close_counter+1;
    end
    counter = counter +1;
    skip = 0;
    while skip == 0
        for z = 1:close_counter-1
            if(open_nodes(counter,2:3) == close_node(z))
                skip=1;
                break;
            end
        end
        if(skip == 1)
            counter = counter+1;
            skip = 0;
            if(counter >= count_opn_nd)
                skip = 3;
            end
        else
            skip = 2;
        end
    end
    if(skip == 2)
        crnt_node = open_nodes(counter,:);
        crnt_node(4:5) = crnt_node(2:3);
    else
        loop_break = 1;
    end
    %open_nodes(counter-1,:) = [-1 -1 -1 -1 -1 -1 -1 -1];
    %if(counter >400)
    %    break;
    %end
end
if(open_nodes(count_opn_nd-1, 2:3) == [goal_x goal_y])
    temp_path_x=0;
    temp_path_y=0;
    buffer = count_opn_nd-1;
    path = [goal_x, goal_y];
    path(2,:) = (open_nodes(buffer, 4:5));
    path_counter =3;
    
    while (1)
        count = 0;
        gn_camp = 10000;
        for k = transpose(open_nodes)
            count = count+1;
            if((open_nodes(buffer,4:5) == transpose(k(2:3))) & (gn_camp > k(7)))
                temp_path_x= k(4);
                temp_path_y= k(5);
                gn_camp = k(7);
                buffer = count;
            end
        end
        path(path_counter,:) = [temp_path_x, temp_path_y];
        path_counter = path_counter+1;
        if(temp_path_x == rob_pos_x && temp_path_y == rob_pos_y)
            break;
        end
    end
else
    msg = msgbox("No path exists to the goal", "DIAT_RPPC Project");
    uiwait(msg,10);
    delete(msg);
end
original_path = path;
%printing path
for k= length(path):-1:1
    plot(path(k,1), path(k,2), 'b*');
    pause(.5);
end


%Dynamic behiviour
click = 0;
obst_add=0;
while click ==0
    [Dyn_x,Dyn_y,click] = ginput(1);
end
Dyn_x = round(Dyn_x);
Dyn_y = round(Dyn_y);
if(node_cost(Dyn_x,Dyn_y) == 1000)
    obst_add=0;
    plot(linspace(Dyn_x-0.5, Dyn_x+0.5, 100),Dyn_y,'w|');
    node_cost(Dyn_x, Dyn_y) = 1;
else
    obst_add=1;
    plot(linspace(Dyn_x-0.5, Dyn_x+0.5, 100),Dyn_y,'k|');
    node_cost(Dyn_x, Dyn_y) = 1000;
end
new_goal_x=path(1,1);
new_goal_y=path(1,2);
new_start_x=path(end,1);
new_start_y=path(end,2);
run =0;
if(obst_add == 0)
    for k = transpose(path)
        if(hn_func(Dyn_x,Dyn_y,new_goal_x, new_goal_y) > hn_func(Dyn_x,Dyn_y,k(1),k(2))) && (hn_func(k(1),k(2),new_goal_x, new_goal_y) < hn_func(new_start_x,new_start_y,k(1),k(2)))
            new_goal_x=k(1);
            new_goal_y=k(2);
        end
        if(hn_func(Dyn_x,Dyn_y,new_start_x, new_start_y) > hn_func(Dyn_x,Dyn_y,k(1),k(2))) && (hn_func(k(1),k(2),new_goal_x, new_goal_y) > hn_func(new_start_x,new_start_y,k(1),k(2)))
            new_start_x=k(1);
            new_start_y=k(2);
        end
    end
    rob_pos_x = new_start_x;
    rob_pos_y=new_start_y;
    goal_x=new_goal_x;
    goal_y=new_goal_y;
    run =1;
end
count = 0;
for k = transpose(path)
    count = count+1;
     if(Dyn_x == k(1) && Dyn_y == k(2))
         run=1;
         break;
     end
end

if(obst_add ==1 && run ==1)
    rob_pos_x =path(count+1,1);
    rob_pos_y=path(count+1,2);
    goal_x=path(count-1,1);
    goal_y=path(count-1,2);
end
if(run == 1)
    close_node = zeros(max_x*max_y, 2);
    hn = hn_func(rob_pos_x, rob_pos_y, goal_x, goal_y);
    crnt_node = [1, rob_pos_x rob_pos_y rob_pos_x rob_pos_y hn 0 hn];
    open_nodes = -1*ones(max_x* max_y*8, 8); %open status, openx, openy, currentx, currenty, fn, gn,hn
    count_opn_nd = 1;
    open_nodes(count_opn_nd,:)= crnt_node;
    count_opn_nd = 2;
    status = 1;
    pathexist = 1;
    counter = 1;
    close_counter = 1;
    help_count = 1;
    help_count_x=1;
    loop_break = 0;
    break_count=0;
    while (loop_break == 0)
        break_count = break_count+1;
        Notinclosenode = 1;
        for i=1:-1:-1
            for j=1:-1:-1
                Not_in_close_node =1;
                for k = (1:close_counter-1)
                    if (close_node(k,1) == crnt_node(4) && close_node(k,2) == crnt_node(5))
                        Not_in_close_node = 0;
                        Notinclosenode=0;
                        break;
                    end
                    if (close_node(k,1) == crnt_node(4)+i && close_node(k,2) == crnt_node(5)+j)
                        Not_in_close_node = 0;
                        break;
                    end
                end
                if ((i~=0)||(j~=0))              
                    if(Not_in_close_node == 1)
                       if ((crnt_node(4)+i)>0 && (crnt_node(5)+j)>0 && (crnt_node(4)+i)<=max_x && (crnt_node(5)+j)<=max_y) 
                          gn_nxt = gn_func(crnt_node(4), crnt_node(5), crnt_node(4)+i, crnt_node(5)+j, crnt_node(7), node_cost);
                          if(gn_nxt <1000)
                             hn = hn_func(crnt_node(4) +i, crnt_node(5)+ j, goal_x, goal_y);
                             open_nodes(count_opn_nd,:) = [1 crnt_node(4)+i crnt_node(5)+j crnt_node(4) crnt_node(5) gn_nxt+hn gn_nxt hn];
                              count_opn_nd = count_opn_nd +1;                        
                          end
                       end
                    end
                end
                if((crnt_node(4)+i == goal_x && crnt_node(5)+j == goal_y))
                    loop_break = 1;
                    break;
                end
            end
            if(loop_break == 1)
                break;
            end
        end
        %buffer = open_nodes(counter:count_opn_nd, :);
        if(Notinclosenode == 1)
            close_node(close_counter,:) = crnt_node(4:5);
            close_counter = close_counter+1;
        end
        counter = counter +1;
        skip = 0;
        while skip == 0
            for z = 1:close_counter-1
                if(open_nodes(counter,2:3) == close_node(z))
                    skip=1;
                    break;
                end
            end
            if(skip == 1)
                counter = counter+1;
                skip = 0;
                if(counter >= count_opn_nd)
                    skip = 3;
                end
            else
                skip = 2;
            end
        end
        if(skip == 2)
            crnt_node = open_nodes(counter,:);
            crnt_node(4:5) = crnt_node(2:3);
        else
            loop_break = 1;
        end
    end
    clear path;
    if(open_nodes(count_opn_nd-1, 2:3) == [goal_x goal_y])
        temp_path_x=0;
        temp_path_y=0;
        buffer = count_opn_nd-1;
        path = [goal_x, goal_y];
        path(2,:) = (open_nodes(buffer, 4:5));
        path_counter =3;
        
        while (1)
            count = 0;
            gn_camp = 10000;
            for k = transpose(open_nodes)
                count = count+1;
                if((open_nodes(buffer,4:5) == transpose(k(2:3))) & (gn_camp > k(7)))
                    temp_path_x= k(4);
                    temp_path_y= k(5);
                    gn_camp = k(7);
                    buffer = count;
                end
            end
            path(path_counter,:) = [temp_path_x, temp_path_y];
            path_counter = path_counter+1;
            if(temp_path_x == rob_pos_x && temp_path_y == rob_pos_y)
                break;
            end
        end
    else
        msg = msgbox("No path exists to the goal", "DIAT_RPPC Project");
        uiwait(msg,10);
        delete(msg);
    end
    for k= length(path):-1:1
        plot(path(k,1), path(k,2), 'ro');
        pause(.5);
    end
end  