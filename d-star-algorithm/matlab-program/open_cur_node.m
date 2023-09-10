function open_list_temp = open_cur_node(rob_x, rob_y, open_list, gn, goal_x, goal_y)
%gn - gn of of the current node
open_list_temp = -1*ones(8, 8);
count = 1;
status = 1;
for i=1:-1:0
    for j=1:-1:0
        if (i~=0)||(j~=0)
            for k = open_list(:, 1:3) %checking if node is already open
                if(rob_x+j == k(2))&& (rob_y+i == k(3) && k(1)==0)
                    gn_nxt = gn_func(rob_x, rob_y, rob_x+j, rob_y+i, gn);
                    if(gn_nxt < k(7))
                        
                    end
                    status = 0;
                    break;
                end
            end
            if(status==1)
                gn_nxt = gn_func(rob_x, rob_y, rob_x+j, rob_x+i, gn);
                hn = hn_func(rob_x+j, rob_y+i, goal_x, goal_y);
                open_list_temp(count,:) = [1 rob_x+j rob_y+i rob_x rob_y gn_nxt+hn gn_nxt hn];
                count= count+1;
            end
        end
    end
end
for i=count:8
    open_list_temp(count,:) = -1*ones(1,6);
    count= count+1;
end