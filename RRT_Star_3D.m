%%RRT* path planning Algorithm
close all;
clear;
clc;
x_max=100;
y_max=100;
z_max=100;
no_of_nodes=2000;
max_dist=5;
%%point parameters
start_node.pos=[0 0 0];
start_node.cost=0;
start_node.parent=0;
goal_node.pos=[60 40 0];
goal_node.cost=0;
      
nodes(1)=start_node;
figure(1)
plot3(start_node.pos(1),start_node.pos(2),start_node.pos(3),'O','Color',[0 0.036 0.33]);
hold on
plot3(goal_node.pos(1),goal_node.pos(2),goal_node.pos(3),'O','Color',[0 0.036 0.33]);
hold on
for i=1:no_of_nodes
    disp(i);
    for j=1:length(nodes)
        if nodes(j).pos==goal_node.pos
            break
        end
    end
    random_node = get_random_points(goal_node,x_max,y_max,z_max);
    [near_node,near_node_value,near_node_index]  = get_initial_near_node(nodes,random_node);
    
    new_node= create_new_node(random_node,near_node,near_node_value,near_node_index,max_dist);   
    plot3(new_node.pos(1),new_node.pos(2),new_node.pos(3),'.','Color',[255 0 0]/255);
    
    hold on
    
    neighbour_node_index=get_neighbour_index(new_node,max_dist,nodes);
    new_node = step_1(new_node,neighbour_node_index,nodes); 
    index=new_node.parent;
%     line([new_node.pos(1),nodes(index).pos(1)],[new_node.pos(2),nodes(index).pos(2)],...
%         'Color',[169 169 169]/255,'LineWidth',0.2);
%     
%     hold on
    nodes=[nodes new_node];
    nodes=rewire_step_2(neighbour_node_index,new_node,nodes);
    
end

D=[];
for j=1:length(nodes)
        tmp=cal_dist(nodes(j).pos,goal_node.pos);
        D=[D tmp];
        plot3(nodes(j).pos(1),nodes(j).pos(2),nodes(j).pos(3),'.','Color',[255 0 0]/255);
        hold on
           
end
    
[val,ind]=min(D);
pt_final=nodes(ind);
goal_node.parent=ind;
goal_node.cost= cal_dist(goal_node.pos,pt_final.pos)+ pt_final.cost;
pt_end= goal_node;
nodes=[nodes goal_node];
way_points=[];
while pt_end.parent~=0
   start=pt_end.parent;
   way_points=[way_points;pt_end.pos];
%    line([pt_end.pos(1),nodes(start).pos(1)],[pt_end.pos(2),nodes(start).pos(2)],...
%        'Color','g','LineWidth',2);
%    drawnow
%    hold on
   pt_end=nodes(start);
end
way_points=[way_points;start_node.pos];
for i=1:length(way_points)
    plot3(way_points(i,1),way_points(i,2),way_points(i,3),'O','Color',[0 0.036 0.33]);
        hold on;
end
%scatter(way_points(:,1),way_points(:,2));
%hold on
% for j=1:length(way_points)
%     way_points(j,3)=0;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function euclidian_distance=cal_dist(p1,p2)
   euclidian_distance= sqrt(sum((p2-p1).^2));
end

function random_node=get_random_points(goal_node,x_max,y_max,z_max)
   if randi(100)>15
     random_node=[rand(1)*x_max,rand(1)*y_max,rand(1)*z_max];
   else
     random_node=goal_node.pos;
   end
end

function [near_node,near_node_value,near_node_index] = get_initial_near_node(nodes,random_node)
    nodes_dist=[];
    for j=1:length(nodes)
        value=cal_dist(nodes(j).pos,random_node);
        nodes_dist=[nodes_dist,value];
    end
    [near_node_value,ind]=min(nodes_dist);
    near_node=nodes(ind);
    near_node_index=ind;
end

function new_node=create_new_node(random_node,near_node,near_node_value,near_node_index,max_dist)
    theta = atan2(random_node(2)-near_node.pos(2),random_node(1)-near_node.pos(1));
    x= sqrt( (random_node(2)-near_node.pos(2))^2 + (random_node(3)-near_node.pos(3))^2);
    psi= atan2(random_node(3)-near_node.pos(3),x);
    new_node.pos= near_node.pos + max_dist.*[cos(theta) sin(theta) sin(psi)];
    new_node.cost= near_node_value + cal_dist(new_node.pos,near_node.pos);
    new_node.parent= near_node_index;
end

function neighbour_node_index=get_neighbour_index(new_node,max_dist,nodes)
      neighbour_node_index=[];
      r=5*max_dist;
      for j=1:length(nodes)
        if cal_dist(nodes(j).pos,new_node.pos)<=r
            neighbour_node_index=[neighbour_node_index j];
        end
      end
end

function new_node=step_1(new_node,neighbour_node_index,nodes)
      list=[];
      parent_index=[];
      if isempty(neighbour_node_index)
          new_node=new_node;
      else
          for j=1:length(neighbour_node_index)
             new_node_cost=nodes(neighbour_node_index(j)).cost +...
                 cal_dist(nodes(neighbour_node_index(j)).pos,new_node.pos);
             list=[list new_node_cost];
             parent_index=[parent_index neighbour_node_index(j)];
          end
          [val,ind]=min(list);
          new_node.cost=val;
          new_node.parent=parent_index(ind);
      end
      
end  

function nodes=rewire_step_2(neighbour_node_index,new_node,nodes)
    for j=1:length(neighbour_node_index)
        rewire_cost= new_node.cost + cal_dist(nodes(neighbour_node_index(j)).pos,new_node.pos);
        if rewire_cost < nodes(neighbour_node_index(j)).cost
            parent_node_ind=nodes(neighbour_node_index(j)).parent;
%             line([nodes(neighbour_node_index(j)).pos(1),nodes(parent_node_ind).pos(1),nodes(parent_node_ind).pos(3)],...
%             [nodes(neighbour_node_index(j)).pos(2),nodes(parent_node_ind).pos(2),nodes(parent_node_ind).pos(3)],...
%                 'Color',[1 1 1],'LineWidth',0.2);
%            
%             hold on
            nodes(neighbour_node_index(j)).parent=length(nodes); 
            nodes(neighbour_node_index(j)).cost=rewire_cost;
%             line([new_node.pos(1),nodes(neighbour_node_index(j)).pos(1)],[new_node.pos(2),nodes(neighbour_node_index(j)).pos(2)],...
%                  'Color',[169 169 169]/255,'LineWidth',0.2);
%            
%             hold on
        end
    end
end

