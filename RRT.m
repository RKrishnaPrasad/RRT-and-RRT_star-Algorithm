%%RRT path planning Algorithm
close all;
clear;
clc;
x_max=600;
y_max=600;
z_max=600;
no_of_nodes=3000;
max_dist=15;
%%point parameters
pt_start.pos=[10 5];
pt_start.cost=0;
pt_start.parent=0;
pt_goal.pos=[423 503];
pt_goal.cost=0;

nodes(1)=pt_start;
figure(1)
plot(pt_start.pos(1),pt_start.pos(2),'O','Color',[0 0.036 0.33]);
hold on
plot(pt_goal.pos(1),pt_goal.pos(2),'O','Color',[0 0.036 0.33]);
hold on
for i=1:no_of_nodes
    disp(i);
    if randi(100)>10
     random_pt=[rand(1)*x_max,rand(1)*y_max];
    else
     random_pt=pt_goal.pos;
    end
    %plot(random_pt(1),random_pt(2),'o','Color',[0.343 0.066 0.343]);
    %hold on
    
    for j=1:length(nodes)
        if nodes(j).pos==pt_goal.pos
            break
        end
    end
    
    nodes_dist=[];
    for j=1:length(nodes)
        value=cal_dist(nodes(j).pos,random_pt);
        nodes_dist=[nodes_dist value];
    end
    [val,ind]=min(nodes_dist);
    pt_near=nodes(ind);
    
    pt_new.pos= position(random_pt,pt_near.pos,val,max_dist);
    plot(pt_new.pos(1),pt_new.pos(2),'.','Color',[255 0 0]/256);
    hold on
%     if i==10
%         break
%     end
%     line([pt_near.pos(1),pt_new.pos(1)],[pt_near.pos(2),pt_new.pos(2)],...
%         'Color',[169 169 169]/256,'LineWidth',0.2);
%     drawnow
%     hold on
    pt_new.cost = cal_dist(pt_new.pos,pt_near.pos) + pt_near.cost;
    
    point_within_radius=[];
    r=50;
    count=1;
    for i=1:length(nodes)
        if cal_dist(nodes(j).pos,pt_new.pos)<=r
            point_within_radius(count).pos=nodes(j).pos;
            point_within_radius(count).cost= nodes(j).cost;
            count=count+1;
        end
    end
    min_pt=pt_near;
    min_cost=pt_new.cost;
    
    for j=1:length(point_within_radius)
        if point_within_radius(j).cost + cal_dist(point_within_radius(j).pos,pt_near.pos) <= pt_new.cost
            min_pt= point_within_radius(j);
            min_cost= point_within_radius(j).cost;
%             line([pt_near.pos(1),min_pt.pos(1)],[pt_near.pos(2),min_pt.pos(2)]...
%                             ,'Color','g','LineWidth',1);
%             hold on
        end
    end
    for j=1:length(nodes)
        if nodes(j).pos==min_pt.pos
            pt_new.parent=j;
        end
    end
    nodes=[nodes pt_new];
end

D=[];
for j=1:length(nodes)
        tmp=cal_dist(nodes(j).pos,pt_goal.pos);
        D=[D tmp];
end
    
[val,ind]=min(D);
pt_final=nodes(ind);
pt_goal.parent=ind;
pt_goal.cost= cal_dist(pt_goal.pos,pt_final.pos)+ pt_final.cost;
pt_end= pt_goal;
nodes=[nodes pt_goal];
figure(2)
plot(pt_start.pos(1),pt_start.pos(2),'o','Color',[0 0.036 0.33]);
hold on
plot(pt_goal.pos(1),pt_goal.pos(2),'o','Color',[0 0.036 0.33]);
hold on
plot([pt_start.pos(1),pt_goal.pos(1)],[pt_start.pos(2),pt_goal.pos(2)]);
hold on
while pt_end.parent~=0
   start=pt_end.parent;
   line([pt_end.pos(1),nodes(start).pos(1)],[pt_end.pos(2),nodes(start).pos(2)],...
       'Color','g','LineWidth',1);
   hold on
   pt_end=nodes(start);
end
hold off
     
function d=cal_dist(p1,p2)
   d= sqrt(sum((p2-p1).^2));
end

function x=position(ptr,ptn,val,max)
  if val>=max
      x = ptn+ (((ptr-ptn).*max)./cal_dist(ptr,ptn));
  else
      x=ptr;
  end
end

      





