%% DUBINS PATH - Example 2
%   author:	Giuseppe Sensolini Arra'
%   date:	August 2020
%   brief:	CSC singular path, computation of the switching function

clear all
close all
clc

%% User data

% min turning radius
R = 1/3

% poses
pose_0 = [0,0,-pi/3]
pose_f = [1,1,-pi/6]

%% COMPUTE DUBINS PATH
% Compute the path segment between poses
dub_conn = dubinsConnection('MinTurningRadius',R);
[path_seg, path_costs] = connect(dub_conn, pose_0, pose_f);

% segment length
l1 = path_seg{1}.MotionLengths(1);
l2 = path_seg{1}.MotionLengths(2);
l3 = path_seg{1}.MotionLengths(3);
l = [l1 l2 l3]

% time (supposing constant v=1 m/s)
t1 = path_seg{1}.MotionLengths(1);
t2 = t1 + path_seg{1}.MotionLengths(2);
t3 = t2 + path_seg{1}.MotionLengths(3);
t = [t1 t2 t3]

%% DUBINS PATH PLOT
vec_color = '#0072be';
figure()
set(gcf,'position',[200,500,500,400]);
x0 = 0.55;
y_len = 0.8;
x_len = (y_len/4)*5
axis([x0-x_len,x0+x_len, -0.4,1.2]);
xlabel('x'), ylabel('y ');

% plot circles
v0 = R*[cos(pose_0(3)); sin(pose_0(3))];
c0 = [0 -1; 1 0]*v0;
circle(pose_0(1)+c0(1), pose_0(2)+c0(2), R);

vf = R*[cos(pose_f(3)); sin(pose_f(3))];
cf = [0 1; -1 0]*vf;
circle(pose_f(1)+cf(1), pose_f(2)+cf(2), R);

% plot initial and final pose
quiver(pose_0(1), pose_0(2), cos(pose_0(3)), sin(pose_0(3)), R, ...
    'linewidth',2, 'color', vec_color, 'MaxHeadSize',.5); hold on;
quiver(pose_f(1), pose_f(2), cos(pose_f(3)), sin(pose_f(3)), R, ...
    'linewidth',2, 'color', vec_color, 'MaxHeadSize',.5);

% plot Dubins path
show(path_seg{1}), grid on, legend off;
set(get(gca,'ylabel'),'rotation',0)

% export figure setup
set(findall(gcf,'DataAspectRatio',[1 1 1], 'type','line'),'linewidth', 2);
set(gcf,'Renderer','Painters');

%% SWITCHING FUNCTION PLOT
[lambda3, time_frame, poses] = switching_function(path_seg);
figure()
plot(time_frame, lambda3); grid on;

xlabel('Time'); 
ylabel('$\mathbf{\lambda_3}$  ','Interpreter','latex', 'FontSize',16);
set(get(gca,'ylabel'),'rotation',0);

xline(path_seg{1}.MotionLengths(1),':','Color','#666666','Linewidth', 2);
xline(sum(path_seg{1}.MotionLengths(1:2)),':','Color','#666666','Linewidth', 2);
xline(sum(path_seg{1}.MotionLengths(1:3)),':','Color','#666666','Linewidth', 2);

text(path_seg{1}.MotionLengths(1)-0.12, -0.5, '$t_1$','Interpreter','latex','FontSize',14);
text(sum(path_seg{1}.MotionLengths(1:2))-0.12, -0.5, '$t_2$','Interpreter','latex','FontSize',14);
text(sum(path_seg{1}.MotionLengths(1:3))-0.12, -0.5, '$t_f$','Interpreter','latex','FontSize',14);

set(gcf,'position',[700,500,500,400]);
set(findall(gcf, 'type','line'),'linewidth', 2);
set(gcf,'Renderer','Painters');
box off;

%% PHASE PORTRAIT
% figure
% lambda3_dot = [diff(lambda3(1:2));diff(lambda3)];
% plot(lambda3,lambda3_dot); grid on;
% xlabel('$\mathbf{\lambda_3}$','Interpreter','latex', 'FontSize',16); 
% ylabel('$\mathbf{\dot{\lambda}_3}$ ','Interpreter','latex', 'FontSize',16);
% set(get(gca,'ylabel'),'rotation',0)
% set(findall(gcf, 'type','line'),'linewidth', 2);
% set(gcf,'Renderer','Painters');


%% Compute switching function
function [lambda3,time_frame,poses] = switching_function(path_seg)
    num_samples = 1000;
    tmp = linspace(0, path_seg{1}.Length, num_samples).';
    poses = interpolate(path_seg{1},tmp);
    N = size(poses,1);
    time_frame = linspace(0, path_seg{1}.Length, N).';

    % get theta1 (theta of the sstraight line segment)   
    X = contains(path_seg{1}.MotionTypes,'S');
    for i = 1:size(path_seg{1}.MotionTypes,2)
        if X(i)==1
            d1 = sum(path_seg{1}.MotionLengths(1:i-1));
            d2 = sum(path_seg{1}.MotionLengths(1:i));
            d = (d1+d2)/2;  % middle point of the straight line segment
            key_sample = round(num_samples*(d/sum(path_seg{1}.MotionLengths)));
            theta1 = poses(key_sample,3)
        end
    end
    
    % get control array
    d0 = 0;
    d1 = sum(path_seg{1}.MotionLengths(1));
    d2 = sum(path_seg{1}.MotionLengths(1:2));
    df = sum(path_seg{1}.MotionLengths(1:3));
    
    k = [0,0,0,0];
    k(1) = 1;
    k(2) = round(num_samples*(d1/sum(path_seg{1}.MotionLengths)));
    k(3) = round(num_samples*(d2/sum(path_seg{1}.MotionLengths)));
    k(4) = N;
    
    a = ones(N,1);
    for i=1:3
        if string(path_seg{1}.MotionTypes(i))=='L'
            a(k(i):k(i+1)) = 1/(path_seg{1}.MinTurningRadius);
        end
        if string(path_seg{1}.MotionTypes(i))=='R'
            a(k(i):k(i+1)) = -(1/(path_seg{1}.MinTurningRadius));
        end
    end
 
    % compute switching function
    lambda3 = -(cos(poses(:,3)-theta1+pi)+1)./a;
end


%% Plot cirle ulitity function
function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, ':', 'Color', '#999999','linewidth', 1.5);
end







%