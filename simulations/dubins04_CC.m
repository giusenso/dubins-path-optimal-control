%% DUBINS PATH - Example 4
%	author:	Giuseppe Sensolini Arra'
%   date:	August 2020
%   brief:	CC nonsingular abnormal optimal path, 
%           computation of the switching function

clear all
close all
clc

%% User data

% min turning radius
R = 1

% poses
pose_0 = [0, 0, 1.5*pi]
pose_f = [4, 0, 1.5*pi]

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
xlabel('x'), ylabel('y');
set(gcf,'position',[200,500,500,400]);
y0 = 0;
x_len = 3;
y_len = (x_len/5)*4
axis([-1,5, y0-y_len,y0+y_len]);

% plot circles
circle(1,0,R);
circle(3,0,R);

% plot Dubins path
show(path_seg{1}), grid on, legend off;

% plot initial and final pose
quiver(pose_0(1), pose_0(2), cos(pose_0(3)), sin(pose_0(3)), R, ...
    'linewidth',2, 'color', vec_color, 'MaxHeadSize',.5); hold on;
quiver(pose_f(1), pose_f(2), cos(pose_f(3)), sin(pose_f(3)), R, ...
    'linewidth',2, 'color', vec_color, 'MaxHeadSize',.5);

% export figure setup
set(findall(gcf, 'type','line'),'linewidth', 2);
set(gcf,'Renderer','Painters');

%% SWITCHING FUNCTION PLOT
[lambda3, time_frame, poses] = switching_function_CC(path_seg);
figure()
plot(time_frame, lambda3); grid on;

xlabel('Time'); 
ylabel('$\mathbf{\lambda_3}$  ','Interpreter','latex', 'FontSize',16);
set(get(gca,'ylabel'),'rotation',0);

xline(sum(path_seg{1}.MotionLengths(1:2)),':','Color','#666666','Linewidth', 2);
xline(sum(path_seg{1}.MotionLengths(1:3)),':','Color','#666666','Linewidth', 2);

text(sum(path_seg{1}.MotionLengths(1:2))-0.4, -3.5, '$t_1$','Interpreter','latex','FontSize',14);
text(sum(path_seg{1}.MotionLengths(1:3))-0.4, -3.5, '$t_f$','Interpreter','latex','FontSize',14);

yaxis([-5,5]);
set(gcf,'position',[700,500,500,400]);
set(findall(gcf, 'type','line'),'linewidth', 2);
set(gcf,'Renderer','Painters');
box off;

%% Compute switching function
function [lambda3,time_frame,poses] = switching_function_CC(path_seg)
    num_samples = 1000;
    tmp = linspace(0, path_seg{1}.Length, num_samples).';
    poses = interpolate(path_seg{1},tmp);
    N = size(poses,1);  % number of poses
    time_frame = linspace(0, path_seg{1}.Length, N).';
    
    % get control array
    d0 = 0;
    d1 = sum(path_seg{1}.MotionLengths(1));
    d2 = sum(path_seg{1}.MotionLengths(1:2));
    df = sum(path_seg{1}.MotionLengths(1:3));
    
    k = [0,0,0,0];
    k(1) = 1;
    if path_seg{1}.MotionLengths(1)~=0       
        k(2) = round(num_samples*(d1/sum(path_seg{1}.MotionLengths)));
    else 
        k(2)=k(1);
    end
    if path_seg{1}.MotionLengths(2)~=0       
        k(3) = round(num_samples*(d2/sum(path_seg{1}.MotionLengths)));
    else 
        k(3)=k(2);
    end
    k(4) = N;

    theta1 = poses(k(3)+1,3)

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
    
    % CC Normal case
    %lambda3 = -2*(cos(poses(:,3)-theta1+(2/3)*pi)+1)./a;
    
    % CC Abnormal case
    lambda3 = -(cos(poses(:,3)-theta1+(1/2)*pi))./a;
end


%% Plot cirle ulitity function
function h = circle(x,y,r)
    hold on
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, ':', 'Color', '#AAAAAA','linewidth', 1.3);
end





%