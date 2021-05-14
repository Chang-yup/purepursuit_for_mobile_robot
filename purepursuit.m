clc
clear all
close all

%% Create Path
dx = 0.1;
dt = 0.1;
dt_length=100000;
Path_x_global=[0:dx:100];
Path_y_global=10.*sin(0.1.*Path_x_global);
length = length(Path_x_global);

global Path_global
Path_global = [Path_x_global; Path_y_global; zeros(1,length)];

%% Vehicle Modeling

%Conditions
L = 1;
v = 0.1;
first_run = 1;
% q = [x_r
%      y_r
%      tehta_r];
 
 
for k=1:dt_length
%Initial Condition
    if(first_run == 1)
        q = [0
             0
             0];
        first_run = 0;
    end
    
    
%Transform Matrix Robot to Global    
    TF_RtoG = [cos(q(3)) -sin(q(3)) 0
               sin(q(3))  cos(q(3)) 0
               0             0      1];
    TF_GtoR = [ cos(q(3))  sin(q(3)) 0
               -sin(q(3))  cos(q(3)) 0
                0             0      1];
           
%path grid from robot         
    Paht_x_gp = Path_global(1,:) - q(1);
    Paht_y_gp = Path_global(2,:) - q(2);
    Path_robot = TF_GtoR*([Paht_x_gp
                           Paht_y_gp
                           zeros(1,length)]);
                       
%Get point near from L
    Path_robot =[Path_robot(1,:)
                 Path_robot(2,:)
                 1:1:length]; %% 로봇 프레임 경로 좌표
    distance_robot_path = (Path_robot(1,:).^2 + Path_robot(2,:).^2).^0.5; %% 경로데이터와 로봇 사이 거리
    
    %L 근방 구간 경로 데이터 추출과정
    distance_check = ((L-2*dx < distance_robot_path & distance_robot_path < L+2*dx));
    x_1= Path_robot(1,:) .* distance_check;
    y_1= Path_robot(2,:) .* distance_check;
    index_1 = Path_robot(3,:).*distance_check;
    [~,num2] = min(distance_robot_path); %%로봇에서 가장 가까운 점의 인덱스
    index_check =  (index_1 - index_1(num2)) > 0; %%인덱스가 높은쪽만 살림
    [~,num2] = max(distance_robot_path.*distance_check.*index_check); 
    
    %제어기
    R_track = L^2/(2*Path_robot(2,num2));
    w=v/R_track;

    %시뮬레이션
    u = [v
         w];
    
    A = [cos(q(3)) 0
         sin(q(3)) 0
         0         1];
    
    q = q + A*u*dt;

    result(:, k) = [q(1) q(2) q(3)];
    
end

%Plot
plot(Path_x_global,Path_y_global,'g') %%Path: Green line
hold on
PLOT_DOT=plot(result(1,1),result(2,2),'-.r*'); %%차량: 빨간색 실선
for i=1:dt_length
    set(PLOT_DOT,'XData',result(1,i));
    set(PLOT_DOT,'YData',result(2,i));
    drawnow
end