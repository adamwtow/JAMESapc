close all;
clear all;
%data=importdata('./gt_pose_2015-08-27-15-33-39.txt');
%data_=importdata('./icp_pose_2015-08-27-15-33-39.txt');

data=importdata('./gt_pose_2015-08-27-15-15-02_from_kinfu.txt');
data_=importdata('./kinfu_pose_2015-08-27-15-15-02.txt');


data_gt=data.data;
data_kinfu=data_.data;
gt_time=data_gt(:,1)/1e9;
kinfu_time=data_kinfu(:,1)/1e9;
init_gt_time=gt_time(1);
init_kinfu_time=kinfu_time(1);

gt_time=gt_time-init_gt_time;
kinfu_time=kinfu_time-init_kinfu_time;

gt_xyz=data_gt(:,2:4);
kinfu_xyz=data_kinfu(:,2:4);
gt_q=data_gt(:,5:8);
kinfu_q=data_kinfu(:,5:8);

N=length(data_gt);
gt_euler=[];
kinfu_euler=[];
for i=1:N
    euler_temp=quat2eul([data_gt(i,8),data_gt(i,5:7)]);
    gt_euler=[gt_euler;euler_temp];
    euler_temp=quat2eul([data_kinfu(i,8),data_kinfu(i,5:7)]);
    %rewinding
    for j=1:3
        if(euler_temp(j)>deg2rad(90))
            euler_temp(j)=-euler_temp(j);
        end
    end
    
    kinfu_euler=[kinfu_euler;euler_temp];
end


%=======================
%       Position plot
%=======================

figure;
s=subplot(3,1,1);
plot(gt_time,gt_xyz(:,1),'r-','linewidth',2);
hold on
plot(kinfu_time,kinfu_xyz(:,1),'r-.','linewidth',2);
legend('GT x','Kinfu x');
xlabel('Time(s)');
ylabel('Position x (m)');
grid on;
%axis equal;
title(s,'Ground truth trajectory VS. Kinfu pose estimation');

subplot(3,1,2);
plot(gt_time,gt_xyz(:,2),'color',[0 0.5 0],'linestyle','-','linewidth',2);
hold on
plot(kinfu_time,kinfu_xyz(:,2),'color',[0 0.5 0],'linestyle','-.','linewidth',2);
legend('GT y','Kinfu y');
xlabel('Time(s)');
ylabel('Position y (m)');
%axis equal;
grid on;

subplot(3,1,3);
plot(gt_time,gt_xyz(:,3),'b','linestyle','-','linewidth',2);
hold on
plot(kinfu_time,kinfu_xyz(:,3),'b','linestyle','-.','linewidth',2);
legend('GT z','Kinfu z');
xlabel('Time(s)');
ylabel('Position z (m)');
%axis equal;
grid on;


%=======================
%       Orientation plot
%=======================

figure;
s=subplot(3,1,1);
plot(gt_time,rad2deg(gt_euler(:,1)),'r-','linewidth',2);
hold on
plot(kinfu_time,rad2deg(kinfu_euler(:,1)),'r-.','linewidth',2);
legend('GT r','Kinfu r');
xlabel('Time(s)');
ylabel('Orientation roll (deg)');
grid on;
%axis equal;
title(s,'Ground truth trajectory VS. Kinfu orientation estimation');

subplot(3,1,2);
plot(gt_time,rad2deg(gt_euler(:,2)),'color',[0 0.5 0],'linestyle','-','linewidth',2);
hold on
plot(kinfu_time,rad2deg(kinfu_euler(:,2)),'color',[0 0.5 0],'linestyle','-.','linewidth',2);
legend('GT p','Kinfu p');
xlabel('Time(s)');
ylabel('Orientation pich (deg)');
%axis equal;
grid on;

subplot(3,1,3);
plot(gt_time,rad2deg(gt_euler(:,3)),'b','linestyle','-','linewidth',2);
hold on
plot(kinfu_time,rad2deg(kinfu_euler(:,3)),'b','linestyle','-.','linewidth',2);
legend('GT y','Kinfu y');
xlabel('Time(s)');
ylabel('Orientation yaw (deg)');
%axis equal;
grid on;



std_position=std(gt_xyz-kinfu_xyz);
std_q=std(gt_q-kinfu_q);

std_orientation=rad2deg(quat2eul([std_q(4),std_q(1:3)]));
fprintf('Kinfu standard deviations in position x=%.3f, y=%.3f, z=%.3f (in metre)\n',std_position(1),std_position(2),std_position(3));
fprintf('Kinfu standard deviations in orientation roll=%.3f, pitch=%.3f, yaw=%.3f (in deg)\n',std_orientation(1),std_orientation(2),std_orientation(3));













