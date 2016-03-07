close all;
clear all;
%data=importdata('./gt_pose_2015-08-27-15-33-39.txt');
%data_=importdata('./icp_pose_2015-08-27-15-33-39.txt');

data=importdata('./gt_pose_2015-08-27-15-15-02.txt');
data_=importdata('./icp_pose_2015-08-27-15-15-02.txt');
data__=importdata('./ndt_pose_2015-08-27-15-15-02.txt');


data_gt=data.data;
data_icp=data_.data;
data_ndt=data__.data;

gt_time=data_gt(:,1)/1e9;
icp_time=data_icp(:,1)/1e9;
ndt_time=data_ndt(:,1)/1e9;

init_gt_time=gt_time(1);
init_icp_time=icp_time(1);
init_ndt_time=ndt_time(1);


gt_time=gt_time-init_gt_time;
icp_time=icp_time-init_icp_time;
ndt_time=ndt_time-init_ndt_time;

gt_xyz=data_gt(:,2:4);
icp_xyz=data_icp(:,2:4);
ndt_xyz=data_ndt(:,2:4);

gt_q=data_gt(:,5:8);
icp_q=data_icp(:,5:8);
ndt_q=data_ndt(:,5:8);


N=length(data_gt);
gt_euler=[];
icp_euler=[];
for i=1:N
    euler_temp=quat2eul(data_gt(i,5:8));
    gt_euler=[gt_euler;euler_temp];
    euler_temp=quat2eul(data_icp(i,5:8));
    icp_euler=[icp_euler;euler_temp];
end

N=length(data_ndt);
ndt_euler=[];
for i=1:N
    euler_temp=quat2eul(data_ndt(i,5:8));
    ndt_euler=[ndt_euler;euler_temp];
end



%=======================
%       Position plot
%=======================

figure;
s=subplot(3,1,1);
plot(gt_time,gt_xyz(:,1),'r-','linewidth',2);
hold on
plot(icp_time,icp_xyz(:,1),'r--','linewidth',2);
hold on
plot(ndt_time,ndt_xyz(:,1),'r:','linewidth',2);
legend('GT x','ICP x','NDT x');
xlabel('Time(s)');
ylabel('Position x (m)');
grid on;
%axis equal;
title(s,'Ground truth trajectory VS. ICP and NDT position');

subplot(3,1,2);
plot(gt_time,gt_xyz(:,2),'color',[0 0.5 0],'linestyle','-','linewidth',2);
hold on
plot(icp_time,icp_xyz(:,2),'color',[0 0.5 0],'linestyle','--','linewidth',2);
hold on
plot(ndt_time,ndt_xyz(:,2),'color',[0 0.5 0],'linestyle',':','linewidth',2);

legend('GT y','ICP y','NDT y');
xlabel('Time(s)');
ylabel('Position y (m)');
%axis equal;
grid on;

subplot(3,1,3);
plot(gt_time,gt_xyz(:,3),'b','linestyle','-','linewidth',2);
hold on
plot(icp_time,icp_xyz(:,3),'b','linestyle','--','linewidth',2);
hold on
plot(ndt_time,ndt_xyz(:,3),'b','linestyle',':','linewidth',2);
legend('GT z','ICP z','NDT z');
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
plot(icp_time,rad2deg(icp_euler(:,1)),'r--','linewidth',2);
hold on
plot(ndt_time,rad2deg(ndt_euler(:,1)),'r:','linewidth',2);

legend('GT r','ICP r','NDT r');
xlabel('Time(s)');
ylabel('Roll (deg)');
grid on;
%axis equal;
title(s,'Ground truth trajectory VS. ICP and NDT orientation');

subplot(3,1,2);
plot(gt_time,rad2deg(gt_euler(:,2)),'color',[0 0.5 0],'linestyle','-','linewidth',2);
hold on
plot(icp_time,rad2deg(icp_euler(:,2)),'color',[0 0.5 0],'linestyle','--','linewidth',2);
hold on;
plot(ndt_time,rad2deg(ndt_euler(:,2)),'color',[0 0.5 0],'linestyle',':','linewidth',2);

legend('GT p','ICP p','NDT p');
xlabel('Time(s)');
ylabel('Pich (deg)');
%axis equal;
grid on;

subplot(3,1,3);
plot(gt_time,rad2deg(gt_euler(:,3)),'b','linestyle','-','linewidth',2);
hold on
plot(icp_time,rad2deg(icp_euler(:,3)),'b','linestyle','--','linewidth',2);
hold on
plot(ndt_time,rad2deg(ndt_euler(:,3)),'b','linestyle',':','linewidth',2);

legend('GT y','ICP y','NDT y');
xlabel('Time(s)');
ylabel('Yaw (deg)');
%axis equal;
grid on;







