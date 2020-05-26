clear;
clc;
format long
x_c = linspace(-3,3,50);
y_c = linspace(-3,3,50);
[X_c,Y_c] = meshgrid(x_c,y_c);
Z_c = grav(X_c,Y_c);



duration = 1000;                                                            % seconds
fs = 10;                                                                    % Hz
N = duration*fs;                                                            % number of samples

radius = 1;                                                                 % meters
speed = 2*pi*radius/1000;                                                   % meters per second

initialYaw = 90;                                                            % degrees

initPos = [radius, 0, 0];
initVel = [0, speed, 0];
initOrientation = quaternion([initialYaw,0,0],'eulerd','zyx','frame');

trj_original = kinematicTrajectory('SampleRate',fs, ...                     % 定义原始航迹
    'Velocity',initVel, ...
    'Position',initPos, ...
    'Orientation',initOrientation);
acc_original = zeros(N,3);
acc_original(:,2) = speed^2/radius;
acc_original(:,3) = 0;

ang_original = zeros(N,3);
ang_original(:,3) = speed/radius;

pitchRotation = quaternion([0,0,0],'eulerd','zyx','frame');
ang_original = rotateframe(pitchRotation,ang_original);
acc_original = rotateframe(pitchRotation,acc_original);
[pos_original, ort_orginal, vel_original] = trj_original(acc_original,ang_original);
                                                                            %生成原始航迹数据

trj_INS = kinematicTrajectory('SampleRate',fs, ...
    'Velocity',initVel, ...
    'Position',initPos, ...
    'Orientation',initOrientation);                                         %定义惯导航迹

acc_err = normrnd(0, (1e-1)* speed^2/radius, [N,1]);                        %添加高斯白噪声
acc_INS = zeros(N,3);
acc_INS(:,2) = speed^2/radius + acc_err;
acc_INS(:,3) = 0;

ang_err = normrnd(0, 3*(1e-1)* speed/radius, [N,1]);
ang_INS = zeros(N,3);
ang_INS(:,3) = speed/radius + ang_err;

pitchRotation = quaternion([0,0,0],'eulerd','zyx','frame');
ang_INS = rotateframe(pitchRotation,ang_INS);
acc_INS = rotateframe(pitchRotation,acc_INS);
[pos_INS, ort_INS, vel_INS] = trj_INS(acc_INS,ang_INS);                     %生成INS数据


i = 1; j = 1;
while j < N

    z0(i) = grav(pos_original(j,1),pos_original(j,2));
    xo(i) = pos_original(j,1);
    yo(i) = pos_original(j,2);
    xp(i) = pos_INS(j,1);
    yp(i) = pos_INS(j,2);
    
    con_N = contour(X_c,Y_c,Z_c,[z0(i),z0(i)]);
    con_data = contourdata(con_N);

    if length(con_data) == 1                                              % 等值线是否闭合
        dis_min = (xp(i)-con_data(1).xdata(1)).^2+...
                (yp(i)-con_data(1).ydata(1)).^2;
        k = 1;
        num_min = 1;
        while k <= con_data.numel
                                                                            % 循环计算点到等值线的距离
            dis = (xp(i)-con_data(1).xdata(k)).^2+...
                (yp(i)-con_data(1).ydata(k)).^2;
            if dis < dis_min
                dis_min = dis;                                              % 不断比较取最小，相同取第一次出现的
                num_min = k;
            end
            k=k+1;
        end
        
        xm(i) = con_data(1).xdata(num_min);
        ym(i) = con_data(1).ydata(num_min);
    
    else
        dis_min = (xp(i)-con_data(1).xdata(1)).^2+...
            (yp(i)-con_data(1).ydata(1)).^2;
        k = 1;
        num_min = 1;
        while k <= con_data(1).numel
            dis(k) = (xp(i)-con_data(1).xdata(k)).^2+...                    % 此时假设最多只有2个独立等值线    
                (yp(i)-con_data(1).ydata(k)).^2;
            if dis < dis_min
                dis_min = dis;
                num_min = k;
            end
            k=k+1;
            xm(i) = con_data(1).xdata(num_min);
            ym(i) = con_data(1).ydata(num_min);
        end
        
        while (k > con_data(1).numel) && (k <= con_data(1).numel + con_data(2).numel)
            dis(k) = (xp(i)-con_data(2).xdata(k - con_data(1).numel)).^2+...
                (yp(i)-con_data(2).ydata(k - con_data(1).numel)).^2; 
            if dis < dis_min
                dis_min = dis;
                num_min = k;
                xm(i) = con_data(2).xdata(num_min - con_data(1).numel);
                ym(i) = con_data(2).ydata(num_min - con_data(1).numel);
            end
            k=k+1;
        end
 
    end

    i=i+1;
    j=j+130;
end



figure(1)
plot(pos_original(:,1),pos_original(:,2),'color','r')
hold on

con_M=contour(X_c,Y_c,Z_c,15,'ShowText','on');                              % 返回重力异常等高线矩阵，（mGAL）

%clearvars x_c y_c X_c Y_c Z_c;                                              % 清楚多余变量

plot(xm,ym,'c')

hold on 
plot(pos_INS(:,1),pos_INS(:,2),...
    'color','b',...
    'Linestyle','-')
xlabel('North (m)')
ylabel('East (m)')
title('Position')
grid on

figure(2)
surf(X_c,Y_c,Z_c,'FaceAlpha',0.7)




