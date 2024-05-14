%% Inverted Pendulum Simulator with Animation
%by Gene Patrick Rible
%May 4, 2024

%This MATLAB code is a supplementary material for the paper "Nonlinear
%Swing-Up and Stabilization of Inverted Pendulum on a Cart Using Energy
%Control Method Based on Multiple Lyapunov Functions, Sliding Mode Control,
%and Pole Placement" by G. P. Rible.

%Note: the control input U is the force F on the cart.

clear all
close all
clc

%%
f1=@invertedp;
%%
while 1
    Tf = input("How long is the simulation in s?\n");
    if (isreal(Tf)&&(Tf>0))
        break
    end
end
%Tf=25; %s           % END TIME
T=[0:0.01:Tf];       % Reducing the timestep from 0.01 slows down the simulation but increases simulation accuracy and provides more information on the behavior of the pendulum with time.
%% Initial Conditions
while 1
    y0_x = input("Enter initial cart position in m:\n");
    if isreal(y0_x)
        break
    end
end
while 1
    y0_xdot = input("Enter initial cart velocity in m/s:\n");
    if isreal(y0_xdot)
        break
    end
end
while 1
    y0_theta = input("Enter initial rod angle in rad:\n");
    if isreal(y0_theta)
        break
    end
end
while 1
    y0_thetadot = input("Enter initial rod angular velocity in rad/s:\n");
    if isreal(y0_thetadot)
        break
    end
end

y0=[y0_x y0_xdot y0_theta y0_thetadot];
% y0=[0 0 pi 0];
% y0=[2 0 pi 0];
% y0=[2 0 -pi 0];
% y0=[1 0 pi 0];
% y0=[1 0 -pi 0];
% y0=[1 0 -pi/2 -0.9];  
% y0=[1 0 pi/2 0.9];    
% y0=[1 0 pi/2 -0.9];    
% y0=[1 0 -pi/2 0.9];    
% y0=[1 0 0.7168 0];
% y0=[1 0 -0.7168 0];
% y0=[1 0 0.7168 -10];
% y0=[1 0 -0.7168 10];

% y0=[1 0 5 54];
% y0=[1 0 -5 54];
% y0=[1 0 -5 -54];
% y0=[1 0 5 -54];


% y0=[1 0 4 -34];
% y0=[1 0 4 34];
% y0=[1 0 -4 34];
% y0=[1 0 -4 -34];

% y0=[1 0 2 -60];
% y0=[1 0 2 60];
% y0=[1 0 -2 60];
% y0=[1 0 -2 -60];

%%
K=[-35 -34 -150 -34];
%%
global U_matrix flag U_tracker question_SL; %flag2 
U_matrix=[];
flag = 0;
% flag2 = 0;
U_tracker = 7;
question_SL = nan;
%%
while 1
    question_SL = input("Enter 0 if you want to use Single Lyapunov controller, input any other number to use Multiple Lyapunov controller:\n");
    if ((isreal(question_SL))&&(~isnan(question_SL)))
        break
    end
end
while 1
    question_animation = input("Enter 0 if you want to animate the simulation, input any other number otherwise:\n");
    if isreal(question_animation)
        break
    end
end
if ~question_animation
    while 1
        question_time = input("Enter 0 if you want to include time in animation (slower simulation), input any other number otherwise:\n");
        if isreal(question_time)
            break
        end
    end    
end
while 1
    question_video = input("Enter 0 if you want to save the animation (slower simulation) to ''myfile'', input any other number otherwise:\n");
    if isreal(question_animation)
        break
    end
end
%%
[t, y] = ode45(f1, T, y0, [], K);
% t1=t;
% y1=y;
% U_matrix_1 = U_matrix;

l=0.5; %m

%% Animation setup

if ~question_animation
    fh = figure(1);
    if ~question_video
        % scr_siz = get(0,'ScreenSize');
        set(fh,'Position',[0 0 1440 900]);
        % fh.WindowState = 'maximized';
        size_fontsize = 30;
        size_pendlinewidth = 5;        
    else
        size_fontsize = 10;
        size_pendlinewidth = 2;
    end
    daspect([1 1 1])
    xmin = -5; xmax = 5; % Axis limits for better visualization
    ymin = -0.5; ymax = 0.5;
    axis([xmin xmax ymin ymax]);
    axis manual;
    ylabel('$y~\left [\textrm{m}  \right ]$','Interpreter','latex','FontSize',size_fontsize)
    xlabel('$x~\left [\textrm{m}  \right ]$','Interpreter','latex','FontSize',size_fontsize)
    if ~question_SL
        txt = sprintf('$\\textrm{Single Lyapunov}: \\quad x_0=%.2f~\\textrm{m},~\\dot{x}_0=%.2f~\\textrm{m}/\\textrm{s},\\theta_0=%.2f~\\textrm{rad},~\\dot{\\theta}_0=%.2f~\\textrm{rad}/\\textrm{s}$', y0_x,y0_xdot,y0_theta,y0_thetadot);
        title(txt,'Interpreter','latex','FontSize',size_fontsize)          
    else
        txt = sprintf('$\\textrm{Lyapunov Switching}: \\quad x_0=%.2f~\\textrm{m},~\\dot{x}_0=%.2f~\\textrm{m}/\\textrm{s},\\theta_0=%.2f~\\textrm{rad},~\\dot{\\theta}_0=%.2f~\\textrm{rad}/\\textrm{s}$', y0_x,y0_xdot,y0_theta,y0_thetadot);
        title(txt,'Interpreter','latex','FontSize',size_fontsize)      
    end
    set(gca,'FontSize',size_fontsize, 'FontName', 'Times')
    hold on;
    % Draw cart as rectangle
    cart_width = 0.4;
    cart_height = 0.2;
    cart = rectangle('Position', [y(1,1)-cart_width/2, -cart_height/2, cart_width, cart_height], 'FaceColor', 'k');
    % Draw pendulum as a line
    pendulum = line([0 0], [0 -l], 'LineWidth', size_pendlinewidth, 'Color', 'b');
    
    if ~question_video
        vidObj = VideoWriter('myFile','MPEG-4');
        vidObj.FrameRate = 100;
        open(vidObj)
    end
    % Animation loop
    for k = 1:length(t)
        x = y(k, 1);
        phi = pi-y(k, 3);
        % Update cart position
        set(cart, 'Position', [x-cart_width/2, -cart_height/2, cart_width, cart_height]);
        % Update pendulum position
        pend_x = x + l * sin(phi);
        pend_y = -l * cos(phi);
        set(pendulum, 'XData', [x pend_x], 'YData', [0 pend_y]);
    
        if ~question_time
            delete(findall(gcf,'FitBoxToText','on'));
            txt = sprintf('%.2f s', t(k));
            ah = annotation('textbox',[0.13,0.2,.3,.3],'string',txt,'interpreter','latex','FitBoxToText','on','FontSize',size_fontsize);
            ah.EdgeColor = 'none'; 
            ah.Color = 'r';     
        end

        drawnow;

        if ~question_video
            F = getframe(gcf);
            writeVideo(vidObj,F)
        end
    end
    if ~question_video
        close(vidObj)
    end
    close all
end    
%% PLOTS

%Plot x
figure(2)
subplot(2,2,1)
plot(t, y(:,1),'r')
ylabel('$x ~\left [\textrm{m}  \right ]$','interpreter','latex')
xlabel('$t ~\left [\textrm{s}  \right ]$','interpreter','latex')
grid
set(gca,'FontSize',10, 'FontName', 'Times')

%Plot time derivative of x
subplot(2,2,2)
plot(t, y(:,2),'r')
ylabel('$\dot{x} ~\left [\textrm{m}/\textrm{s}  \right ]$','interpreter','latex')
xlabel('$t ~\left [\textrm{s}  \right ]$','interpreter','latex')
grid
set(gca,'FontSize',10, 'FontName', 'Times')

%Plot theta
theta_mat = mod(y(:,3),2*pi);
for i=1:length(theta_mat)
    if(theta_mat(i)>pi)
        theta_mat(i) = theta_mat(i)-2*pi;
    end
end
% theta_mat_1 = mod(y1(:,3),2*pi);
% for i=1:length(theta_mat_1)
%     if(theta_mat_1(i)>pi)
%         theta_mat_1(i) = theta_mat_1(i)-2*pi;
%     end
% end

subplot(2,2,3)
plot(t, theta_mat.*180./pi,'r')
ylabel('$\theta ~\left [^{\circ}  \right ]$','interpreter','latex')
xlabel('$t ~\left [\textrm{s}  \right ]$','interpreter','latex')
grid
set(gca,'FontSize',10, 'FontName', 'Times')

%Plot time derivative of theta
thetadot_mat = y(:,4);
subplot(2,2,4)
plot(t, thetadot_mat,'r')
ylabel('$\dot{\theta} ~\left [\textrm{rad}/\textrm{s}  \right ]$','interpreter','latex')
xlabel('$t ~\left [\textrm{s}  \right ]$','interpreter','latex')
grid
set(gca,'FontSize',10, 'FontName', 'Times')

%Plot input
figure(3)
% scatter(U_matrix_1(:,2),U_matrix_1(:,1),200,'o','MarkerEdgeColor','b')
% hold on
% scatter(U_matrix(:,2),U_matrix(:,1),200,'*','MarkerEdgeColor','r')
scatter(U_matrix(:,2),U_matrix(:,1),200,'o','MarkerEdgeColor','r')
% ylim([-250 250]);
ylabel('$F~\left [\textrm{N}  \right ]$','Interpreter','latex','FontSize',30)
xlabel('$t~\left [\textrm{s}  \right ]$','Interpreter','latex','FontSize',30)
grid
set(gca,'FontSize',20, 'FontName', 'Times')

%Plot input type
figure4 = figure(4);
% scatter(U_matrix_1(:,2),U_matrix_1(:,3),200,'o','MarkerEdgeColor','b')
% hold on
% scatter(U_matrix(:,2),U_matrix(:,3),200,'*','MarkerEdgeColor','r')
scatter(U_matrix(:,2),U_matrix(:,3),200,'o','MarkerEdgeColor','r')
ylim([0 7]);
xlabel('$t~\left [\textrm{s}  \right ]$','Interpreter','latex','FontSize',30)
grid
set(gca,'FontSize',20, 'FontName', 'Times')
set(gca,'TickLabelInterpreter','latex')
% curtick = get(gca, 'YTick');
% set(gca, 'YTick', unique(round(curtick)))
yticks([0 1 2 3 4 5 6 7])
yticklabels({'$u_{\theta_{1,0}}$','$u_{\theta_{1,1}}$','$u_{\theta_2}$','$u_{\theta_3}$','$u_{L2}$', [], [],'$u_{L1}$'})
% h(1) = plot(nan, nan, 'bo', 'MarkerSize', 200, 'DisplayName', 'Single Lyapunov','LineWidth',3);
% h(2) = plot(nan, nan, 'r*', 'MarkerSize', 200, 'DisplayName', 'Lyapunov Switching','LineWidth',3);
% legend(h)


% % %Plot only theta
% figure5 = figure(5);
% % plot(t1, theta_mat_1.*180./pi,'-b','LineWidth',3)
% % hold on
% % plot(t, theta_mat.*180./pi,'--r','LineWidth',3)
% plot(t, theta_mat.*180./pi,'-r','LineWidth',3)
% ylabel('$\theta~\left [^{\circ}  \right ]$','Interpreter','latex','FontSize',30)
% xlabel('$t~\left [\textrm{s}  \right ]$','Interpreter','latex','FontSize',30)
% set(gca,'FontSize',20, 'FontName', 'Times')
% grid

% %Plot only x
% figure6 = figure(6);
% % plot(t1, y1(:,1),'-b','LineWidth',3)
% % hold on
% % plot(t, y(:,1),'--r','LineWidth',3)
% plot(t, y(:,1),'-r','LineWidth',3)
% ylabel('$x~\left [m  \right ]$','Interpreter','latex','FontSize',30)
% xlabel('$t~\left [\textrm{s}  \right ]$','Interpreter','latex','FontSize',30)
% set(gca,'FontSize',20, 'FontName', 'Times')
% grid


%%
function Yp = invertedp(t, y, K)

global U_matrix flag U_tracker question_SL; %flag2 
% global y1d y1d_past;

%System Parameters
c=0.1; %N m s
M=2; %kg
m=0.1; %kg
% n=0; %kg
l=0.5; %m
g=9.8; %m/s^2
I=0.0; %kg.m^2
% J=0; %kg.m^2
% b=0; %N s/m

% u_gain1 = 30;
% u_gain2 = 30;
% u_gain1 = 29.5;
% u_gain2 = 29.5;
% u_gain1 = 29.35;
% u_gain2 = 29.35;
% u_gain1 = 29.8;
% u_gain2 = 29.9;
u_gain1 = 29.8;
u_gain2 = 29.9;
%Controller Parameters
U0 = u_gain1; U1 = u_gain2; U2 = u_gain2; U3 = u_gain2;

%Variable Calculations
y1=y(1);
y2=y(2);
% sin_theta = sin(y(3));
% cos_theta = cos(y(3));
theta = mod(y(3),2*pi);
if(theta>pi)
    theta = theta-2*pi;
end
thetadot = y(4);
sin_theta = sin(theta);
cos_theta = cos(theta);
denom = l*(m*(sin_theta^2)+M);

y_in = [y1; y2; theta; thetadot];

%Control Law Calculation
E = 0.5*((I+m*l^2)*(thetadot)^2)+m*g*l*(cos_theta-1); %%Note: J is excluded from the energy calculation.
mode1 = sign(E*thetadot*cos_theta);
mode2 = sign(-thetadot*cos_theta);


% if((abs(y1)>5)&&t>10)
%     error('Cart went out of boundaries.')
% %     flag = 0;
% end

% %Re-Initializer
% if (abs(y1)>10)
%     flag=0;
% end

if flag||((abs(cos_theta)>=0.9)&&(abs(thetadot)<0.0000001)&&(y2<0.000001)&&(abs(y1)<1.5))

    U=-K*y_in;
%     %Conditional Switchers
%     A=(U>100)&&(theta<(-0.14))&&(theta>(-pi/2));
%     B=(U<-100)&&(theta>0.14)&&(theta<(pi/2));
    if (cos_theta>=0.9) %&&(abs(y1)<5) %&&(abs(thetadot)<0.9) %&&(~(A||B))  %&&(abs(U)<500) %Pendulum Up Stabilizing Controller: LQR Pole Placement
        fprintf('linear: %d s\n',t);
        U_tracker = 4;
        %flag2 = 1;
    elseif ~question_SL %Single Lyapunov Swing-Up Controller
        fprintf('Lyapunov0: %d s\n',t);
        U = U0*mode1;
        U_tracker = 0;       
    else %Lyapunov Switching Swing-Up Controller
        if (thetadot>0) && (theta<0) && (theta<pi) && (abs(E)<0.05)
            fprintf('Lyapunov2: %d s\n',t);
            U = U2*mode2;
            U_tracker = 2;
        elseif (thetadot<0) && (theta<0) && (theta>(-pi)) && (abs(E)<0.05)
            fprintf('Lyapunov3: %d s\n',t);
            U = U3*mode2;
            U_tracker = 3;
        else
            if abs(E)>0.05
                fprintf('Lyapunov0: %d s\n',t);
                U = U0*mode1;
                U_tracker = 0;
            else
                fprintf('Lyapunov1: %d s\n',t);
                U = U1*mode2;
                U_tracker = 1;
            end
        end
    end
     
%     if((U<0)&&(~flag2)) %Accelerator: Lyapunov Converter
%         U=abs(U);
%         if y2>4
%             flag2 = 1;
%         end
%         U_tracker = 6;
%     end
%     if((U>0)&&(~flag2))
%         U=-abs(U);
%         if y2<-4
%             flag2 = 1;
%         end
%         U_tracker = 6;
%     end

    flag=1;
   
else %Pendulum Down Initializer: LQR Pole Placement
    fprintf('Intializer: %d s\n',t);
    if(theta>0)
        phi=pi-theta;
    else
        phi=-theta-pi;
    end
    phidot=-thetadot;
    if(abs(phi)>pi/2)
        U = 20*(phidot)-5*(y1)-10*y2;
    else
        U = 50*phi+20*(phidot)-5*(y1)-10*y2;
    end
    U_tracker = 7;
end

% if(y1>4) %Bounder
%     U=-min(abs(U),70);
%     %U=-abs(U);
%     U_tracker = 8;
% elseif(y1<-4)
%     U=min(abs(U),70);
%     %U=abs(U);
%     U_tracker = 8;
% end


% if(U>100) %Input Limiter
% %     U=0;
%     U=U0*mode1;
%     U_tracker = 9;
% elseif (U<-100)
% %     U=-0;
%     U=U0*mode1;
%     U_tracker = 9;
% end

U_matrix = [U_matrix;U,t,U_tracker];

Yp = zeros(4,1);


%Note: I, J, n, b are excluded in the following model:
Yp(1)=y(2);
Yp(2)= -(m*(g*sin_theta-c*thetadot)*cos_theta-l*m*((thetadot)^2)*sin_theta-U)/(denom/l);
Yp(3)=thetadot;
Yp(4)= -(((l*m*((thetadot)^2)*sin_theta)+U)*cos_theta-(m+M)*(g*sin_theta-c*thetadot))/denom;
end