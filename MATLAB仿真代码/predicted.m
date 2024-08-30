function [x,y]=predicted(X,a,w,t,pret,L)
% X 状态空间 a,w pso迭代 t 时间轴时间 dt 预测时间 L 旋转方向
    dt = 0.01;
    for i = 1:floor(pret/dt)
        x = X(1);
        y = X(2);
        omega = X(3);
        phi = X(4);

        A = [x*cos(omega*dt) - y*sin(omega*dt);
            y*cos(omega*dt) + x*sin(omega*dt);
            L*(a*sin(w*(t+dt) + phi) + 2.090 - a);
            phi];
        
        X = A;
        t = t + dt;
%         disp(['当前循环的i值为: ', num2str(i)]);
    end
    x = X(1,1);
    y = X(2,1);
end