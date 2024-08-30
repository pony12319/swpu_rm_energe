function [temp] = fit(particlesize,x,t0,theta)
    fitness=@(x,t,theta)abs((mod((-x(1)./x(2) *cos(x(2)*t+x(3))+(2.090-x(1))*t),2*pi) - theta));
    f = zeros(particlesize,length(theta));
    for i=1:particlesize
        for j = 1:1:length(theta)
	        f(i,j)=fitness(x(i,:),t0(j),theta(j));	
        end
    end
    temp = zeros(particlesize,1);
    for i = 1:length(temp)
        temp(i,1) = sum(f(i,:));
    end
end