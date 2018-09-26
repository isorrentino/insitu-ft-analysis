function [dx]=numericalDifferentiation(x,delta)
xMinusH=x(1:end-2,:);
xPlusH=x(3:end,:);

dx=(xPlusH-xMinusH)/(2*delta);
dx=[zeros(size(dx(1,:)));
dx;
zeros(size(dx(1,:)))];