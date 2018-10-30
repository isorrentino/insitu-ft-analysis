function [xFiltered, dxFiltered] = sgolayFilterAndDerivate(polynomOrder,frameLen,derivOrder,xRaw,dt)

[b,g] = sgolay(polynomOrder,frameLen);

dx = zeros(numel(xRaw),2);

if derivOrder>(polynomOrder+1)
    error('Boundary exceeded: the derivative order must be ≤ order of the fitting polynom + 1 !!');
end

for p = 0:derivOrder
  dx(:,p+1) = conv(xRaw, factorial(p)/(-dt)^p * g(:,p+1), 'same');
end

xFiltered = dx(:,1);
dxFiltered = dx(:,2:end);

end