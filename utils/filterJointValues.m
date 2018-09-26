function [filteredQj,mask ]=filterJointValues(qj,timeDelta)
N=2;
F=tests+1;
mask= false(length(dataset.qj),1);
for joint=1:size(qj,2)
y = qj(:,joint);
        nrOfSamples = length(qj);
        [b,g] = sgolay(N,F);
        HalfWin  = ((F+1)/2) -1;        
        for n = (F+1)/2:nrOfSamples-(F+1)/2
            % Zeroth derivative (smoothing only)
            filteredQj(n,joint) = dot(g(:,1),y(n - HalfWin:n + HalfWin));
            % First derivative
        end
end     
mask((F+1)/2:nrOfSamples-(F+1)/2)=1;