function [ddqj,mask_acc,dqj]=filterJointData(qj,timeDelta)
% FILTERFTDATA Filter the FT data contained in the  ftData 
% A sgolay filter with N = 2 and F = 201 is run on the sensors measurement
% to smooth the sensors value
% The function returns in output a filteredFtData structure that contains 
% the filteredFata and a mask for the value of the original samples that
% are not zero in the filtered version 

 % values found during extensive tests
 polynomOrder=4;
 frameLen=81;%35;
 nrOfSamples=size(qj,1);
 % create mask
 mask_acc= false(size(qj,1),1);
 for joint=1:size(qj,2)
     [~,deriv]=sgolayFilterAndDerivate(polynomOrder,frameLen,2,qj(:,joint),timeDelta);%derivOrder,xRaw,dt)
     sfiltddq(:,joint)=deriv(:,2);
     sfiltdq(:,joint)=deriv(:,1);
 end
 mask_acc((frameLen+1)/2:nrOfSamples-(frameLen+1)/2)=1;
 ddqj=sfiltddq;
 dqj=sfiltdq;
 
end