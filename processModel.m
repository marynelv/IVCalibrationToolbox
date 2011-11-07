function x_k=processModel(x_kminus1, timestep, usetranslationonly)

% x_kminus1: 26x1 state vector or 26 x 2N+1 sigma points
% timestep: scalar
% usetranslationonly: true or false, true if only translation model is used
% 
% x_k: if only translation model is used: 3x1 state vector or 3 x 2N+1
%      sigma points
%      if translation model is not used: 26x1 state vector or 26 x 2N+1
%      sigma points
%
% NOTE: The full x_k vector is defined in equation (13), page 9
if usetranslationonly
    x_k=processModelTranslation(x_kminus1(1:3,:), x_kminus1(8:10,:), timestep);
else
    fprintf('Sorry, the general model is not yet implemented :). set usetranslationonly to 1');
    x_k=[];
end

end