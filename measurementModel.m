function z_k=measurementModel(x_k, world_pts, K, usetranslationonly)

% x_k: 26x1 state vector or 26 x 2N+1 sigma points
% world_pts: 3xP points
% K: 3x3 matrix
% usetranslationonly: true or false, true if only translation will be used
% z_k: 2Px1 output (first all x coordinates, then all y coordinates)

if usetranslationonly
    z_k=measurementModelTranslation(x_k(1:3,:), x_k(20:22), x_k(4:7,:), x_k(23:26), world_pts, K);
else
    fprintf('Sorry, the general model is not yet implemented :). set usetranslationonly to 1');
    z_k=[];
end

end