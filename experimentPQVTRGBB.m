plotFlag = 0;

nRuns = 50;
expDistanceError = [];
expOrientationError = [];
expVelocityError = [];
expPicError = [];
expQicError = [];
expGravityError = [];
expBiasAccelError = [];
expBiasGyroError = [];

fprintf('Run ... ');
for r=1:nRuns
    
    fprintf('%d',r);
    pqvtrgbbUKF
    expDistanceError = [expDistanceError distanceError'];
    expOrientationError = [expOrientationError orientationError'.*180/pi];
    expVelocityError = [expVelocityError velocityError'];
    expPicError = [expPicError picError'];
    expQicError = [expQicError qicError'.*180/pi];
    expGravityError = [expGravityError gravityError'];
    expBiasAccelError = [expBiasAccelError biasAccelError'];
    expBiasGyroError = [expBiasGyroError biasGyroError'];
    
    if (r<nRuns), fprintf(', '); end
    
end

fprintf(' done\n');

meanDistanceError = mean(expDistanceError,2);
stdDistanceError = std(expDistanceError,0,2);

meanOrientationError = mean(expOrientationError,2);
stdOrientationError = std(expOrientationError,0,2);

meanVelocityError = mean(expVelocityError,2);
stdVelocityError = std(expVelocityError,0,2);

meanPicError = mean(expPicError,2);
stdPicError = std(expPicError,0,2);

meanQicError = mean(expQicError,2);
stdQicError = std(expQicError,0,2);

meanGravityError = mean(expGravityError,2);
stdGravityError = std(expGravityError,0,2);

meanBiasAccelError = mean(expBiasAccelError,2);
stdBiasAccelError = std(expBiasAccelError,0,2);

meanBiasGyroError = mean(expBiasGyroError,2);
stdBiasGyroError = std(expBiasGyroError,0,2);

save('results/experimentPQVTRGBB.mat', ...
     'expDistanceError','expOrientationError','expVelocityError',...
     'expPicError','expQicError','expGravityError','expBiasAccelError',...
     'expBiasGyroError','meanDistanceError','stdDistanceError',...
     'meanOrientationError','stdOrientationError','meanVelocityError',...
     'stdVelocityError','meanPicError','stdPicError','meanQicError',...
     'stdQicError','meanGravityError','stdGravityError','meanBiasAccelError',...
     'stdBiasAccelError','meanBiasGyroError','stdBiasGyroError');
     
     
