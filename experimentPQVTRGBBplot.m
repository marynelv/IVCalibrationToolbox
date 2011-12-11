f = figure('position',[10,10,800,300]);

%% Position

values = meanDistanceError(2:2:end-6);
values2 = meanDistanceError(1:100:end-6);
values2STD = stdDistanceError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;

axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');
 
set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Distance.png', '-dpng');

%% Orientation

values = meanOrientationError(2:2:end-6);
values2 = meanOrientationError(1:100:end-6);
values2STD = stdOrientationError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;
 
axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Orientation.png', '-dpng');

%% Velocity

values = meanVelocityError(2:2:end-6);
values2 = meanVelocityError(1:100:end-6);
values2STD = stdVelocityError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;

axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Velocity.png', '-dpng');

%% Gravity

values = meanGravityError(2:2:end-6);
values2 = meanGravityError(1:100:end-6);
values2STD = stdGravityError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;
 
axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Gravity.png', '-dpng');

%% Pic

values = meanPicError(2:2:end-6);
values2 = meanPicError(1:100:end-6);
values2STD = stdPicError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;
 
axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Pic.png', '-dpng');

%% Qic

values = meanQicError(2:2:end-6);
values2 = meanQicError(1:100:end-6);
values2STD = stdQicError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;
 
axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-Qic.png', '-dpng');

%% Accel Bias

values = meanBiasAccelError(2:2:end-6);
values2 = meanBiasAccelError(1:100:end-6);
values2STD = stdBiasAccelError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;

axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-AccelBias.png', '-dpng');

%% Gyro Bias

values = meanBiasGyroError(2:2:end-6);
values2 = meanBiasGyroError(1:100:end-6);
values2STD = stdBiasGyroError(1:100:end-6);
plot(1:length(values),values);
hold on;
errorbar([1 (1:length(values2)-1)*50],values2,values2STD,'xr');
hold off;
 
axis([0 length(values) 0 max(values)+max(values2)]);
xlabel('Time'); ylabel('Error');

set(gca,'FontSize',18);
set(get(gca,'XLabel'),'FontSize',18); 
set(get(gca,'YLabel'),'FontSize',18); 
set(gcf,'PaperUnits','points')
set(gcf,'PaperPosition',[10,10,800,300])

print(gcf, 'final-report/experiment-GyroBias.png', '-dpng');