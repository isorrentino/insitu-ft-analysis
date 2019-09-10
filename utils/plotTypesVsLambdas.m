%% selecty color order
my_colorOrder13=[
    253,141,60;
    252,78,42;
    227,26,28;
    177,0,38;
    65,182,196;
    29,145,192;
    34,94,168;
    12,44,132;
    116,196,118;
    65,171,93;
    35,139,69;
    0,90,50;
    140,45,4;
    ]/255;

my_colorOrder19=[
    254,178,76
253,141,60
252,78,42
227,26,28
189,0,38
128,0,38
158,202,225
107,174,214
66,146,198
33,113,181
8,81,156
8,48,107
161,217,155
116,196,118
65,171,93
35,139,69
0,109,44
0,68,27
102,37,6
]/255;
%

 

figure,
set(gca,'ColorOrder',my_colorOrder13);
hold on;
plot(1:length(lambdas),extForceToPlot,'LineWidth',3)
legend(escapeUnderscores( xBarNames));
ax=axis; % Set axis tight only on y-axes
xlim([1 length(lambdas)])
set(gca,'XTick',1:length(lambdas));
set(gca,'XTickLabel',lambdas','fontsize',18);
xlabel('\lambda');
ylabel('Contact Force Magnitude (N)');
