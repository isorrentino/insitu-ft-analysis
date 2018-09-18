function [lambdasNames]=generateLambdaNames(lambdas)

for namingIndex=1:length(lambdas)
    if (lambdas(namingIndex)==0)
        lambdasNames{namingIndex}='';
    else
        lambdasNames{namingIndex}=strcat('_l',num2str(lambdas(namingIndex)));
    end
end
lambdasNames=lambdasNames';
