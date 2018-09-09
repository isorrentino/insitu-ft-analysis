function test_suite = tests_on_utils
%     this suite can be run using the moxunit_runtests, check 
%     https://github.com/MOxUnit/MOxUnit for more info
    try % assignment of 'localfunctions' is necessary in Matlab >= 2016
        test_functions=localfunctions();
    catch % no problem; early Matlab versions can use initTestSuite fine
    end
    initTestSuite;
    
function test_estimateCalibrationMatrix
%     Test for the estimateCalibrationMatrix function 
%     TODO: tests on the different functionalities of the function
%     Generate N random wrenches
    nSamples = 100;
    W = rand(nSamples,6);
%     We assume a random non-singular C and a random offset 
    C = rand(6,6);
%    C = eye(6,6);
    R =(W)*(inv(C)');
    [Cest,full_scale] = estimateCalibrationMatrix(R,W);
    assertElementsAlmostEqual(Cest,C);
    Cest
    C
%     let's have an offset simple such that it is easier to debug 
    offset = (1.0:6.0)';
%     as we have that:
%     w = C*r + offset 
%     we also have : 
%     w' = r'*C' - offset' 
%     stacking the row vectors w,r in the datset matrices 
%     of dimension nSamples \times 6 we have: 
%     W = R*C'  + repmat(offset',nSamples,1)
%     From which we get, given that C is invertible and inv(C)' = inv(C')
%    R = (W-repmat(offset',nSamples,1))*(inv(C)') 
    R = (W-repmat(offset',nSamples,1))*(inv(C)');
%     Now, if we pass R computed in this way we should get C and offset 
    [Cest,full_scale,offsetEst] = estimateCalibrationMatrix(R,W,'estimateoffset',true);
    assertElementsAlmostEqual(Cest,C);
    assertElementsAlmostEqual(round(offsetEst),offset);
    offsetEst
    offset  
    
    
    

