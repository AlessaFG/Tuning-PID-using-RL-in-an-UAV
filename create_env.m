% obsInfo = rlNumericSpec([12 1], 'LowerLimit', -inf, 'UpperLimit', inf);
% actInfo = rlNumericSpec([6 1], 'LowerLimit', 0, 'UpperLimit', 35);
% env = rlFunctionEnv(obsInfo, actInfo, "quadrotorStepFunction", "myResetFunction");


obsInfo = rlNumericSpec([6 1], 'LowerLimit', -inf, 'UpperLimit', inf);
actInfo = rlNumericSpec([6 1], 'LowerLimit', 0, 'UpperLimit', 35);
env = rlFunctionEnv(obsInfo, actInfo, "quadrotorStepFunction", "myResetFunction");
