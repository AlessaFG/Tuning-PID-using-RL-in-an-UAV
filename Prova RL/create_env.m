obsInfo = rlNumericSpec([12 1], 'LowerLimit', -inf, 'UpperLimit', inf);
actInfo = rlNumericSpec([12 1], 'LowerLimit', 0, 'UpperLimit', 1);
env = rlFunctionEnv(obsInfo, actInfo, "quadrotorStepFunction", "myResetFunction");
