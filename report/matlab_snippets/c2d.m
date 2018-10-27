%% Create state-space model using the linearized system matrices
Mc = ss(A,B,C,D);

%%- Calculate the discrete-time linear model of the system.
Md = c2d(Mc, sampling_time, 'zoh'); 

%%- Get the discretized system and input matrix
Phi = Md.A;
Gamma = Md.B;
