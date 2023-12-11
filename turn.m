filename=['turn.csv'];
M=csvread(filename);
turn_dps_per_sec=M(:,1);
signal=M(:,2);