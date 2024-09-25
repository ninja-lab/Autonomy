k_rat = 2;
C = tf([1 k_rat ], 1) ;
P = tf(1, [1 1 0]);
FG = series(C,P );

rlocus(FG)