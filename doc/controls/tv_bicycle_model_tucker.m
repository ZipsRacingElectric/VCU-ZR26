N = 101
M = 101

x = linspace (-600, 600, N);
y = linspace (-11, 11, M);

torqueRl = []
torqueRr = []
torqueFl = []
torqueFr = []

for i = [1:N]
  for j = [1:M]

  #TODO(Barach): Validate all these

  wheelRadius = 0.0254 * 10;
  gearRatio = 14;

  trackWidthFront = 1.22;
  trackWidthRear = 1.2;
  frontRearWeightBias = 0.52;

  wheelBase = 1.528;

  wheelAngle = y (j);

  lx = trackWidthFront / 2.0;
  ly = wheelBase * frontRearWeightBias;
  lr = sqrt (lx * lx + ly * ly);
  a = atand (ly / lx);

  yawTransferRl =  gearRatio * trackWidthRear / (2.0 * wheelRadius);
  yawTransferRr = -gearRatio * trackWidthRear / (2.0 * wheelRadius);

  yawTransferFl =  gearRatio * lr / wheelRadius * cosd (a - wheelAngle);
  yawTransferFr = -gearRatio * lr / wheelRadius * cosd (a + wheelAngle);

  throttlePosition = 0.0;
  torqueLimit = 84 * throttlePosition;

  momentIdeal = x (i);

  drivingTorqueLimit = 21;
  regenTorqueLimit = -21;

  # Option 1 (Will likely overload the FL tire in understeer conditions)

##  drivingBiasFr = 0.6;
##  momentBiasFr = 0.4;
##
##  A = [1,             1,             0,             0
##       0,             0,             1,             1
##       yawTransferRl, yawTransferRr, 0,             0
##       0,             0,             yawTransferFl, yawTransferFr];
##
##  B = [     drivingBiasFr  * torqueLimit,
##       (1 - drivingBiasFr) * torqueLimit,
##            momentBiasFr   * momentIdeal,
##       (1 - momentBiasFr)  * momentIdeal];

  # Option 2 (Cannot find any values that are normal, useless)

##  drivingBiasFr = 0.6;
##  drivingBiasLr = 0.45;
##
##  A = [1,                 1,                 1,                 1,
##       yawTransferRl,     yawTransferRr,     yawTransferFl,     yawTransferFr,
##       drivingBiasFr-1,   drivingBiasFr-1,   drivingBiasFr,     drivingBiasFr,
##       drivingBiasLr-1,   drivingBiasLr,     drivingBiasLr-1,   drivingBiasLr]
##
##  B = [torqueLimit,
##       momentIdeal,
##       0,
##       0]

  # Option 3 (Doesn't respect driving F/R bias in oversteer/understeer conditions)

##  drivingBiasFr = 0.6;
##
##  A = [1,             1,             1,             1,
##       yawTransferRl, yawTransferRr, yawTransferFl, yawTransferFr,
##       0,             2,             0,             0,
##       0,             0,             2,             0]
##
##  B = [torqueLimit,
##       momentIdeal,
##       drivingBiasFr * torqueLimit,
##       (1-drivingBiasFr) * torqueLimit]
##
##  C = A^-1 * B;
##
##  torqueRl (i) = C (1);
##  torqueRr (i) = C (2);
##  torqueFl (i) = C (3);
##  torqueFr (i) = C (4);

  # Current Implementation

  drivingBiasFr = 0.6;

  if (wheelAngle >= 0)
  #if (momentIdeal >= 0)
    yawTransferRo = yawTransferRl;
    yawTransferRi = yawTransferRr;
    yawTransferFo = yawTransferFl;
    yawTransferFi = yawTransferFr;
  else
    yawTransferRo = yawTransferRr;
    yawTransferRi = yawTransferRl;
    yawTransferFo = yawTransferFr;
    yawTransferFi = yawTransferFl;
  endif

  torqueRi = drivingBiasFr * torqueLimit / 2;
  torqueFo = (1 - drivingBiasFr) * torqueLimit / 2;

  momentRequired = momentIdeal - torqueRi * yawTransferRi - torqueFo * yawTransferFo;

  torqueRo =  (momentRequired - 0.5 * torqueLimit * yawTransferFi) / (yawTransferRo - yawTransferFi);
  torqueFi = -(momentRequired - 0.5 * torqueLimit * yawTransferRo) / (yawTransferRo - yawTransferFi);

  # TODO(Barach): Does not work.

  if (torqueRo > drivingTorqueLimit)
    torqueRo = drivingTorqueLimit
    torqueFi = torqueLimit/2 - drivingTorqueLimit
  endif
  if (torqueFi > drivingTorqueLimit)
    torqueFi = drivingTorqueLimit
    torqueRo = torqueLimit/2 - drivingTorqueLimit
  endif

  if (torqueRo < regenTorqueLimit)
    torqueRo = regenTorqueLimit;
    torqueFi = torqueLimit/2 - regenTorqueLimit;
  endif
  if (torqueFi < regenTorqueLimit)
    torqueFi = regenTorqueLimit;
    torqueRo = torqueLimit/2 - regenTorqueLimit;
  endif

  if (wheelAngle >= 0)
  #if (momentIdeal >= 0)
    torqueRl (j,i) = torqueRo;
    torqueRr (j,i) = torqueRi;
    torqueFl (j,i) = torqueFo;
    torqueFr (j,i) = torqueFi;
  else
    torqueRl (j,i) = torqueRi;
    torqueRr (j,i) = torqueRo;
    torqueFl (j,i) = torqueFi;
    torqueFr (j,i) = torqueFo;
  endif

endfor
endfor

figure;
hold on;

#surf (x, y (51:101), torqueRl (51:101,:));
#surf (x, y (1:49),   torqueRr (1:49,:));
#surf (x, y (1:49),   torqueFl (1:49,:));
#surf (x, y (51:101), torqueFr (51:101,:));

surf (x, y, torqueFl);
surf (x, y, torqueFr);

#plot (x, drivingTorqueLimit * ones (N));
#plot (x, regenTorqueLimit * ones (N));

#plot ([momentMax, momentMax], [regenTorqueLimit, drivingTorqueLimit])
#plot ([momentMin, momentMin], [regenTorqueLimit, drivingTorqueLimit])

#legend ("RL", "RR", "FL", "FR");

#xlabel ("Moment Request (Nm)");
#ylabel ("Motor Torques (Nm)");
