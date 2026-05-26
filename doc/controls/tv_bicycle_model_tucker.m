N = 101

#x = linspace (-11, 11, N)
x = linspace (0, 500, N)

torqueRl = []
torqueRr = []
torqueFl = []
torqueFr = []

for i = [1:N]

  #TODO(Barach): Validate all these

  wheelRadius = 0.0254 * 10;
  gearRatio = 14;

  trackWidthFront = 1.22;
  trackWidthRear = 1.2;
  frontRearWeightBias = 0.52;

  wheelBase = 1.528;

  # wheelAngle = 11 * x (i) / 500;
  wheelAngle = 11;

  lx = trackWidthFront / 2.0;
  ly = wheelBase * frontRearWeightBias;
  lr = sqrt (lx * lx + ly * ly);
  a = atand (ly / lx);

  yawTransferRl =  gearRatio * trackWidthRear / (2.0 * wheelRadius);
  yawTransferRr = -gearRatio * trackWidthRear / (2.0 * wheelRadius);

  yawTransferFl =  gearRatio * lr / wheelRadius * cosd (a - wheelAngle);
  yawTransferFr = -gearRatio * lr / wheelRadius * cosd (a + wheelAngle);

  throttlePosition = 0.5;
  torqueLimit = 84 * throttlePosition;

  momentIdeal = x (i);

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

  drivingBiasFr = 0.6;

  A = [1,             1,             1,             1,
       yawTransferRl, yawTransferRr, yawTransferFl, yawTransferFr,
       0,             2,             0,             0,
       0,             0,             2,             0]

  B = [torqueLimit,
       momentIdeal,
       drivingBiasFr * torqueLimit,
       (1-drivingBiasFr) * torqueLimit]

  C = A^-1 * B;

  torqueRl (i) = C (1);
  torqueRr (i) = C (2);
  torqueFl (i) = C (3);
  torqueFr (i) = C (4);

  torqueRr2 (i) = drivingBiasFr * torqueLimit / 2;
  torqueFl2 (i) = (1 - drivingBiasFr) * torqueLimit / 2;

  momentRequired = momentIdeal - torqueRr2 (i) * yawTransferRr - torqueFl2 (i) * yawTransferFl

  torqueRl2 (i) =  (momentRequired - 0.5 * torqueLimit * yawTransferFr) / (yawTransferRl - yawTransferFr);
  torqueFr2 (i) = -(momentRequired - 0.5 * torqueLimit * yawTransferRl) / (yawTransferRl - yawTransferFr);

endfor

figure;
hold on;
plot (x, torqueRl);
plot (x, torqueRl2);

#figure;
#hold on;

#plot (x, torqueRl, c='r');
#plot (x, torqueRr, c='b');
#plot (x, torqueFl, c='y');
#plot (x, torqueFr, c='g');

#plot (x, zeros (N))
#plot (x, 21 * ones (N))

legend ("RL", "RR", "FL", "FR");

xlabel ("Moment Request (Nm)");
ylabel ("Motor Torques (Nm)");
