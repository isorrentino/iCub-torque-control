modelPath = '../urdf/';
modelName = 'iCub3_right_leg.urdf';

%% Load the estimator
estimator = iDynTree.ExtWrenchesAndJointTorquesEstimator();

% Load model and sensors from the URDF file
estimator.loadModelAndSensorsFromFile(strcat(modelPath,modelName));
estimator.model().toString() % print model

%% Set kinematics information
grav_idyn = iDynTree.Vector3();
grav = [0.0;0.0;-9.81];
grav_idyn.fromMatlab(grav);

% Get joint information. for the time being we can just assume that
% all the joints position, velocity and acceleration are zero.
% Warning!! iDynTree takes in input **radians** based units,
% while the iCub port stream **degrees** based units.
dofs = estimator.model().getNrOfDOFs();

% >>>>>>>>>>>>>>>> Questo va in un for loop
qj = zeros(dofs,1);
dqj = zeros(dofs,1);
ddqj = zeros(dofs,1);

qj_idyn   = iDynTree.JointPosDoubleArray(dofs);
dqj_idyn  = iDynTree.JointDOFsDoubleArray(dofs);
ddqj_idyn = iDynTree.JointDOFsDoubleArray(dofs);

qj_idyn.fromMatlab(qj);
dqj_idyn.fromMatlab(dqj);
ddqj_idyn.fromMatlab(ddqj);

r_sole_index = estimator.model().getFrameIndex('r_sole');
estimator.updateKinematicsFromFixedBase(qj_idyn,dqj_idyn,ddqj_idyn,r_sole_index,grav_idyn);


%% Specify unknown wrenches

% We need to set the location of the unknown wrench. We express the unknown
% wrench at the origin of the r_sole frame
unknownWrench = iDynTree.UnknownWrenchContact();
unknownWrench.unknownType = iDynTree.FULL_WRENCH;

% the position is the origin, so the conctact point wrt to r_sole is zero
unknownWrench.contactPoint.zero();

% The fullBodyUnknowns is a class storing all the unknown external wrenches
% acting on a class
fullBodyUnknowns = iDynTree.LinkUnknownWrenchContacts(estimator.model());
fullBodyUnknowns.clear();

fullBodyUnknowns.addNewContactInFrame(estimator.model(),r_sole_index,unknownWrench);

% Print the unknowns to make sure that everything is properly working
fullBodyUnknowns.toString(estimator.model())






