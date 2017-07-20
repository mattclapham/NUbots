// Base class for all gait engines
// File: gait_engine.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include "GaitEngine.h"

#include <functional>

#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/comparison.h"
#include "utility/math/filter/LinSinFillet.h"
#include "utility/math/filter/SlopeLimiter.h"
#include "utility/math/filter/SmoothDeadband.h"

// Defines
#define DEFAULT_HALT_EFFORT_ARMS 0.25  // Default joint effort to use in the halt pose for all the arm joints
#define DEFAULT_HALT_EFFORT_LEGS 0.50  // Default joint effort to use in the halt pose for all the leg joints

//
// GaitEngine class
//
namespace gait {
using ServoID = utility::input::ServoID;

GaitEngine::GaitEngine()
    : model(nullptr)
    , haltJointCmd()
    , haltJointEffort()
    , haltUseRawJointCmds(false)
    , m_posX(0.0)
    , m_posY(0.0)
    , m_rotZ(0.0)
    , CONFIG_PARAM_PATH("/cap_gait/")
    , config()
    , m_gcvAccSmoothX(9)
    , m_gcvAccSmoothY(9)
    , m_gcvAccSmoothZ(9)
    , m_resetIntegrators(false)
    , m_saveIFeedToHaltPose(false)
    , rxRobotModel(config)
    , rxModel(config)
    , mxModel(config)
    , txModel(config)
    , m_gcvZeroTime(0.0)
    , m_plotData(false) {

    in.reset();
    out.reset();
    GaitEngine::updateHaltPose();
    GaitEngine::updateOdometry();

    // Reset the gait engine
    GaitEngine::reset();

    // Initialise the halt pose
    updateHaltPose();
    m_lastJointPose = m_jointHaltPose;

    // Set up callbacks for the basic feedback config parameters
    config.addParamCallback(std::bind(&GaitEngine::resizeFusedFilters, this, std::ref(config.basicFusedFilterN)));
    config.addParamCallback(std::bind(&GaitEngine::resizeDFusedFilters, this, std::ref(config.basicDFusedFilterN)));
    config.addParamCallback(std::bind(&GaitEngine::resizeIFusedFilters, this, std::ref(config.basicIFusedFilterN)));
    config.addParamCallback(std::bind(&GaitEngine::resizeGyroFilters, this, std::ref(config.basicGyroFilterN)));
    resizeFusedFilters(config.basicFusedFilterN);
    resizeDFusedFilters(config.basicDFusedFilterN);
    resizeIFusedFilters(config.basicIFusedFilterN);
    resizeGyroFilters(config.basicGyroFilterN);

    // TODO: How to do this crap?
    // Set up callbacks for the local config parameters
    // m_resetIntegrators.setCallback(std::bind(&GaitEngine::resetIntegrators, this));
    // m_showRxVis.setCallback(std::bind(&GaitEngine::callbackShowRxVis, this));
    // m_plotData.setCallback(std::bind(&GaitEngine::callbackPlotData, this));
    resetIntegrators();
    callbackShowRxVis();
    callbackPlotData();

    // Reset save integral feedback config parameter(s)
    resetSaveIntegrals();
}

// Reset function
void GaitEngine::reset() {
    // Reset the pose variables
    m_jointPose.reset();
    m_jointHaltPose.reset();
    m_lastJointPose.reset();
    m_inversePose.reset();
    m_abstractPose.reset();
    m_abstractHaltPose.reset();

    // Reset the gait command vector variables
    m_gcv.setZero();
    m_gcvInput.setZero();
    m_gcvDeriv.reset();
    m_gcvAccSmoothX.reset();
    m_gcvAccSmoothY.reset();
    m_gcvAccSmoothZ.reset();
    m_gcvAcc.setZero();

    // Reset the gait flags
    m_walk         = false;
    m_walking      = false;
    m_leftLegFirst = true;

    // Reset the step motion variables
    m_gaitPhase = 0.0;

    // Reset the blending
    resetBlending(USE_HALT_POSE);

    // Reset the capture step variables
    resetCaptureSteps(true);

    // Reset the integrators
    resetIntegrators();
}

void GaitEngine::resetBase() {
    in.reset();
    out.reset();
    updateHaltPose();
    m_posX = 0.0;
    m_posY = 0.0;
    m_rotZ = 0.0;
}

// Step function
void GaitEngine::step() {
    // Clear the plot manager for a new cycle
    m_PM.clear();

    // Update the visualisation for a new cycle
    m_rxVis.clear();
    m_rxVis.setVisOffset(config.visOffsetX, config.visOffsetY, config.visOffsetZ);

    // Reset integrator flags
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;
    m_savedArmIFeed = false;
    m_savedLegIFeed = false;

    // Process the gait engine inputs
    processInputs();

    // Update the halt pose (updates m_abstractHaltPose, m_jointHaltPose and the inherited halt pose variables)
    updateHaltPose();

    // Retrieve the current blend factor
    double factor = blendFactor();

    // Generate the arm and leg motions if the gait is currently supposed to be walking
    if (m_walking)  // Robot is walking...
    {
        // Initialise the abstract pose to the halt pose
        m_abstractPose = m_abstractHaltPose;

        // Leg motions
        if (config.tuningNoLegs) {
            // Coerce the abstract leg poses
            clampAbstractLegPose(m_abstractPose.leftLeg);
            clampAbstractLegPose(m_abstractPose.rightLeg);

            // Convert legs: Abstract --> Joint
            m_jointPose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);
        }
        else {
            // Generate the abstract leg motion
            abstractLegMotion(m_abstractPose.leftLeg);
            abstractLegMotion(m_abstractPose.rightLeg);

            // Coerce the abstract leg poses
            clampAbstractLegPose(m_abstractPose.leftLeg);
            clampAbstractLegPose(m_abstractPose.rightLeg);

            // Convert legs: Abstract --> Inverse
            m_inversePose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);

            // Generate the inverse leg motion
            inverseLegMotion(m_inversePose.leftLeg);
            inverseLegMotion(m_inversePose.rightLeg);

            // Convert legs: Inverse --> Joint
            m_jointPose.setLegsFromInversePose(m_inversePose.leftLeg, m_inversePose.rightLeg);
        }

        // Arm motions
        if (config.tuningNoArms) {
            // Coerce the abstract arm poses
            clampAbstractArmPose(m_abstractPose.leftArm);
            clampAbstractArmPose(m_abstractPose.rightArm);

            // Convert arms: Abstract --> Joint
            m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
        }
        else {
            // Calculate the abstract arm motion
            abstractArmMotion(m_abstractPose.leftArm);
            abstractArmMotion(m_abstractPose.rightArm);

            // Coerce the abstract arm poses
            clampAbstractArmPose(m_abstractPose.leftArm);
            clampAbstractArmPose(m_abstractPose.rightArm);

            // Convert arms: Abstract --> Joint
            m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
        }

        // Pose blending
        if (factor != 0.0) m_jointPose.blendTowards(m_jointHaltPose, factor);

        // Save the last joint pose that was commanded during walking
        m_lastJointPose = m_jointPose;
    }
    else if (m_blending)  // Robot is blending but not walking...
    {
        // Reassert the last joint pose that was commanded during walking
        m_jointPose = m_lastJointPose;

        // Pose blending
        if (factor != 0.0) m_jointPose.blendTowards(m_jointHaltPose, factor);
    }
    else  // Robot is not walking or blending...
    {
        // Set the working joint pose to the halt pose
        m_jointPose = m_jointHaltPose;
    }

    // Update the gait engine outputs
    updateOutputs();

    // Reset the integrators if their offsets were saved to the halt pose
    if (m_saveIFeedToHaltPose) {
        m_saveIFeedToHaltPose = false;
        resetIntegrators();
    }

    // Plotting
    if (m_PM.getEnabled()) {
        m_PM.plotScalar(usedIFusedX, PM_USED_IFUSEDX);
        m_PM.plotScalar(usedIFusedY, PM_USED_IFUSEDY);
        m_PM.plotScalar(factor, PM_HALT_BLEND_FACTOR);
    }

    // Update visualisation markers
    if (m_rxVis.willPublish()) m_rxVis.updateMarkers();

    // Publish the plot data
    m_PM.publish();
    m_rxVis.publish();
}

// Update the robot's halt pose
void GaitEngine::updateHaltPose() {
    // Calculate the required legAngleX with regard for motion stances
    double legAngleXFact = (config.enableMotionStances ? m_motionLegAngleXFact : 1.0);
    double legAngleX     = legAngleXFact * config.haltLegAngleX + (1.0 - legAngleXFact) * config.haltLegAngleXNarrow;

    // Set whether to use raw joint commands
    haltUseRawJointCmds = !config.useServoModel;

    // Set the halt pose for the legs
    m_abstractHaltPose.leftLeg.setPoseMirrored(
        config.haltLegExtension, legAngleX, config.haltLegAngleY, config.haltLegAngleZ);
    m_abstractHaltPose.rightLeg.setPoseMirrored(
        config.haltLegExtension, legAngleX, config.haltLegAngleY, config.haltLegAngleZ);
    m_abstractHaltPose.leftLeg.setFootPoseMirrored(config.haltFootAngleX, config.haltFootAngleY);
    m_abstractHaltPose.rightLeg.setFootPoseMirrored(config.haltFootAngleX, config.haltFootAngleY);

    // Set the halt pose for the arms
    m_abstractHaltPose.leftArm.setPoseMirrored(config.haltArmExtension, config.haltArmAngleX, config.haltArmAngleY);
    m_abstractHaltPose.rightArm.setPoseMirrored(config.haltArmExtension, config.haltArmAngleX, config.haltArmAngleY);

    // Apply the non-mirrored halt pose biases
    m_abstractHaltPose.leftLeg.angleX += config.haltLegAngleXBias;
    m_abstractHaltPose.rightLeg.angleX += config.haltLegAngleXBias;
    m_abstractHaltPose.leftLeg.footAngleX += config.haltFootAngleXBias;
    m_abstractHaltPose.rightLeg.footAngleX += config.haltFootAngleXBias;
    if (config.haltLegExtensionBias >= 0.0) {
        m_abstractHaltPose.leftLeg.extension += config.haltLegExtensionBias;
    }
    else {
        m_abstractHaltPose.rightLeg.extension -= config.haltLegExtensionBias;
    }
    m_abstractHaltPose.leftArm.angleX += config.haltArmAngleXBias;
    m_abstractHaltPose.rightArm.angleX += config.haltArmAngleXBias;

    // Set the halt joint efforts for the arms
    m_abstractHaltPose.leftArm.cad.setEffort(config.haltEffortArm);
    m_abstractHaltPose.rightArm.cad.setEffort(config.haltEffortArm);

    // Set the halt joint efforts for the legs
    m_abstractHaltPose.leftLeg.cld.setEffort(config.haltEffortHipYaw,
                                             config.haltEffortHipRoll,
                                             config.haltEffortHipPitch,
                                             config.haltEffortKneePitch,
                                             config.haltEffortAnklePitch,
                                             config.haltEffortAnkleRoll);
    m_abstractHaltPose.rightLeg.cld.setEffort(config.haltEffortHipYaw,
                                              config.haltEffortHipRoll,
                                              config.haltEffortHipPitch,
                                              config.haltEffortKneePitch,
                                              config.haltEffortAnklePitch,
                                              config.haltEffortAnkleRoll);

    // Set the halt pose support coefficients
    m_abstractHaltPose.leftLeg.cld.setSupportCoeff(0.5);
    m_abstractHaltPose.rightLeg.cld.setSupportCoeff(0.5);

    // Set the link lengths
    m_abstractHaltPose.setLinkLengths(config.legLinkLength, config.armLinkLength);

    // Convert the clamped halt pose into the joint representation (abstract poses derived from m_abstractHaltPose are
    // clamped later, always right before they are converted into another space, i.e. joint/inverse)
    pose::AbstractPose clampedAbstractHaltPose = m_abstractHaltPose;
    clampAbstractPose(clampedAbstractHaltPose);
    m_jointHaltPose.setFromAbstractPose(clampedAbstractHaltPose);

    // Transcribe the joint halt pose to the required halt pose arrays
    haltJointCmd    = m_jointHaltPose.writeJointPosArray();
    haltJointEffort = m_jointHaltPose.writeJointEffortArray();
}

// Set the robot's odometry
void GaitEngine::setOdometry(double posX, double posY, double rotZ) {
    // Update the odometry of the internal robot model object
    // Note: rotZ is interpreted as a fused yaw angle relative to the global fixed frame
    rxRobotModel.setOdom(posX, posY, rotZ);
}

// Update the robot's odometry
void GaitEngine::updateOdometry() {
    // Transcribe the position odometry information
    Eigen::Vector3d trunkPos = rxRobotModel.trunkLink.position();
    out.odomPosition[0]      = trunkPos.x();
    out.odomPosition[1]      = trunkPos.y();
    out.odomPosition[2]      = trunkPos.z();

    // Transcribe the orientation odometry information
    Eigen::Quaterniond trunkRot = rxRobotModel.trunkLink.orientation();
    out.odomOrientation[0]      = trunkRot.w();
    out.odomOrientation[1]      = trunkRot.x();
    out.odomOrientation[2]      = trunkRot.y();
    out.odomOrientation[3]      = trunkRot.z();
}

// Reset function for the integrators
void GaitEngine::resetIntegrators() {
    // Reset the integrators
    iFusedXFeedIntegrator.reset();
    iFusedYFeedIntegrator.reset();
    iFusedXFeedFilter.reset();
    iFusedYFeedFilter.reset();
    iFusedXLastTime = 0.0;
    iFusedYLastTime = 0.0;
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;

    // Reset the triggering config parameter if it is set
    if (m_resetIntegrators) {
        m_resetIntegrators = false;
    }
}

// Reset function for the config parameter(s) that save the integrated feedback values
void GaitEngine::resetSaveIntegrals() {
    // Reset the required config parameter(s) to false
    m_saveIFeedToHaltPose = false;
}

// Reset function for capture steps functionality
void GaitEngine::resetCaptureSteps(bool resetRobotModel) {
    // Reset basic feedback filters
    fusedXFeedFilter.reset();
    fusedYFeedFilter.reset();
    dFusedXFeedFilter.reset();
    dFusedYFeedFilter.reset();
    iFusedXFeedFilter.reset();
    iFusedYFeedFilter.reset();
    gyroXFeedFilter.reset();
    gyroYFeedFilter.reset();

    // Reset the integrator variables (Note: We don't reset the integrators themselves here as they have their own
    // time-based reset mechanism)
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;
    m_savedArmIFeed = false;
    m_savedLegIFeed = false;

    // Reset basic feedback variables
    fusedXFeed  = 0.0;
    fusedYFeed  = 0.0;
    dFusedXFeed = 0.0;
    dFusedYFeed = 0.0;
    iFusedXFeed = 0.0;
    iFusedYFeed = 0.0;
    gyroXFeed   = 0.0;
    gyroYFeed   = 0.0;

    // Reset capture step objects
    m_rxVis.init();
    m_rxVis.setVisOffset(config.visOffsetX, config.visOffsetY, config.visOffsetZ);
    if (resetRobotModel) {
        rxRobotModel.reset(resetRobotModel);
        rxRobotModel.setSupportLeg(m_leftLegFirst ? 1 : -1);  // Left leg first in the air means that the first support
                                                              // leg is the right leg, 1 in the margait/contrib sign
                                                              // convention
        rxRobotModel.supportExchangeLock = true;
    }
    rxModel.reset();
    mxModel.reset();
    txModel.reset();
    limp.reset();
    m_comFilter.reset(in.nominaldT);

    // Reset capture step variables
    adaptation.x()         = 1.0;
    adaptation.y()         = 1.0;
    lastSupportOrientation = 0.0;
    oldGcvTargetY          = 0.0;
    virtualSlope           = 0.0;
    stepTimeCount          = 0.0;
    lastStepDuration       = 0.0;
    stepCounter            = 0;
    noCLStepsCounter       = std::max((int) (m_gcvZeroTime / in.nominaldT + 0.5), 1);
    resetCounter           = 100;  // Force Mx to match Rx completely for the next/first few cycles
    cycleNumber            = 0;

    // Reset the motion stance variables
    if (resetRobotModel) {
        resetMotionStance();
    }

    // Reset save integral feedback config parameter
    resetSaveIntegrals();
}

// Reset walking function (this should be the only function that modifies the m_walking flag)
void GaitEngine::resetWalking(bool walking, const Eigen::Vector3d& gcvBias) {
    // Reset variables
    m_walking      = walking;
    m_gcv          = gcvBias;
    m_gaitPhase    = 0.0;
    m_leftLegFirst = config.leftLegFirst;

    // Initialise the blending
    if (m_walking) {
        resetBlending(USE_HALT_POSE);
        setBlendTarget(USE_CALC_POSE, config.startBlendPhaseLen);
    }
    else {
        setBlendTarget(USE_HALT_POSE, config.stopBlendPhaseLen);
    }

    // Reset the capture step variables
    resetCaptureSteps(walking);
}

// Process inputs function
void GaitEngine::processInputs() {
    // Transcribe whether it is desired of us to walk right now
    m_walk = in.gaitCmd.walk;

    // Transcribe and bias the desired gait velocity
    Eigen::Vector3d gcvBias(config.gcvBiasLinVelX, config.gcvBiasLinVelY, config.gcvBiasAngVelZ);
    if (m_walk) {
        m_gcvInput = gcvBias + Eigen::Vector3d(in.gaitCmd.linVelX, in.gaitCmd.linVelY, in.gaitCmd.angVelZ);
    }
    else {
        m_gcvInput = gcvBias;
    }

    // Check whether we should start walking
    if (m_walk && !m_walking) {
        resetWalking(true, gcvBias);
    }

    // Perform update tasks if walking is active
    if (m_walking) {
        updateRobot(gcvBias);
    }
    else if (m_blending) {
        m_blendPhase += config.gaitFrequency * M_PI * in.nominaldT;  // Increment the blend phase by the nominal gait
                                                                     // frequency (blend phase updates usually happen
                                                                     // inside updateRobot())
    }

    // Plot data
    if (m_PM.getEnabled) {
        m_PM.plotVec3d(m_gcv, PM_GCV);
        m_PM.plotVec3d(m_gcvAcc, PM_GCV_ACC);
        m_PM.plotScalar(m_gaitPhase, PM_GAIT_PHASE);
    }
}

// Update robot function
void GaitEngine::updateRobot(const Eigen::Vector3d& gcvBias) {
    // Note: Coming into this function we assume that we have in, m_walk and m_gcvInput available and up to date.
    //       Other class member variables such as m_gcv etc should naturally also be available and have their
    //       values from the last update (or other, if for example this is being called fresh from a reset).

    //
    // Preliminary work
    //

    // Increment the cycle number
    cycleNumber++;

    // Transcribe the gcv input
    Eigen::Vector3f gcvInput = m_gcvInput.cast<float>();

    // Calculate the current unbiased gait command vector
    Eigen::Vector3d gcvUnbiased = m_gcv - gcvBias;  // Should only be used for checking the proximity of gcv to a
                                                    // velocity command of zero (as the input to the gait engine sees
                                                    // it)

    // Retrieve the system iteration time and true time dT
    double systemIterationTime = in.nominaldT;
    double truedT              = in.truedT;
    stepTimeCount += truedT;

    // Retrieve the robot's fused angle
    const double fusedX = model->robotFRollPR() + config.mgFusedOffsetX;
    const double fusedY = model->robotFPitchPR() + config.mgFusedOffsetY;

    // Update the required input data for the limp models
    rxModel.updateInputData(systemIterationTime, fusedX, fusedY);
    mxModel.updateInputData(systemIterationTime, fusedX, fusedY);
    txModel.updateInputData(systemIterationTime, fusedX, fusedY);

    // Make local aliases of frequently used config variables
    const double C     = config.mgC;
    const double alpha = config.mgAlpha;  // Lateral step apex size.
    const double delta = config.mgDelta;  // Minimum lateral support exchange location.
    const double omega = config.mgOmega;  // Maximum lateral support exchange location.
    const double sigma = config.mgSigma;  // Maximum sagittal apex velocity.

    //
    // State estimation
    //

    // What we need is the com trajectory in the support foot frame, which is naturally discontinuous.
    // When a support exchange occurs, the com state has to be transformed into the new support frame.
    // This includes a rotation of the com velocity vector by the angle of the new support foot with
    // respect to the old support foot.

    // Retrieve the robot's current measured pose in terms of joint angles and populate a Pose struct with it
    // Note: We only need to populate the x, y, z fields for each joint, as this is all that our rxRobotModel needs.
    contrib::Pose measuredPose;
    measuredPose.headPose.neck.setPos(0.0, 0.0, 0.0);
    measuredPose.trunkPose.spine.setPos(0.0, 0.0, 0.0);
    measuredPose.leftArmPose.shoulder.setPos(
        in.jointPos[ServoID::L_SHOULDER_ROLL], in.jointPos[ServoID::L_SHOULDER_PITCH], 0.0);
    measuredPose.leftArmPose.elbow.setPos(0.0, in.jointPos[ServoID::L_ELBOW], 0.0);
    measuredPose.leftLegPose.hip.setPos(
        in.jointPos[ServoID::L_HIP_ROLL], in.jointPos[ServoID::L_HIP_PITCH], in.jointPos[ServoID::L_HIP_YAW]);
    measuredPose.leftLegPose.knee.setPos(0.0, in.jointPos[ServoID::L_KNEE], 0.0);
    measuredPose.leftLegPose.ankle.setPos(in.jointPos[ServoID::L_ANKLE_ROLL], in.jointPos[ServoID::L_ANKLE_PITCH], 0.0);
    measuredPose.rightArmPose.shoulder.setPos(
        in.jointPos[ServoID::R_SHOULDER_ROLL], in.jointPos[ServoID::R_SHOULDER_PITCH], 0.0);
    measuredPose.rightArmPose.elbow.setPos(0.0, in.jointPos[ServoID::R_ELBOW], 0.0);
    measuredPose.rightLegPose.hip.setPos(
        in.jointPos[ServoID::R_HIP_ROLL], in.jointPos[ServoID::R_HIP_PITCH], in.jointPos[ServoID::R_HIP_YAW]);
    measuredPose.rightLegPose.knee.setPos(0.0, in.jointPos[ServoID::R_KNEE], 0.0);
    measuredPose.rightLegPose.ankle.setPos(
        in.jointPos[ServoID::R_ANKLE_ROLL], in.jointPos[ServoID::R_ANKLE_PITCH], 0.0);

    // Update the rx robot model and filtered CoM state
    rxRobotModel.update(measuredPose, fusedX, fusedY);
    Eigen::Vector3d suppComVector = rxRobotModel.suppComVector();
    if (cycleNumber == 1) {
        m_comFilter.reset(systemIterationTime, suppComVector.x(), suppComVector.y(), 0.0, 0.0);
    }
    m_comFilter.update(suppComVector.x(), suppComVector.y());

    // If this is the first cycle then reset the odometry properly as we are now in pose
    if (cycleNumber == 1) {
        rxRobotModel.setOdom(0.0, 0.0, 0.0);
    }

    // Update the rx model timing
    rxModel.timeSinceStep += systemIterationTime;
    rxModel.nominalTimeToStep -= systemIterationTime;

    // Handle rx robot model support exchange
    if (rxRobotModel.supportExchange) {
        double supportOrientation = rxRobotModel.fusedYaw(rxRobotModel.suppFootstep.orientation());
        double angleFromLastToNewSupport =
            utility::math::angle::normalizeAngle(supportOrientation - lastSupportOrientation);
        lastSupportOrientation = supportOrientation;

        // To maintain continuous velocity in the world frame, rotate the CoM velocity vector into the new support
        // frame.
        Eigen::Matrix2f rot;
        // clang-format off
        // 2D rotation by (-angleFromLastToNewSupport)
        rot <<  std::cos(angleFromLastToNewSupport), std::sin(angleFromLastToNewSupport)
               -std::sin(angleFromLastToNewSupport), std::cos(angleFromLastToNewSupport);
        // clang-format on
        Eigen::Vector2f v(rxModel.vx, rxModel.vy);
        v = rot * v;

        m_comFilter.reset(systemIterationTime, suppComVector.x(), suppComVector.y(), v.x(), v.y());
        rxModel.timeSinceStep     = 0;
        rxModel.nominalTimeToStep = 2.0 * rxModel.nominalFootStepTHalf;
        stepCounter++;
        lastStepDuration = stepTimeCount;
        stepTimeCount    = 0.0;
    }

    // Add CoM offsets
    contrib::LimpState ms(m_comFilter.x(), m_comFilter.y(), m_comFilter.vx(), m_comFilter.vy(), 0.0, 0.0);
    ms.supportLegSign = rxRobotModel.supportLegSign;
    ms.x += config.mgComOffsetX;
    ms.y += ms.supportLegSign * config.mgComOffsetY + config.mgComOffsetYBias;

    // The LimpState ms concludes the estimation of the "CoM" state using direct sensor input and the kinematic model.
    // Let's set the rx limp model.
    rxModel.setState(ms, gcvInput);  // TODO: Should this be gcvInput or m_gcv?

    //
    // Adaptation gate
    //

    // A blending factor between 0 and 1 is used to blend between the rx state and a simulated ideal LIP state.
    // When the loop close factor is 1, the loop is closed completely and the model state (mx) will be
    // equal to the rx state. When the loop close factor is 0, sensor input is ignored and the
    // mx state is entirely driven by an ideal pendulum model. The factor interpolates between the sensor
    // input and an idealized pendulum model. We use this to inhibit adaptation near the steps.

    // Forward and update the mxModel
    mxModel.forwardThroughStep(systemIterationTime);
    contrib::LimpState mx = mxModel.getMotionState();
    contrib::LimpState rx = rxModel.getMotionState();
    contrib::LimpState ls = mx;
    double adaptx         = utility::math::clamp(0.0f, adaptation.x(), 1.0f);
    double adapty         = utility::math::clamp(0.0f, adaptation.y(), 1.0f);
    if (rxModel.supportLegSign * mxModel.supportLegSign > 0) {
        ls.x  = (1.0 - adaptx) * mx.x + adaptx * rx.x;
        ls.vx = (1.0 - adaptx) * mx.vx + adaptx * rx.vx;
        ls.y  = (1.0 - adapty) * mx.y + adapty * rx.y;
        ls.vy = (1.0 - adapty) * mx.vy + adapty * rx.vy;
    }
    mxModel.setState(ls, gcvInput);  // TODO: Should this be gcvInput or m_gcv?

    // Latency preview (beyond here we are working config.mgLatency seconds into the future with all our
    // calculations!)
    txModel        = mxModel;
    double latency = config.mgLatency;
    if (mxModel.timeToStep > latency) {
        txModel.forward(latency);
    }
    else {
        txModel.forward(mxModel.timeToStep);
        txModel.step();
        txModel.forward(latency - mxModel.timeToStep);
    }

    //
    // Noise suppression
    //

    // The step noise suppression factor is implemented as a Gaussian function. It's 0 near the support exchange
    // to inhibit problematic phases of state estimation. It needs to be computed after the step reset to avoid jumps.
    // Expectation adaptation. The idea is that we stay open loop when everything is as expected,
    // and only adapt to sensor input when it's far away from an expected value. It inhibits the adaptation rate a
    // little bit, but it has a nice smoothing effect. Nominal adaptation. With the nominal adaptation we can inhibit
    // adaptation when we are near the "optimal" nominal state.

    // Determine whether the loop should be closed
    bool loopClosed = config.cmdUseRXFeedback && (rxModel.supportLegSign == mxModel.supportLegSign);

    // Initialise the adaptation terms (Note: The initial value is an implicit scale of the entire adaptation term)
    double adaptationX = (loopClosed ? config.nsAdaptationGain : 0.0);
    double adaptationY = (loopClosed ? config.nsAdaptationGain : 0.0);

    // Calculate step noise suppression factor
    double phase = std::max(std::min(txModel.timeToStep, std::min(mxModel.timeToStep, mxModel.timeSinceStep)), 0.0);
    double stepNoiseSuppression =
        1.0 - std::exp(-(phase * phase) / (2.0 * config.nsStepNoiseTime * config.nsStepNoiseTime));
    adaptationX *= stepNoiseSuppression;
    adaptationY *= stepNoiseSuppression;

    // If we are near the expected state, there is no need for adaptation.
    Eigen::Vector2f expectationDeviation;
    expectationDeviation.x() =
        config.nsGain * (Eigen::Vector2f(rxModel.x, rxModel.vx) - Eigen::Vector2f(mxModel.x, mxModel.vx)).norm();
    expectationDeviation.y() =
        config.nsGain * (Eigen::Vector2f(rxModel.y, rxModel.vy) - Eigen::Vector2f(mxModel.y, mxModel.vy)).norm();
    adaptationX *= expectationDeviation.x();
    adaptationY *= expectationDeviation.y();

    //  // If our fused angle is within a certain nominal range then don't adapt
    //  Eigen::Vector2f fusedAngleAdaptation;
    //  fusedAngleAdaptation.x = ((fusedY <= config.nsFusedYRangeLBnd || fusedY >= config.nsFusedYRangeUBnd) ? 1.0 :
    //  0.0); fusedAngleAdaptation.y = ((fusedX <= config.nsFusedXRangeLBnd || fusedX >= config.nsFusedXRangeUBnd)
    //  ? 1.0 : 0.0); adaptationX *= fusedAngleAdaptation.x; adaptationY *= fusedAngleAdaptation.y;

    //  // If we are near the nominal state, there is no need for adaptation.
    //  Eigen::Vector2f nominalAdaptation;
    //  nominalAdaptation.x = (qAbs(rxModel.energyX - rxModel.nominalState.energyX) > config.nsMinDeviation ?
    //  config.nsGain * qAbs(rxModel.energyX - rxModel.nominalState.energyX) : 0.0); nominalAdaptation.y =
    //  (qAbs(rxModel.energyY - rxModel.nominalState.energyY) > config.nsMinDeviation ? config.nsGain *
    //  qAbs(rxModel.energyY - rxModel.nominalState.energyY) : 0.0);

    //  // If the fused angle is near zero, there is no need for adaptation.
    //  Eigen::Vector2f fusedAngleAdaptation;
    //  fusedAngleAdaptation.x = (absFusedY > config.nsMinDeviation ? config.nsGain * absFusedY : 0.0);
    //  fusedAngleAdaptation.y = (absFusedX > config.nsMinDeviation ? config.nsGain * absFusedX : 0.0);

    // Calculate the required adaptation (0.0 => Completely trust Mx, 1.0 => Completely trust Rx)
    adaptation.x() = utility::math::clamp(0.0, adaptationX, double(config.nsMaxAdaptation));
    adaptation.y() = utility::math::clamp(0.0, adaptationY, double(config.nsMaxAdaptation));

    // Adaptation resetting (force complete trust in Rx)
    if (resetCounter > 0) {
        adaptation.x() = 1.0;
        adaptation.y() = 1.0;
        resetCounter--;
    }

    //
    // Basic feedback mechanisms
    //

    // Calculate the fused angle deviation from expected
    double expectedFusedX = config.basicFusedExpXSinOffset
                            + config.basicFusedExpXSinMag * std::sin(m_gaitPhase - config.basicFusedExpXSinPhase);
    double expectedFusedY = config.basicFusedExpYSinOffset
                            + config.basicFusedExpYSinMag * std::sin(m_gaitPhase - config.basicFusedExpYSinPhase);
    double deviationFusedX = fusedX - expectedFusedX;
    double deviationFusedY = fusedY - expectedFusedY;

    // Calculate the gyro deviation from expected
    double expectedGyroX  = config.basicGyroExpX;
    double expectedGyroY  = config.basicGyroExpY;
    double deviationGyroX = model->robotAngularVelocity().x() - expectedGyroX;
    double deviationGyroY = model->robotAngularVelocity().y() - expectedGyroY;

    // Calculate basic fused angle deviation feedback values
    fusedXFeedFilter.put(deviationFusedX);
    fusedYFeedFilter.put(deviationFusedY);
    fusedXFeed = config.basicFusedGainAllLat
                 * utility::math::filter::SmoothDeadband::eval(fusedXFeedFilter.value(), config.basicFusedDeadRadiusX);
    fusedYFeed = config.basicFusedGainAllSag
                 * utility::math::filter::SmoothDeadband::eval(fusedYFeedFilter.value(), config.basicFusedDeadRadiusY);
    if (!config.basicFusedEnabledLat) {
        fusedXFeed = 0.0;
    }
    if (!config.basicFusedEnabledSag) {
        fusedYFeed = 0.0;
    }

    // Calculate basic fused angle deviation derivative feedback values
    // TODO: Modify the weighting based on gait phase to reject known fused angle bumps
    dFusedXFeedFilter.addXYW(in.timestamp, deviationFusedX, 1.0);
    // TODO: Modify the weighting based on gait phase to reject known fused angle bumps
    dFusedYFeedFilter.addXYW(in.timestamp, deviationFusedY, 1.0);
    dFusedXFeed = config.basicDFusedGainAllLat * utility::math::filter::SmoothDeadband::eval(
                                                     dFusedXFeedFilter.deriv(), config.basicDFusedDeadRadiusX);
    dFusedYFeed = config.basicDFusedGainAllSag * utility::math::filter::SmoothDeadband::eval(
                                                     dFusedYFeedFilter.deriv(), config.basicDFusedDeadRadiusY);
    if (!config.basicDFusedEnabledLat) {
        dFusedXFeed = 0.0;
    }
    if (!config.basicDFusedEnabledSag) {
        dFusedYFeed = 0.0;
    }

    // Update the half life time of the fused angle deviation integrators
    double halfLifeCycles = config.basicIFusedHalfLifeTime / systemIterationTime;
    iFusedXFeedIntegrator.setHalfLife(halfLifeCycles);
    iFusedYFeedIntegrator.setHalfLife(halfLifeCycles);

    // Integrate up the deviation in the fused angle X
    double timeSinceIFusedXFeedback = in.timestamp - iFusedXLastTime;
    if (timeSinceIFusedXFeedback < config.basicIFusedTimeToFreeze) {  // Integrate normally
        iFusedXFeedIntegrator.integrate(deviationFusedX);
    }
    else if (timeSinceIFusedXFeedback > config.basicIFusedTimeToDecay) {  // Decay the value of the integral to zero
                                                                          // (integral is exponentially weighted)
        iFusedXFeedIntegrator.integrate(0.0);
    }

    // Integrate up the deviation in the fused angle Y
    double timeSinceIFusedYFeedback = in.timestamp - iFusedYLastTime;
    if (timeSinceIFusedYFeedback < config.basicIFusedTimeToFreeze) {  // Integrate normally
        iFusedYFeedIntegrator.integrate(deviationFusedY);
    }
    else if (timeSinceIFusedYFeedback > config.basicIFusedTimeToDecay) {  // Decay the value of the integral to zero
                                                                          // (integral is exponentially weighted)
        iFusedYFeedIntegrator.integrate(0.0);
    }

    // Calculate a basic fused angle deviation integral feedback value
    iFusedXFeedFilter.put(iFusedXFeedIntegrator.integral());
    iFusedYFeedFilter.put(iFusedYFeedIntegrator.integral());
    iFusedXFeed = 1e-3 * config.basicIFusedGainAllLat * iFusedXFeedFilter.value();
    iFusedYFeed = 1e-3 * config.basicIFusedGainAllSag * iFusedYFeedFilter.value();
    if (!config.basicIFusedEnabledLat) {
        iFusedXFeed = 0.0;
    }
    if (!config.basicIFusedEnabledSag) {
        iFusedYFeed = 0.0;
    }
    haveIFusedXFeed = (config.basicIFusedGainAllLat != 0.0 && config.basicIFusedEnabledLat);
    haveIFusedYFeed = (config.basicIFusedGainAllSag != 0.0 && config.basicIFusedEnabledSag);

    // Calculate basic gyro deviation feedback values
    // TODO: Modify the weighting based on gait phase to reject known gyro bumps
    gyroXFeedFilter.addXYW(in.timestamp, deviationGyroX, 1.0);
    // TODO: Modify the weighting based on gait phase to reject known gyro bumps
    gyroYFeedFilter.addXYW(in.timestamp, deviationGyroY, 1.0);
    gyroXFeed = config.basicGyroGainAllLat
                * utility::math::filter::SmoothDeadband::eval(gyroXFeedFilter.value(), config.basicGyroDeadRadiusX);
    gyroYFeed = config.basicGyroGainAllSag
                * utility::math::filter::SmoothDeadband::eval(gyroYFeedFilter.value(), config.basicGyroDeadRadiusY);
    if (!config.basicGyroEnabledLat) {
        gyroXFeed = 0.0;
    }
    if (!config.basicGyroEnabledSag) {
        gyroYFeed = 0.0;
    }

    // Calculate a modification to the gait phase frequency based on basic feedback terms
    double timingFeedWeight = utility::math::clamp(
        -1.0, -config.basicTimingWeightFactor * sin(m_gaitPhase - 0.5 * config.doubleSupportPhaseLen), 1.0);
    double timingFeed = utility::math::filter::SmoothDeadband::eval(fusedXFeedFilter.value() * timingFeedWeight,
                                                                    config.basicTimingFeedDeadRad);
    double timingFreqDelta =
        (timingFeed >= 0.0 ? config.basicTimingGainSpeedUp * timingFeed : config.basicTimingGainSlowDown * timingFeed);
    if (!(config.basicEnableTiming && config.basicGlobalEnable)) {
        timingFreqDelta = 0.0;
    }

    // Virtual slope
    virtualSlope = 0.0;
    if (config.basicEnableVirtualSlope && config.basicGlobalEnable) {
        virtualSlope  = config.virtualSlopeOffset;
        double dev    = fusedY - config.virtualSlopeMidAngle;
        double absdev = fabs(dev);
        if (absdev > config.virtualSlopeMinAngle) {
            virtualSlope += utility::math::sign(dev)
                            * (dev * config.gcvPrescalerLinVelX * m_gcv.x() > 0 ? config.virtualSlopeGainAsc
                                                                                : config.virtualSlopeGainDsc)
                            * (absdev - config.virtualSlopeMinAngle);
        }
    }

    //
    // Gait phase update
    //

    // Save the value of the gait phase before the update
    double oldGaitPhase = m_gaitPhase;

    // Get the support leg sign according to the current gait phase
    int gaitPhaseLegSign = utility::math::sign(m_gaitPhase);

    // By default use the nominal gait frequency for timing (these variables tell us on what leg we should be standing,
    // for how much gait phase to come, traversing the gait phase with what frequency)
    int supportLegSign        = gaitPhaseLegSign;
    double gaitFrequency      = config.gaitFrequency;
    double remainingGaitPhase = (m_gaitPhase > 0.0 ? M_PI - m_gaitPhase : 0.0 - m_gaitPhase);
    if (remainingGaitPhase < 1e-8) {
        remainingGaitPhase = 1e-8;
    }

    // Handle the case of closed loop timing feedback
    if (config.cmdUseCLTiming) {
        // If desired, use the TX model timing
        if (config.cmdUseTXTiming) {
            supportLegSign = txModel.supportLegSign;
            if (supportLegSign != gaitPhaseLegSign) {  // Premature step case - happens even to the best of us...
                remainingGaitPhase += M_PI;  // The txModel.timeToStep variable tells us how much time we should take
                                             // until the end of the step *after* the one we're currently taking, so
                                             // that's an extra PI radians to cover
            }
            if (txModel.timeToStep > 1e-8) {
                gaitFrequency = remainingGaitPhase / (M_PI * txModel.timeToStep);  // Note: Always in the range (0,Inf)
            }
            else {
                gaitFrequency = 1e8;  // Big enough to max out the gait frequency in any case
            }
        }

        // If desired, add basic timing feedback
        if (config.basicEnableTiming && config.basicGlobalEnable) {
            gaitFrequency += timingFreqDelta;
        }
    }

    // Coerce the gait frequency to the allowed range
    gaitFrequency = utility::math::clamp(1e-8, gaitFrequency, double(config.gaitFrequencyMax));

    // Calculate the time to step based on how much gait phase we have to cover and how fast we intend to cover it
    double timeToStep = remainingGaitPhase / (M_PI * gaitFrequency);  // Note: Always in the range (0,Inf)

    // Update the gait phase with the appropriate phase increment
    double gaitPhaseIncrement = gaitFrequency * M_PI * systemIterationTime;
    // The gait phase must be in the range (-pi,pi], and thus must be wrapped!
    m_gaitPhase = utility::math::angle::normalizeAngle(m_gaitPhase + gaitPhaseIncrement);

    // Increment the blending phase if we are in the process of blending
    if (m_blending) {
        m_blendPhase += gaitPhaseIncrement;
    }

    //
    // Gait command vector update
    //

    // By default use a nominal CL step size (this should approximately emulate walking OL with zero GCV)
    Eigen::Vector3f stepSize(0.0, 2.0 * supportLegSign * config.hipWidth, 0.0);

    // If desired, use the TX model step size instead
    if (config.cmdUseTXStepSize) {
        stepSize = txModel.stepSize;
    }

    // Clamp the step size to a maximal radius
    stepSize.x() = utility::math::clamp(-config.mgMaxStepRadiusX, stepSize.x(), config.mgMaxStepRadiusX);
    stepSize.y() = utility::math::clamp(-config.mgMaxStepRadiusY, stepSize.y(), config.mgMaxStepRadiusY);

    // Step size to GCV conversion.
    // To dodge the fact that the gait engine can only take symmetrical steps, while the limp model may produce
    // asymmetrical steps, we use the center point between the feet as the sex and map the sex position between alpha,
    // delta and omega to the gcv.
    Eigen::Vector3d gcvTarget;
    Eigen::Vector3f sex = 0.5 * stepSize;
    limp.set(0.0, sigma, C);
    limp.update(contrib::Limp(alpha, 0, C).tLoc(delta));
    gcvTarget.x() = sex.x() / limp.x0;
    if (supportLegSign * gcvInput.y() >= 0) {
        gcvTarget.y() = supportLegSign * utility::math::clamp(-1.0, (std::abs(sex.y()) - delta) / (omega - delta), 1.0);
    }
    else {
        gcvTarget.y() = oldGcvTargetY;  // TODO: This looks relatively unsafe
    }
    gcvTarget.z() = sex.z();
    oldGcvTargetY = gcvTarget.y();
    if (!config.cmdAllowCLStepSizeX) {
        gcvTarget.x() = 0.0;
    }

    // Move the current gcv smoothly to the target gcv in timeToStep amount of time
    Eigen::Vector3d CLGcv = gcvTarget;

    // TODO: Build some kind of protection into this update strategy to avoid increasingly explosive GCV updates
    // super-close to the end of the step, causing sharp joint curves.
    if (timeToStep > systemIterationTime) {
        CLGcv = m_gcv + (systemIterationTime / timeToStep) * (gcvTarget - m_gcv);
    }

    // Handle the situation differently depending on whether we are using CL step sizes or not
    // Closed loop step sizes...
    if (config.cmdUseCLStepSize) {
        // Set the calculated closed loop GCV
        if (noCLStepsCounter > 0) {
            m_gcv.setZero();
            noCLStepsCounter--;
        }
        else {
            if (config.cmdAllowCLStepSizeX) {
                m_gcv.x() = CLGcv.x();
            }
            else {
                double D = config.gcvDecToAccRatio;
                if (gcvUnbiased.x() >= 0.0) {
                    m_gcv.x() += utility::math::clamp(-truedT * config.gcvAccForwards * D,
                                                      m_gcvInput.x() - m_gcv.x(),
                                                      truedT * config.gcvAccForwards);
                }
                else {
                    m_gcv.x() += utility::math::clamp(-truedT * config.gcvAccBackwards,
                                                      m_gcvInput.x() - m_gcv.x(),
                                                      truedT * config.gcvAccBackwards * D);
                }
            }
            m_gcv.y() = CLGcv.y();
            m_gcv.z() = CLGcv.z();
        }
    }
    // Open loop step sizes...
    else {
        // Zero out the step size to show that we are OL
        stepSize = Eigen::Vector3f::Zero();

        // Update the internal gcv based on the gcv input using a slope-limiting approach
        double D = config.gcvDecToAccRatio;
        if (gcvUnbiased.x() >= 0.0) {
            m_gcv.x() += utility::math::clamp(
                -truedT * config.gcvAccForwards * D, m_gcvInput.x() - m_gcv.x(), truedT * config.gcvAccForwards);
        }
        else {
            m_gcv.x() += utility::math::clamp(
                -truedT * config.gcvAccBackwards, m_gcvInput.x() - m_gcv.x(), truedT * config.gcvAccBackwards * D);
        }
        if (gcvUnbiased.y() >= 0.0) {
            m_gcv.y() += utility::math::clamp(
                -truedT * config.gcvAccSidewards * D, m_gcvInput.y() - m_gcv.y(), truedT * config.gcvAccSidewards);
        }
        else {
            m_gcv.y() += utility::math::clamp(
                -truedT * config.gcvAccSidewards, m_gcvInput.y() - m_gcv.y(), truedT * config.gcvAccSidewards * D);
        }
        if (gcvUnbiased.z() >= 0.0) {
            m_gcv.z() += utility::math::clamp(
                -truedT * config.gcvAccRotational * D, m_gcvInput.z() - m_gcv.z(), truedT * config.gcvAccRotational);
        }
        else {
            m_gcv.z() += utility::math::clamp(
                -truedT * config.gcvAccRotational, m_gcvInput.z() - m_gcv.z(), truedT * config.gcvAccRotational * D);
        }
    }

    // Calculate and filter the gait command acceleration
    m_gcvDeriv.put(m_gcv);
    Eigen::Vector3d gcvAccRaw = m_gcvDeriv.value() / systemIterationTime;
    m_gcvAccSmoothX.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.x(), m_gcvAcc.x(), config.gcvAccJerkLimitX * systemIterationTime));
    m_gcvAccSmoothY.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.y(), m_gcvAcc.y(), config.gcvAccJerkLimitY * systemIterationTime));
    m_gcvAccSmoothZ.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.z(), m_gcvAcc.z(), config.gcvAccJerkLimitZ * systemIterationTime));
    m_gcvAcc.x() = m_gcvAccSmoothX.value();
    m_gcvAcc.y() = m_gcvAccSmoothY.value();
    m_gcvAcc.z() = m_gcvAccSmoothZ.value();

    //
    // Motion stance control
    //

    // Calculate the current target motion stance factor
    double targetLegAngleXFact = 1.0;
    bool haveMotionStance      = (config.enableMotionStances && in.motionPending);
    if (haveMotionStance) {
        switch (in.motionStance) {
            case STANCE_DEFAULT: targetLegAngleXFact = 1.0; break;
            case STANCE_KICK: targetLegAngleXFact    = 0.0; break;
            default: targetLegAngleXFact             = 1.0; break;
        }
    }

    // Update the motion stance factors, taking into consideration which legs we are allowed to adjust
    if (!haveMotionStance
        || (gcvUnbiased.norm() <= config.stanceAdjustGcvMax
            && ((in.motionAdjustLeftFoot && rxRobotModel.supportLegSign == contrib::RobotModel::RIGHT_LEG)
                || (in.motionAdjustRightFoot && rxRobotModel.supportLegSign == contrib::RobotModel::LEFT_LEG)
                || (!in.motionAdjustLeftFoot && !in.motionAdjustRightFoot)))) {
        m_motionLegAngleXFact = utility::math::filter::SlopeLimiter::eval(
            targetLegAngleXFact, m_motionLegAngleXFact, config.stanceAdjustRate * systemIterationTime);
    }

    // Determine whether the motion stance adjustment is still ongoing
    bool motionStanceOngoing = (fabs(m_motionLegAngleXFact - targetLegAngleXFact) > 1e-6);

    //
    // Walking control
    //

    // Check whether we should stop walking
    double nominalGaitPhaseInc = M_PI * systemIterationTime * config.gaitFrequency;
    double LB = nominalGaitPhaseInc * config.stoppingPhaseTolLB;  // Lower bound tolerance config variable is in the
                                                                  // units of nominal phase increments, calculated
                                                                  // using nominalStepTime (also a config variable)
    double UB = nominalGaitPhaseInc * config.stoppingPhaseTolUB;  // Upper bound tolerance config variable is in the
                                                                  // units of nominal phase increments, calculated
                                                                  // using nominalStepTime (also a config variable)
    if (!m_walk && gcvUnbiased.norm() <= config.stoppingGcvMag
        &&  // Note that this is intentionally checking the gcv (unbiased) from the last cycle, not the new and
        // unexecuted modified one from the current call to this update function
        ((m_gaitPhase >= 0.0 && oldGaitPhase <= 0.0 && (m_gaitPhase <= UB || oldGaitPhase >= -LB))
         || (m_gaitPhase <= 0.0 && oldGaitPhase >= 0.0 && (m_gaitPhase <= UB - M_PI || oldGaitPhase >= M_PI - LB)))
        && (!motionStanceOngoing)) {
        resetWalking(false, gcvBias);
    }

    //
    // Plotting
    //

    // Plot update function variables
    if (m_PM.getEnabled()) {
        Eigen::Vector3d tmp;
        tmp = rxRobotModel.suppComVector();
        m_PM.plotScalar(tmp.x, PM_RXRMODEL_SUPPVEC_X);
        m_PM.plotScalar(tmp.y, PM_RXRMODEL_SUPPVEC_Y);
        m_PM.plotScalar(tmp.z, PM_RXRMODEL_SUPPVEC_Z);
        tmp = rxRobotModel.suppStepVector();
        m_PM.plotScalar(tmp.x, PM_RXRMODEL_STEPVEC_X);
        m_PM.plotScalar(tmp.y, PM_RXRMODEL_STEPVEC_Y);
        m_PM.plotScalar(tmp.z, PM_RXRMODEL_STEPVEC_Z);
        m_PM.plotScalar(rxRobotModel.suppStepYaw(), PM_RXRMODEL_STEPVEC_FYAW);
        m_PM.plotScalar(fusedX, PM_FUSED_X);
        m_PM.plotScalar(fusedY, PM_FUSED_Y);
        m_PM.plotScalar(m_comFilter.x(), PM_COMFILTER_X);
        m_PM.plotScalar(m_comFilter.y(), PM_COMFILTER_Y);
        m_PM.plotScalar(m_comFilter.vx(), PM_COMFILTER_VX);
        m_PM.plotScalar(m_comFilter.vy(), PM_COMFILTER_VY);
        m_PM.plotScalar(rxModel.x, PM_RXMODEL_X);
        m_PM.plotScalar(rxModel.y, PM_RXMODEL_Y);
        m_PM.plotScalar(rxModel.vx, PM_RXMODEL_VX);
        m_PM.plotScalar(rxModel.vy, PM_RXMODEL_VY);
        m_PM.plotScalar(rxModel.supportLegSign, PM_RXMODEL_SUPPLEG);
        m_PM.plotScalar(rxModel.timeToStep, PM_RXMODEL_TIMETOSTEP);
        m_PM.plotScalar(mxModel.x, PM_MXMODEL_X);
        m_PM.plotScalar(mxModel.y, PM_MXMODEL_Y);
        m_PM.plotScalar(mxModel.vx, PM_MXMODEL_VX);
        m_PM.plotScalar(mxModel.vy, PM_MXMODEL_VY);
        m_PM.plotScalar(mxModel.supportLegSign, PM_MXMODEL_SUPPLEG);
        m_PM.plotScalar(mxModel.timeToStep, PM_MXMODEL_TIMETOSTEP);
        m_PM.plotScalar(mxModel.zmp.x, PM_MXMODEL_ZMP_X);
        m_PM.plotScalar(mxModel.zmp.y, PM_MXMODEL_ZMP_Y);
        m_PM.plotScalar(txModel.x, PM_TXMODEL_X);
        m_PM.plotScalar(txModel.y, PM_TXMODEL_Y);
        m_PM.plotScalar(txModel.vx, PM_TXMODEL_VX);
        m_PM.plotScalar(txModel.vy, PM_TXMODEL_VY);
        m_PM.plotScalar(txModel.supportLegSign, PM_TXMODEL_SUPPLEG);
        m_PM.plotScalar(txModel.timeToStep, PM_TXMODEL_TIMETOSTEP);
        m_PM.plotScalar(txModel.stepSize.x, PM_TXMODEL_STEPSIZEX);
        m_PM.plotScalar(txModel.stepSize.y, PM_TXMODEL_STEPSIZEY);
        m_PM.plotScalar(txModel.stepSize.z, PM_TXMODEL_STEPSIZEZ);
        m_PM.plotScalar(adaptx, PM_ADAPTATION_X);
        m_PM.plotScalar(adapty, PM_ADAPTATION_Y);
        m_PM.plotScalar(expectedFusedX, PM_EXP_FUSED_X);
        m_PM.plotScalar(expectedFusedY, PM_EXP_FUSED_Y);
        m_PM.plotScalar(deviationFusedX, PM_DEV_FUSED_X);
        m_PM.plotScalar(deviationFusedY, PM_DEV_FUSED_Y);
        m_PM.plotScalar(fusedXFeed, PM_FEEDBACK_FUSED_X);
        m_PM.plotScalar(fusedYFeed, PM_FEEDBACK_FUSED_Y);
        m_PM.plotScalar(dFusedXFeed, PM_FEEDBACK_DFUSED_X);
        m_PM.plotScalar(dFusedYFeed, PM_FEEDBACK_DFUSED_Y);
        m_PM.plotScalar(iFusedXFeed, PM_FEEDBACK_IFUSED_X);
        m_PM.plotScalar(iFusedYFeed, PM_FEEDBACK_IFUSED_Y);
        m_PM.plotScalar(gyroXFeed, PM_FEEDBACK_GYRO_X);
        m_PM.plotScalar(gyroYFeed, PM_FEEDBACK_GYRO_Y);
        m_PM.plotScalar(timingFeedWeight, PM_TIMING_FEED_WEIGHT);
        m_PM.plotScalar(timingFreqDelta, PM_TIMING_FREQ_DELTA);
        m_PM.plotScalar(gaitFrequency, PM_GAIT_FREQUENCY);
        m_PM.plotScalar(remainingGaitPhase, PM_REM_GAIT_PHASE);
        m_PM.plotScalar(timeToStep, PM_TIMETOSTEP);
        m_PM.plotScalar(stepSize.x, PM_STEPSIZE_X);
        m_PM.plotScalar(stepSize.y, PM_STEPSIZE_Y);
        m_PM.plotScalar(stepSize.z, PM_STEPSIZE_Z);
        m_PM.plotScalar(gcvTarget.x(), PM_GCVTARGET_X);
        m_PM.plotScalar(gcvTarget.y(), PM_GCVTARGET_Y);
        m_PM.plotScalar(gcvTarget.z(), PM_GCVTARGET_Z);
        m_PM.plotScalar(lastStepDuration, PM_LAST_STEP_DURATION);
    }
}


// Calculate common motion data
GaitEngine::CommonMotionData GaitEngine::calcCommonMotionData(bool isFirst) const {
    // Declare variables
    CommonMotionData CMD;

    // Set the gcv and absolute gcv
    CMD.gcvX = config.gcvPrescalerLinVelX * m_gcv.x();  // The raw commanded gcv x prescaled by a preconfigured scaler
                                                        // to adjust the dynamic range of the gait for inputs in the
                                                        // range [0,1]
    CMD.gcvY = config.gcvPrescalerLinVelY * m_gcv.y();  // The raw commanded gcv y prescaled by a preconfigured scaler
                                                        // to adjust the dynamic range of the gait for inputs in the
                                                        // range [0,1]
    CMD.gcvZ = config.gcvPrescalerAngVelZ * m_gcv.z();  // The raw commanded gcv z prescaled by a preconfigured scaler
                                                        // to adjust the dynamic range of the gait for inputs in the
                                                        // range [0,1]
    CMD.absGcvX = fabs(CMD.gcvX);                       // Absolute commanded gcv x
    CMD.absGcvY = fabs(CMD.gcvY);                       // Absolute commanded gcv y
    CMD.absGcvZ = fabs(CMD.gcvZ);                       // Absolute commanded gcv z

    // Set the gait phase variables
    CMD.gaitPhase = m_gaitPhase;  // gaitPhase: Starts at 0 at the beginning of walking, and has the swing phase of the
                                  // first leg in [0,pi] (left or right depending on m_leftLegFirst)
    CMD.oppGaitPhase = utility::math::angle::normalizeAngle(
        CMD.gaitPhase + M_PI);  // oppGaitPhase: Is in exact antiphase to gaitPhase, and has the
                                // support phase of the first leg in [0,pi] (left or right
                                // depending on m_leftLegFirst)
    CMD.limbPhase = (isFirst ? CMD.gaitPhase : CMD.oppGaitPhase);  // limbPhase: Is in exact phase or antiphase to
                                                                   // m_gaitPhase, and always has the swing phase of
                                                                   // 'this' limb in [0,pi]
    CMD.absPhase = (m_leftLegFirst ? CMD.gaitPhase : CMD.oppGaitPhase);  // absPhase: Is in exact phase or antiphase to
                                                                         // m_gaitPhase, and always has the swing phase
                                                                         // of the left leg in [0,pi]

    // Set the first collection of phase marks
    CMD.doubleSupportPhase = config.doubleSupportPhaseLen;  // The length of the double support phase (from gait phase
                                                            // zero to this value), in the range [0,pi/2]
    CMD.swingStartPhase =
        CMD.doubleSupportPhase
        + config.swingStartPhaseOffset;  // Limb phase mark at which the swing starts, in the range [0,pi]
    CMD.swingStopPhase =
        M_PI - config.swingStopPhaseOffset;  // Limb phase mark at which the swing stops, in the range [0,pi]

    // Check for violation of the minimum allowed swing phase length and automatically rescale the swing phase offsets
    // if required
    if (CMD.swingStopPhase - CMD.swingStartPhase < config.swingMinPhaseLen)  // The config parameter swingMinPhaseLen
                                                                             // specifies the minimum allowed value of
                                                                             // sinusoidPhaseLen, in the range [0.1,1]
    {
        double resize =
            (M_PI - CMD.doubleSupportPhase - config.swingMinPhaseLen)
            / (config.swingStartPhaseOffset + config.swingStopPhaseOffset);  // The denominator of this should never
                                                                             // be <= 0 due to the permitted ranges
                                                                             // on the various config variables
        CMD.swingStartPhase = CMD.doubleSupportPhase + resize * config.swingStartPhaseOffset;
        CMD.swingStopPhase  = M_PI - resize * config.swingStopPhaseOffset;
        NUClear::log(3.0,
                     "Invalid configuration for cap_gait swing phase (in violation of minimum swing phase length) "
                     "=> Automatically fixing");
    }

    // Set the remaining phase marks
    CMD.suppTransStartPhase =
        -config.suppTransStartRatio * (M_PI - CMD.swingStopPhase);  // Phase mark at which the support transition
                                                                    // starts (small negative number, i.e. backwards
                                                                    // from zero/pi, the ratio interpolates between
                                                                    // the beginning of the double support phase
                                                                    // (zero) and the first negative swing stop phase)
    CMD.suppTransStopPhase =
        CMD.doubleSupportPhase
        + config.suppTransStopRatio
              * (CMD.swingStartPhase - CMD.doubleSupportPhase);  // Phase mark at which the support transition stops, in
                                                                 // the range [0,pi] (the ratio interpolates between the
                                                                 // end of the double support phase and the first
                                                                 // positive swing start phase)

    // Calculate the extra swing variables
    CMD.liftingPhaseLen = M_PI - CMD.doubleSupportPhase;  // Length of each lifting (single support) phase
    CMD.suppPhaseLen = M_PI - CMD.suppTransStartPhase + CMD.suppTransStopPhase;  // Length of the support phase (phase
                                                                                 // length for which a support
                                                                                 // coefficient is non-zero at a time)
    CMD.nonSuppPhaseLen = (2.0 * M_PI) - CMD.suppPhaseLen;  // Length of the non-support phase (phase length for which a
                                                            // support coefficient is zero at a time)
    CMD.sinusoidPhaseLen = CMD.swingStopPhase - CMD.swingStartPhase;  // Length of the sinusoidal forwards swing phase
    CMD.linearPhaseLen   = (2.0 * M_PI) - CMD.sinusoidPhaseLen;       // Length of the linear backwards swing phase

    // Calculate the gait phase dependent dimensionless swing angle (ranging from -1 to 1)
    if (CMD.limbPhase >= CMD.swingStartPhase && CMD.limbPhase <= CMD.swingStopPhase) {
        CMD.swingAngle = -cos(M_PI * (CMD.limbPhase - CMD.swingStartPhase)
                              / CMD.sinusoidPhaseLen);  // Sinusoid forwards swing from dimensionless angle -1 to +1
                                                        // (section from phase swingStartPhase to phase swingStopPhase)
    }
    else if (CMD.limbPhase > CMD.swingStopPhase) {
        CMD.swingAngle =
            1.0 - (2.0 / CMD.linearPhaseLen) * (CMD.limbPhase - CMD.swingStopPhase);  // Linear backwards swing from
                                                                                      // dimensionless angle +1 to a
                                                                                      // mid-way point C (section from
                                                                                      // phase swingStopPhase to phase
                                                                                      // pi)
    }
    else {
        CMD.swingAngle = 1.0
                         - (2.0 / CMD.linearPhaseLen)
                               * (CMD.limbPhase - CMD.swingStopPhase + (2.0 * M_PI));  // Linear backwards swing
                                                                                       // from dimensionless
                                                                                       // angle C to -1 (section
                                                                                       // from phase -pi to phase
                                                                                       // swingStartPhase)
    }

    // Return the common motion data
    return CMD;
}

// Generate the abstract leg motion
// 'leg' is assumed to contain the desired leg halt pose
void GaitEngine::abstractLegMotion(pose::AbstractLegPose& leg) {
    //
    // Common motion data
    //

    // The legs swing about all three axes to achieve the required walking velocity. They do this with a special timing
    // and waveform, summarised by the dimensionless swing angle waveform template. This template consists of a
    // sinusoidal 'forwards' motion, followed by a linear 'backwards' motion. The forwards motion is referred to as the
    // swing phase of the corresponding limb, while the backwards motion is referred to as the support phase. This
    // alludes to the expected behaviour that the foot of a particular leg should be in contact with the ground during
    // that leg's support phase, and in the air during its swing phase. The ratio of swing time to support time in a
    // human gait is typically 40 to 60, and so the swing phase is made shorter. The exact timing of the swing phase is
    // defined by the swingStartPhaseOffset, swingStopPhaseOffset and doubleSupportPhaseLen configuration variables. It
    // is expected that the foot achieves toe-off at approximately the swing start phase, and heel-strike at
    // approximately the swing stop phase. The asymmetry of the two phases in combination with the explicit double
    // support phase leads to a brief-ish phase where both legs are expected to be in contact with the ground
    // simultaneously, performing their backwards swing.

    // Calculate the common motion data
    CommonMotionData CMD = calcCommonMotionData(leg.cld.isLeft == m_leftLegFirst);

    // Disable the leg swing if required
    if (config.tuningNoLegSwing) {
        CMD.swingAngle = 0.0;
    }

    // Initialise the additive hip angles
    double hipAngleX = 0.0;
    double hipAngleY = 0.0;

    //
    // Leg lifting (extension, angleY)
    //

    // The leg is alternately lifted off the ground (step) and pushed down into it (push), with a short break in-between
    // given by the double support phase. The double support phases start at limb phase 0 and pi, and the leg
    // lifting/pushing phases start immediately after the double support phases end, respectively. The lifting/pushing
    // phases always end at limb phases of pi and 0, respectively. The leg push phase, if enabled, assists with
    // achieving foot clearance of the swing foot.

    // Apply leg stepping and pushing to the abstract leg pose
    double legExtensionOffset = 0.0;
    if (!config.tuningNoLegLifting) {
        // Calculate the desired step and push height from the current gcv
        double stepHeight =
            config.legStepHeight + CMD.absGcvX * config.legStepHeightGradX + CMD.absGcvY * config.legStepHeightGradY;
        double pushHeight = config.legPushHeight + CMD.absGcvX * config.legPushHeightGradX;

        // Precalculate parameters and phases for the following calculation
        double sinAngFreq     = M_PI / CMD.liftingPhaseLen;
        double stepStartPhase = CMD.limbPhase - CMD.doubleSupportPhase;
        double stepStopPhase  = -utility::math::angle::normalizeAngle(CMD.limbPhase - M_PI);
        double pushStartPhase = stepStartPhase + M_PI;
        double pushStopPhase  = -CMD.limbPhase;

        // Calculate the basic sinusoid step and push waveforms
        if (stepStartPhase >= 0.0) {
            legExtensionOffset =
                stepHeight * sin(sinAngFreq * stepStartPhase);  // Leg stepping phase => Limb phase in the positive half
        }
        else if (pushStartPhase >= 0.0 && CMD.limbPhase <= 0.0) {
            legExtensionOffset =
                -pushHeight * sin(sinAngFreq * pushStartPhase);  // Leg pushing phase => Limb phase in the negative half
        }
        else {
            legExtensionOffset = 0.0;  // Double support phase
        }

        // Add fillets to the waveforms to avoid overly large acceleration/torque jumps
        legExtensionOffset += utility::math::filter::LinSinFillet::eval(
            stepStartPhase, stepHeight, sinAngFreq, 0.5 * config.filletStepPhaseLen, config.doubleSupportPhaseLen);
        legExtensionOffset += utility::math::filter::LinSinFillet::eval(
            stepStopPhase, stepHeight, sinAngFreq, 0.5 * config.filletStepPhaseLen, config.doubleSupportPhaseLen);
        legExtensionOffset += utility::math::filter::LinSinFillet::eval(
            pushStartPhase, -pushHeight, sinAngFreq, 0.5 * config.filletPushPhaseLen, config.doubleSupportPhaseLen);
        legExtensionOffset += utility::math::filter::LinSinFillet::eval(
            pushStopPhase, -pushHeight, sinAngFreq, 0.5 * config.filletPushPhaseLen, config.doubleSupportPhaseLen);

        // Update the leg extension
        leg.extension += legExtensionOffset;
        leg.angleY += config.legExtToAngleYGain * legExtensionOffset;  // This trims the lift angle of the feet, which
                                                                       // can be used to trim walking on the spot, but
                                                                       // this also has very positive effects on the
                                                                       // remainder of OL walking
    }

    //
    // Sagittal leg swing (angleY)
    //

    // Apply the sagittal leg swing to the abstract leg pose (responsible for fulfilling the gcv x-velocity)
    // We use a different sagittal leg swing gradient depending on whether we're walking forwards or backwards.
    double legSagSwingMag =
        CMD.gcvX * (CMD.gcvX >= 0.0 ? config.legSagSwingMagGradXFwd : config.legSagSwingMagGradXBwd);
    double legSagSwing = -CMD.swingAngle * legSagSwingMag;
    leg.angleY += legSagSwing;

    //
    // Lateral leg pushout, lateral leg swing and lateral hip swing (angleX)
    //

    // Apply the lateral leg pushout to the abstract leg pose
    // This rolls the legs outwards (from the hip) in proportion to each of the absolute gcv values. This acts to
    // separate the feet more the faster the robot is walking, and the more the robot is trying to walk with combined
    // velocity components (e.g. forwards velocity coupled with a rotational velocity). This term seeks to prevent foot
    // to foot self-collisions, and should more be seen as a bias to the halt pose, as opposed to a dynamic motion
    // component of the gait. This is because for constant walking velocity command, this term is constant.
    double legLatPushoutMag = CMD.absGcvX * config.legLatPushoutMagGradX + CMD.absGcvY * config.legLatPushoutMagGradY
                              + CMD.absGcvZ * config.legLatPushoutMagGradZ;
    if (!config.tuningNoLegPushout) {
        leg.angleX += leg.cld.limbSign * legLatPushoutMag;
    }

    // Apply the lateral leg swing to the abstract leg pose
    // This is responsible for fulfilling the gcv y-velocity.
    double legLatSwingMag = CMD.gcvY * config.legLatSwingMagGradY;
    double legLatSwing    = CMD.swingAngle * legLatSwingMag;
    leg.angleX += legLatSwing;

    // The lateral hip swing motion component sways the hips left and right relative to the feet. This is achieved via
    // direct addition of a hip swing term to the hip roll (i.e. angleX). Hip swing to the left occurs when the left
    // foot is in its support phase (i.e. absolute phase in (-pi,0]), and should correspond to negative roll. Hip swing
    // to the right occurs when the right foot is in its support phase (i.e. absolute phase in (0,pi]), and should
    // correspond to positive roll. The hip swing is applied equally to both hip roll joints, and is calculated as the
    // sum of two overlapping sinusoid 'halves', one for the hip swing to the left, and one for the hip swing to the
    // right. The overlap between the sinusoids occurs during the support transition phase, during which time the
    // sinusoids sum up, and the hip travels from one body side to the other. To be totally explicit, the left hip swing
    // sinusoid completes exactly half a sinusoid from the beginning of the left leg support transition to the end of
    // the next right leg support transition, and is zero everywhere else. The right hip swing sinusoid behaves
    // similarly, and they are added up to get the full dimensionless hip swing waveform. Recall that the absolute phase
    // is in phase or antiphase to the gait phase such that the swing phase of the left leg is in [0,pi].

    // Perform phase calculations for the left and right halves (sinusoids) of the hip swing motion
    double hipSwingStartL = CMD.suppTransStartPhase + M_PI;  // The start phase of the period of support of the left leg
    double hipSwingStartR = CMD.suppTransStartPhase;  // The start phase of the period of support of the right leg
    double hipSwingPhaseL = (CMD.absPhase < hipSwingStartL ? CMD.absPhase + (2.0 * M_PI) : CMD.absPhase)
                            - hipSwingStartL;  // The current phase relative to the start of the transition to the left
                                               // leg as the support leg, in the range [0,2*pi]
    double hipSwingPhaseR = (CMD.absPhase < hipSwingStartR ? CMD.absPhase + (2.0 * M_PI) : CMD.absPhase)
                            - hipSwingStartR;  // The current phase relative to the start of the transition to the right
                                               // leg as the support leg, in the range [0,2*pi]

    // Calculate the dimensionless hip swing angle (range -1 to 1) by summing up the two zero-clamped sinusoid
    // sub-waveforms
    double hipSwingAngleL = -std::max(std::sin(M_PI * hipSwingPhaseL / CMD.suppPhaseLen), 0.0);
    double hipSwingAngleR = std::max(std::sin(M_PI * hipSwingPhaseR / CMD.suppPhaseLen), 0.0);
    double hipSwingAngle  = hipSwingAngleL + hipSwingAngleR;  // The hip swing angle is in the range -1 to 1

    // Apply the lateral hip swing to the abstract leg pose
    double legLatHipSwingMag = config.legLatHipSwingMag + CMD.absGcvX * config.legLatHipSwingMagGradX
                               + CMD.absGcvY * config.legLatHipSwingMagGradY;
    double legLatHipSwing = config.legLatHipSwingBias + legLatHipSwingMag * hipSwingAngle;
    if (!config.tuningNoLegHipSwing) {
        leg.angleX += legLatHipSwing;
    }

    //
    // Rotational leg V pushout and rotational leg swing (angleZ)
    //

    // Apply the rotational leg V pushout to the abstract leg pose
    // Offset the yaw of the feet to be in a tendentially more toe-out configuration the faster we are expecting the
    // robot to turn. This is referred to as rotational V pushout, and has the effect of attempting to avoid toe to toe
    // self-collisions. For a constant walking velocity command, this term is constant, and hence should be considered
    // to be more of a bias to the halt pose, rather than a dynamic motion component of the gait.
    double legRotVPushoutMag = CMD.absGcvZ * config.legRotVPushoutMagGradZ;
    if (!config.tuningNoLegPushout) {
        leg.angleZ += leg.cld.limbSign * legRotVPushoutMag;
    }

    // Apply the rotational leg swing to the abstract leg pose
    // This is responsible for fulfilling the gcv z-velocity.
    double legRotSwingMag = CMD.gcvZ * config.legRotSwingMagGradZ;
    double legRotSwing    = CMD.swingAngle * legRotSwingMag;
    leg.angleZ += legRotSwing;

    //
    // Leaning (hipAngleX, hipAngleY)
    //

    // Apply the leaning to the abstract leg pose
    // When walking at higher speeds, it is advantageous to make use of leaning to assist with balance. Velocity-based
    // leaning is chosen over acceleration-based leaning in the interest of producing continuous abstract pose outputs.
    double legSagLean = 0.0;
    double legLatLean = 0.0;
    if (!config.tuningNoLegLeaning) {
        // Lean forwards and backwards based on the sagittal walking velocity
        legSagLean = CMD.gcvX * (CMD.gcvX >= 0.0 ? config.legSagLeanGradVelXFwd : config.legSagLeanGradVelXBwd)
                     + CMD.absGcvZ * config.legSagLeanGradVelZAbs;
        legSagLean +=
            m_gcvAcc.x() * (m_gcvAcc.x() >= 0.0 ? config.legSagLeanGradAccXFwd : config.legSagLeanGradAccXBwd);
        hipAngleY += -legSagLean;

        // Lean laterally into curves (based on the rotational walking velocity)
        legLatLean = CMD.gcvX * CMD.gcvZ * (CMD.gcvX >= 0.0 ? config.legLatLeanGradXZFwd : config.legLatLeanGradXZBwd);
        hipAngleX += legLatLean;
    }

    //
    // Basic feedback mechanisms (hipAngleX, hipAngleY, footAngleX, footAngleY)
    //

    // Apply fused angle and gyro feedback to the legs
    double hipAngleXFeedback     = 0.0;
    double hipAngleYFeedback     = 0.0;
    double footAngleXFeedback    = 0.0;
    double footAngleYFeedback    = 0.0;
    double footAngleCtsXFeedback = 0.0;
    double footAngleCtsYFeedback = 0.0;
    if (!config.tuningNoLegFeedback) {
        // Compute the phase waveform over which to apply the foot feedback
        double footPhaseLen = utility::math::clamp(0.05, double(config.basicFootAnglePhaseLen), M_PI_2);
        double footFeedbackSlope =
            1.0 / footPhaseLen;  // This can't go uncontrolled because the footPhaseLen is clamped to a reasonable range
        double footFeedbackScaler = 0.0;
        // This works because of the clamp above
        if (CMD.limbPhase <= -M_PI_2) {
            footFeedbackScaler = utility::math::clamp(0.0, footFeedbackSlope * (CMD.limbPhase + M_PI), 1.0);
        }
        else {
            footFeedbackScaler =
                utility::math::clamp(0.0, footFeedbackSlope * (CMD.doubleSupportPhase - CMD.limbPhase), 1.0);
        }

        // Compute the integrated foot and hip angle feedback
        double hipAngleXIFeed     = config.basicIFusedHipAngleX * iFusedXFeed;
        double hipAngleYIFeed     = config.basicIFusedHipAngleY * iFusedYFeed;
        double footAngleCtsXIFeed = config.basicIFusedFootAngleCX * iFusedXFeed;
        double footAngleCtsYIFeed = config.basicIFusedFootAngleCY * iFusedYFeed;

        // Compute the total foot and hip angle feedback
        hipAngleXFeedback = config.basicFeedBiasHipAngleX + config.basicFusedHipAngleX * fusedXFeed
                            + config.basicDFusedHipAngleX * dFusedXFeed + hipAngleXIFeed
                            + config.basicGyroHipAngleX * gyroXFeed;
        hipAngleYFeedback = config.basicFeedBiasHipAngleY + config.basicFusedHipAngleY * fusedYFeed
                            + config.basicDFusedHipAngleY * dFusedYFeed + hipAngleYIFeed
                            + config.basicGyroHipAngleY * gyroYFeed;
        footAngleXFeedback = config.basicFeedBiasFootAngleX + config.basicFusedFootAngleX * fusedXFeed
                             + config.basicDFusedFootAngleX * dFusedXFeed + config.basicIFusedFootAngleX * iFusedXFeed
                             + config.basicGyroFootAngleX * gyroXFeed;
        footAngleYFeedback = config.basicFeedBiasFootAngleY + config.basicFusedFootAngleY * fusedYFeed
                             + config.basicDFusedFootAngleY * dFusedYFeed + config.basicIFusedFootAngleY * iFusedYFeed
                             + config.basicGyroFootAngleY * gyroYFeed;
        footAngleXFeedback *= footFeedbackScaler;
        footAngleYFeedback *= footFeedbackScaler;
        footAngleCtsXFeedback = config.basicFeedBiasFootAngCX + footAngleCtsXIFeed;
        footAngleCtsYFeedback = config.basicFeedBiasFootAngCY + footAngleCtsYIFeed;

        // Disable the foot and hip angle feedback if required
        if (!(config.basicEnableHipAngleX && config.basicGlobalEnable)) {
            hipAngleXFeedback = hipAngleXIFeed = 0.0;
        }
        if (!(config.basicEnableHipAngleY && config.basicGlobalEnable)) {
            hipAngleYFeedback = hipAngleYIFeed = 0.0;
        }
        if (!(config.basicEnableFootAngleX && config.basicGlobalEnable)) {
            footAngleXFeedback = 0.0;
        }
        if (!(config.basicEnableFootAngleY && config.basicGlobalEnable)) {
            footAngleYFeedback = 0.0;
        }
        if (!(config.basicEnableFootAngleCX && config.basicGlobalEnable)) {
            footAngleCtsXFeedback = footAngleCtsXIFeed = 0.0;
        }
        if (!(config.basicEnableFootAngleCY && config.basicGlobalEnable)) {
            footAngleCtsYFeedback = footAngleCtsYIFeed = 0.0;
        }

        // Apply the foot and hip angle feedback
        hipAngleX += hipAngleXFeedback;
        hipAngleY += hipAngleYFeedback;
        leg.footAngleX += footAngleXFeedback + footAngleCtsXFeedback;
        leg.footAngleY += footAngleYFeedback + footAngleCtsYFeedback;

        // Handle saving of the current integral feedback values as halt pose offsets
        if (m_saveIFeedToHaltPose && !m_savedLegIFeed) {
            // Note: The calculations of these offsets should match up with how the hip angle is added to the abstract
            // pose further down in this function!
            config.haltLegAngleXBias  = config.haltLegAngleXBias + hipAngleXIFeed;
            config.haltLegAngleY      = config.haltLegAngleY + hipAngleYIFeed;
            config.haltFootAngleXBias = config.haltFootAngleXBias + footAngleCtsXIFeed + hipAngleXIFeed;
            config.haltFootAngleY     = config.haltFootAngleY + footAngleCtsYIFeed + hipAngleYIFeed;
            config.haltLegExtensionBias =
                config.haltLegExtensionBias + config.legHipAngleXLegExtGain * std::sin(hipAngleXIFeed);
            NUClear::log("Saved the current integrated leg feedback offsets as modifications to the halt pose");
            m_savedLegIFeed = true;
        }

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config.basicGlobalEnable
            && ((config.basicIFusedHipAngleX != 0.0 && config.basicEnableHipAngleX)
                || (config.basicIFusedFootAngleX != 0.0 && config.basicEnableFootAngleX)
                || (config.basicIFusedFootAngleCX != 0.0 && config.basicEnableFootAngleCX))) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config.basicGlobalEnable
            && ((config.basicIFusedHipAngleY != 0.0 && config.basicEnableHipAngleY)
                || (config.basicIFusedFootAngleY != 0.0 && config.basicEnableFootAngleY)
                || (config.basicIFusedFootAngleCY != 0.0 && config.basicEnableFootAngleCY))) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Hip angle (angleX, angleY, footAngleX, footAngleY, extension)
    //

    // Add the required hip angle X to the abstract pose
    leg.angleX += hipAngleX;
    leg.footAngleX += hipAngleX;
    if (utility::math::sign0(hipAngleX) == leg.cld.limbSign) {
        leg.extension += config.legHipAngleXLegExtGain * std::abs(std::sin(hipAngleX));
    }

    // Add the required hip angle Y to the abstract pose
    leg.angleY += hipAngleY;
    leg.footAngleY += hipAngleY;

    //
    // Support coefficients
    //

    // The support coefficient gives the proportion of the robot's weight that is expected to be on this leg at the
    // current time. It is a dimensionless parameter in the range from 0 to 1, and ideally the sum of the left and right
    // leg support coefficients should be 1 at all times. In general during a leg's swing phase the support coefficient
    // is 0, during the support phase the support coefficient is 1, and over the double support phase the support
    // coefficient is linearly blended. The exact start and stop of the support transition blends are given by
    // CMD.suppTransStartPhase and CMD.suppTransStopPhase.

    // Calculate the phase length over which the support coefficient transition should take place
    double supportTransitionPhaseLen = CMD.suppTransStopPhase - CMD.suppTransStartPhase;
    double supportTransitionSlope    = 1e10;
    if (supportTransitionPhaseLen > 1e-10) {
        supportTransitionSlope = 1.0 / supportTransitionPhaseLen;
    }

    // Calculate the required support coefficient of this leg
    if (CMD.limbPhase <= CMD.suppTransStopPhase - M_PI) {
        leg.cld.supportCoeff = utility::math::clamp(
            0.0, 1.0 - supportTransitionSlope * (CMD.suppTransStopPhase - M_PI - CMD.limbPhase), 1.0);
    }
    else if (CMD.limbPhase <= CMD.suppTransStartPhase) {
        leg.cld.supportCoeff = 1.0;
    }
    else if (CMD.limbPhase <= CMD.suppTransStopPhase) {
        leg.cld.supportCoeff =
            utility::math::clamp(0.0, supportTransitionSlope * (CMD.suppTransStopPhase - CMD.limbPhase), 1.0);
    }
    else if (CMD.limbPhase >= CMD.suppTransStartPhase + M_PI) {
        leg.cld.supportCoeff =
            utility::math::clamp(0.0, supportTransitionSlope * (CMD.limbPhase - CMD.suppTransStartPhase - M_PI), 1.0);
    }
    else {
        leg.cld.supportCoeff = 0.0;
    }

    // Rescale the support coefficients to the required range
    leg.cld.supportCoeff = config.supportCoeffRange * (leg.cld.supportCoeff - 0.5) + 0.5;

    // Disable variations in the support coefficients if required
    if (config.tuningNoLegSuppCoeff) leg.cld.supportCoeff = 0.5;

    //
    // Plotting
    //

    // Plot the leg motion components
    if (m_PM.getEnabled()) {
        m_PM.plotScalar(legExtensionOffset, PM_LEG_EXTENSION_R + leg.cld.isLeft);
        m_PM.plotScalar(CMD.swingAngle, PM_LEG_SWING_ANGLE_R + leg.cld.isLeft);
        m_PM.plotScalar(legSagSwing, PM_LEG_SAG_SWING_R + leg.cld.isLeft);
        m_PM.plotScalar(legLatSwing, PM_LEG_LAT_SWING_R + leg.cld.isLeft);
        m_PM.plotScalar(legRotSwing, PM_LEG_ROT_SWING_R + leg.cld.isLeft);
        m_PM.plotScalar(legLatHipSwing, PM_LEG_LAT_HIP_SWING_R + leg.cld.isLeft);
        m_PM.plotScalar(legSagLean, PM_LEG_SAG_LEAN_R + leg.cld.isLeft);
        m_PM.plotScalar(legLatLean, PM_LEG_LAT_LEAN_R + leg.cld.isLeft);
        m_PM.plotScalar(hipAngleXFeedback, PM_LEG_FEED_HIPANGLEX_R + leg.cld.isLeft);
        m_PM.plotScalar(hipAngleYFeedback, PM_LEG_FEED_HIPANGLEY_R + leg.cld.isLeft);
        m_PM.plotScalar(footAngleXFeedback, PM_LEG_FEED_FOOTANGLEX_R + leg.cld.isLeft);
        m_PM.plotScalar(footAngleYFeedback, PM_LEG_FEED_FOOTANGLEY_R + leg.cld.isLeft);
        m_PM.plotScalar(footAngleCtsXFeedback, PM_LEG_FEED_FOOTANGLECX_R + leg.cld.isLeft);
        m_PM.plotScalar(footAngleCtsYFeedback, PM_LEG_FEED_FOOTANGLECY_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.extension, PM_LEG_ABS_LEGEXT_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.angleX, PM_LEG_ABS_LEGANGLEX_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.angleY, PM_LEG_ABS_LEGANGLEY_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.angleZ, PM_LEG_ABS_LEGANGLEZ_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.footAngleX, PM_LEG_ABS_FOOTANGLEX_R + leg.cld.isLeft);
        m_PM.plotScalar(leg.footAngleY, PM_LEG_ABS_FOOTANGLEY_R + leg.cld.isLeft);
    }
}

// Generate the abstract arm motion
// 'arm' is assumed to contain the desired arm halt pose
void GaitEngine::abstractArmMotion(pose::AbstractArmPose& arm) {
    //
    // Common motion data
    //

    // The limb phase and swing angles for the arms are calculated exactly like for the legs, only the limb phase is
    // inverted. That is, the support and swing phases of the left arm match those of the right leg, and vice versa.

    // Calculate the common motion data
    CommonMotionData CMD = calcCommonMotionData(arm.cad.isLeft != m_leftLegFirst);

    // Disable the arm swing if required
    if (config.tuningNoArmSwing) {
        CMD.swingAngle = 0.0;
    }

    //
    // Sagittal arm swing (angleY)
    //

    // Apply the sagittal arm swing to the abstract arm pose
    double armSagSwingMag = config.armSagSwingMag + CMD.gcvX * config.armSagSwingMagGradX;
    double armSagSwing    = -CMD.swingAngle * armSagSwingMag;
    arm.angleY += armSagSwing;

    //
    // Basic feedback mechanisms (angleX, angleY)
    //

    // Apply fused angle and gyro feedback to the arms
    double armAngleXFeedback = 0.0;
    double armAngleYFeedback = 0.0;
    if (!config.tuningNoArmFeedback) {
        // Compute the integrated arm angle feedback
        double armAngleXIFeed = config.basicIFusedArmAngleX * iFusedXFeed;
        double armAngleYIFeed = config.basicIFusedArmAngleY * iFusedYFeed;

        // Compute the total arm angle feedback
        armAngleXFeedback = config.basicFeedBiasArmAngleX + config.basicFusedArmAngleX * fusedXFeed
                            + config.basicDFusedArmAngleX * dFusedXFeed + armAngleXIFeed
                            + config.basicGyroArmAngleX * gyroXFeed;
        armAngleYFeedback = config.basicFeedBiasArmAngleY + config.basicFusedArmAngleY * fusedYFeed
                            + config.basicDFusedArmAngleY * dFusedYFeed + armAngleYIFeed
                            + config.basicGyroArmAngleY * gyroYFeed;

        // Disable the arm angle feedback if required
        if (!(config.basicEnableArmAngleX && config.basicGlobalEnable)) {
            armAngleXFeedback = armAngleXIFeed = 0.0;
        }
        if (!(config.basicEnableArmAngleY && config.basicGlobalEnable)) {
            armAngleYFeedback = armAngleYIFeed = 0.0;
        }

        // Apply the arm angle feedback
        arm.angleX += armAngleXFeedback;
        arm.angleY += armAngleYFeedback;

        // Handle saving of the current integral feedback values as halt pose offsets
        if (m_saveIFeedToHaltPose && !m_savedArmIFeed) {
            config.haltArmAngleXBias = config.haltArmAngleXBias + armAngleXIFeed;
            config.haltArmAngleY     = config.haltArmAngleY + armAngleYIFeed;
            NUClear::log("Saved the current integrated arm feedback offsets as modifications to the halt pose");
            m_savedArmIFeed = true;
        }

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config.basicIFusedArmAngleX != 0.0 && config.basicEnableArmAngleX
            && config.basicGlobalEnable) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config.basicIFusedArmAngleY != 0.0 && config.basicEnableArmAngleY
            && config.basicGlobalEnable) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Plotting
    //

    // Plot the arm motion components
    if (m_PM.getEnabled()) {
        m_PM.plotScalar(CMD.swingAngle, PM_ARM_SWING_ANGLE_R + arm.cad.isLeft);
        m_PM.plotScalar(armSagSwing, PM_ARM_SAG_SWING_R + arm.cad.isLeft);
        m_PM.plotScalar(armAngleXFeedback, PM_ARM_FEED_ARMANGLEX_R + arm.cad.isLeft);
        m_PM.plotScalar(armAngleYFeedback, PM_ARM_FEED_ARMANGLEY_R + arm.cad.isLeft);
        m_PM.plotScalar(arm.extension, PM_ARM_ABS_ARMEXT_R + arm.cad.isLeft);
        m_PM.plotScalar(arm.angleX, PM_ARM_ABS_ARMANGLEX_R + arm.cad.isLeft);
        m_PM.plotScalar(arm.angleY, PM_ARM_ABS_ARMANGLEY_R + arm.cad.isLeft);
    }
}

// Generate the inverse leg motion
// 'leg' is assumed to contain the desired abstract motion
void GaitEngine::inverseLegMotion(pose::InverseLegPose& leg) {
    //
    // Common motion data
    //

    // Calculate the common motion data
    // Note: We don't zero out the swingAngle here if tuning says we should because we use it for virtual slope walking
    // not swinging in this function
    CommonMotionData CMD = calcCommonMotionData(leg.cld.isLeft == m_leftLegFirst);

    //
    // Basic feedback mechanisms (footPosX, footPosY)
    //

    // Apply fused angle and gyro feedback
    double comShiftXFeedback = 0.0;
    double comShiftYFeedback = 0.0;
    if (!config.tuningNoLegFeedback) {
        // Compute the CoM shifting feedback
        comShiftXFeedback = config.basicFeedBiasComShiftX + config.basicFusedComShiftX * fusedYFeed
                            + config.basicDFusedComShiftX * dFusedYFeed + config.basicIFusedComShiftX * iFusedYFeed
                            + config.basicGyroComShiftX * gyroYFeed;
        comShiftYFeedback = -(config.basicFeedBiasComShiftY + config.basicFusedComShiftY * fusedXFeed
                              + config.basicDFusedComShiftY * dFusedXFeed
                              + config.basicIFusedComShiftY * iFusedXFeed
                              + config.basicGyroComShiftY * gyroXFeed);

        // Apply the required limits if enabled
        if (config.basicComShiftXUseLimits) {
            comShiftXFeedback = utility::math::clampSoft(double(config.basicComShiftXMin),
                                                         comShiftXFeedback,
                                                         double(config.basicComShiftXMax),
                                                         double(config.basicComShiftXBuf));
        }
        if (config.basicComShiftYUseLimits) {
            comShiftYFeedback = utility::math::clampSoft(double(config.basicComShiftYMin),
                                                         comShiftYFeedback,
                                                         double(config.basicComShiftYMax),
                                                         double(config.basicComShiftYBuf));
        }

        // Disable the CoM shifting feedback if required
        if (!(config.basicEnableComShiftX && config.basicGlobalEnable)) {
            comShiftXFeedback = 0.0;
        }
        if (!(config.basicEnableComShiftY && config.basicGlobalEnable)) {
            comShiftYFeedback = 0.0;
        }

        // Apply the CoM shifting feedback
        leg.footPos.x() += comShiftXFeedback;
        leg.footPos.y() += comShiftYFeedback;

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config.basicIFusedComShiftY != 0.0 && config.basicComShiftYMin < 0.0
            && config.basicComShiftYMax > 0.0
            && config.basicEnableComShiftY
            && config.basicGlobalEnable) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config.basicIFusedComShiftX != 0.0 && config.basicComShiftXMin < 0.0
            && config.basicComShiftXMax > 0.0
            && config.basicEnableComShiftX
            && config.basicGlobalEnable) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Virtual slope leg lifting (footPosZ)
    //

    // Adjust the lift height of the foot depending on the virtual slope (a slope derived from the pitch fused angle and
    // a configured offset). This has the qualitative effect that the robot lifts its feet more when it is falling
    // forwards.
    double virtualComponent = 0.0;
    if (config.basicEnableVirtualSlope && config.basicGlobalEnable && !config.tuningNoLegVirtual) {
        double endVirtualSlope = virtualSlope * CMD.gcvX;
        if (endVirtualSlope >= 0.0) {
            virtualComponent = 0.5 * (CMD.swingAngle + 1.0) * endVirtualSlope;
        }
        else {
            virtualComponent = 0.5 * (CMD.swingAngle - 1.0) * endVirtualSlope;
        }
        leg.footPos.z() += virtualComponent;
    }

    //
    // Plotting
    //

    // Plot the inverse leg motion components
    if (m_PM.getEnabled()) {
        m_PM.plotScalar(comShiftXFeedback, PM_LEG_FEED_COMSHIFTX_R + leg.cld.isLeft);
        m_PM.plotScalar(comShiftYFeedback, PM_LEG_FEED_COMSHIFTY_R + leg.cld.isLeft);
        m_PM.plotScalar(virtualSlope, PM_LEG_VIRTUAL_SLOPE_R + leg.cld.isLeft);
        m_PM.plotScalar(virtualComponent, PM_LEG_VIRTUAL_COMP_R + leg.cld.isLeft);
    }
}

// Abstract pose coercion function
void GaitEngine::clampAbstractPose(pose::AbstractPose& pose) {
    // Coerce each of the limbs in the abstract pose
    clampAbstractArmPose(pose.leftArm);
    clampAbstractArmPose(pose.rightArm);
    clampAbstractLegPose(pose.leftLeg);
    clampAbstractLegPose(pose.rightLeg);
}

// Abstract arm pose coercion function
void GaitEngine::clampAbstractArmPose(pose::AbstractArmPose& arm) {
    // Apply the required limits if enabled
    if (config.limArmAngleXUseLimits) {
        arm.angleX = arm.cad.limbSign
                     // Minimum is negative towards inside, maximum is positive towards outside
                     * utility::math::clampSoft(config.limArmAngleXMin,
                                                arm.angleX / arm.cad.limbSign,
                                                config.limArmAngleXMax,
                                                config.limArmAngleXBuf);
    }
    if (config.limArmAngleYUseLimits) {
        arm.angleY = utility::math::clampSoft(
            config.limArmAngleYMin, arm.angleY, config.limArmAngleYMax, config.limArmAngleYBuf);
    }
}

// Abstract leg pose coercion function
void GaitEngine::clampAbstractLegPose(pose::AbstractLegPose& leg) {
    // Apply the required limits if enabled
    if (config.limLegAngleXUseLimits)
        leg.angleX =
            leg.cld.limbSign
            * utility::math::clampSoft(config.limLegAngleXMin,
                                       leg.angleX / leg.cld.limbSign,
                                       config.limLegAngleXMax,
                                       4);  // Minimum is negative towards inside, maximum is positive towards outside
    if (config.limLegAngleYUseLimits) {
        leg.angleY = utility::math::clampSoft(config.limLegAngleYMin, leg.angleY, config.limLegAngleYMax, 4);
    }
    if (config.limFootAngleXUseLimits) {
        leg.footAngleX = leg.cld.limbSign
                         // Minimum is negative towards inside, maximum is positive towards outside
                         * utility::math::clampSoft(config.limFootAngleXMin,
                                                    leg.footAngleX / leg.cld.limbSign,
                                                    config.limFootAngleXMax,
                                                    config.limFootAngleXBuf);
    }
    if (config.limFootAngleYUseLimits) {
        leg.footAngleY = utility::math::clampSoft(
            config.limFootAngleYMin, leg.footAngleY, config.limFootAngleYMax, config.limFootAngleYBuf);
    }
    if (config.limLegExtUseLimits) {
        leg.extension = utility::math::clampSoftMin(config.limLegExtMin, float(leg.extension), config.limLegExtBuf);
    }
}

// Update outputs function
void GaitEngine::updateOutputs() {
    // Transcribe the joint commands
    out.jointCmd        = m_jointPose.writeJointPosArray();
    out.jointEffort     = m_jointPose.writeJointEffortArray();
    out.useRawJointCmds = haltUseRawJointCmds;

    // Transcribe the walking flag
    out.walking = m_walking || m_blending;

    // Transcribe the leg support coefficients
    out.supportCoeffLeftLeg  = m_jointPose.leftLeg.cld.supportCoeff;
    out.supportCoeffRightLeg = m_jointPose.rightLeg.cld.supportCoeff;

    // Update the odometry
    updateOdometry();
}

// Callback for when the plotData parameter is updated
void GaitEngine::callbackPlotData() {
    // Enable or disable plotting as required
    if (m_plotData()) {
        m_PM.enable();
    }
    else {
        m_PM.disable();
    }
}

// Reset the blending variables
void GaitEngine::resetBlending(double b) {
    // Disable the blending variables
    m_blending      = false;
    m_b_current     = b;
    m_b_initial     = b;
    m_b_target      = b;
    m_blendPhase    = 0.0;
    m_blendEndPhase = 0.0;
}

// Set a new blend target (USE_CALC_POSE = 0, USE_HALT_POSE = 1, and everything inbetween is interpolation)
void GaitEngine::setBlendTarget(double target,
                                double phaseTime)  // phaseTime is the gait phase in which to complete the blend
{
    // Blend target range checking
    target = utility::math::clamp(0.0, target, 1.0);

    // Immediately change the current blending factor if the required phase time is non-positive, or the current blend
    // factor is already equal to the target blend factor
    if (phaseTime <= 0.0 || m_b_current == target) {
        resetBlending(target);
        return;
    }

    // Update the blending variables
    m_blending      = true;
    m_b_initial     = m_b_current;
    m_b_target      = target;
    m_blendPhase    = 0.0;
    m_blendEndPhase = phaseTime;
}

// Evaluate the current blend factor
double GaitEngine::blendFactor() {
    // Update the current blend factor if we are in the process of blending
    if (m_blending) {
        // Should never happen...
        if (m_blendPhase < 0.0) {
            m_blendPhase = 0.0;
            m_b_current  = m_b_initial;
        }
        // Done with our blend...
        else if (m_blendPhase >= m_blendEndPhase) {
            resetBlending(m_b_target);  // Note: This internally sets m_blending to false
        }
        // In the process of blending...
        else {
            double u    = sin(M_PI_2 * m_blendPhase / m_blendEndPhase);
            m_b_current = m_b_initial + (u * u) * (m_b_target - m_b_initial);
        }
    }

    // Return the current blend factor
    return m_b_current;
}

// Reset the motion stance adjustment variables
void GaitEngine::resetMotionStance() {
    // Reset variables
    m_motionLegAngleXFact = 1.0;  // Feet normal
}

}  // namespace gait