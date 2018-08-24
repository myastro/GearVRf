/* Articulated Character Engine C API */
#ifndef ACE_API_H
#define ACE_API_H

#include "ape_api.h"

// -----------------------------------------------------------------------------------------
// ACE (Articulated Character Engine) API
// -----------------------------------------------------------------------------------------

AP_DECLARE_HANDLE(acController);
AP_DECLARE_HANDLE(acSimBiConState);
AP_DECLARE_HANDLE(acTrajectory);
AP_DECLARE_HANDLE(acExternalForce);
AP_DECLARE_HANDLE(acTrajector1d);
AP_DECLARE_HANDLE(acLinearBalanceFeedback);
AP_DECLARE_HANDLE(acTrajectoryComponent);
AP_DECLARE_HANDLE(acSimBiControllerState);
AP_DECLARE_HANDLE(acTRexControllerState);
AP_DECLARE_HANDLE(acHumanoidControllerState);
AP_DECLARE_HANDLE(acDogControllerState);
AP_DECLARE_HANDLE(acBVH);
AP_DECLARE_HANDLE(acMotionClip);
AP_DECLARE_HANDLE(acFeedbackPolicyVector);
AP_DECLARE_HANDLE(acIKActuator);

#ifdef __cplusplus
extern "C" {
#endif

	EXPORT_API void acRegisterController(acController controller);
	EXPORT_API void acUnregisterController(acController controller);
    EXPORT_API void acEnableController(acController controller, bool enable);
	EXPORT_API void acDeleteController(acController controller);

	EXPORT_API acController acCreateBipedController(
		apArticulationDynamicsWorldHandle world,
		apArticulationBaseHandle base,
		int lFoot, int rFoot, int lAnkle, int rAnkle, int lKnee, int rKnee,
		int lHip, int rHip, int lowerBack, int torso, int head,
		apReal stanceHipDamping, apReal stanceHipMaxVelocity, apReal rootPredictiveTorqueScale,
		int startingState, int startingStance, apReal stepWidth,
		bool isIKVMOn
		);

	EXPORT_API void acSetIKVMMode(acController controller, bool isInvertedPendulumOn, bool isGravityCompensationOn);
	EXPORT_API void acEnableRagdoll(acController controller, bool enable);
	EXPORT_API void acSetMaxGyro(acController controller, apReal maxGyro);
	EXPORT_API void acUseExplicitPDControl(acController controller, int linkIdx);
    EXPORT_API void acUseImplicitPDControl(acController controller, int linkIdx);
    EXPORT_API void acUseConstraintBasedPDControl(acController controller, int linkIndex);
    EXPORT_API void acUseExplicitWithConstrainedBasedDampingPDControl(acController controller, int linkIndex);
	EXPORT_API void acUseImplicitPositionError(acController controller, bool use);
	EXPORT_API void acUseMOIAboutJointPosition(acController controller, bool use);

	EXPORT_API void acInstallStateSensor(acController controller, acSimBiControllerState pState);
	int EXPORT_API acGetCurrentState(acController controller);
	EXPORT_API void acSetNextState(acController controller, int curState, int nextState, bool immediate);
	EXPORT_API void acSetCurrentNextState(acController controller, int nextState, bool immeidate);

	// TBD: apDeleteBipedController()
	EXPORT_API acSimBiConState acAddSimBiConState(
		acController controller,
		int nextStateIndex,
		apReal stateTime,
		apReal maxGyro,
		bool reverseStance,
		bool keepStance,
		int stateStance,
		bool transitionOnFootContact,
		apReal minPhi,
		apReal minForce
		);

	EXPORT_API acExternalForce acAddExternalForce(acSimBiConState state, int leftStanceIndex, int rightStanceIndex);
	EXPORT_API acTrajector1d acGetForceXTraj(acExternalForce force);
	EXPORT_API acTrajector1d acGetForceYTraj(acExternalForce force);
	EXPORT_API acTrajector1d acGetForceZTraj(acExternalForce force);
	EXPORT_API acTrajector1d acGetTorqueXTraj(acExternalForce force);
	EXPORT_API acTrajector1d acGetTorqueYTraj(acExternalForce force);
	EXPORT_API acTrajector1d acGetTorqueZTraj(acExternalForce force);

	EXPORT_API void acAddTrajectory1DElement(acTrajector1d traj, apReal t, apReal value);
	EXPORT_API void acUpdateTrajectory1D(acTrajector1d traj, int knotCount, const apReal* pTime, const apReal* pValue);

	EXPORT_API acTrajectory acAddTrajectory(acSimBiConState state, int leftStanceIndex, int rightStanceIndex, bool relToCharFrame);

	EXPORT_API acLinearBalanceFeedback acCreateLinearBalanceFeedback(apReal axisX, apReal axisY, apReal axisZ,
																	 apReal cd, apReal cv, apReal vMin, apReal vMax, apReal dMin, apReal dMax);

	EXPORT_API acTrajector1d acAddTrajectoryComponent(acTrajectory trajectory, bool reverseAngleOnLeftStance,
													  bool reverseAngleOnRightStance, apReal axisX, apReal axisY, apReal axisZ, acLinearBalanceFeedback bFeedback);
	EXPORT_API acTrajector1d acGetStrengthTraj(acTrajectory trajectory);
	EXPORT_API acTrajector1d acGetDTrajX(acSimBiConState state);
	EXPORT_API acTrajector1d acGetDTrajZ(acSimBiConState state);
	EXPORT_API acTrajector1d acGetVTrajX(acSimBiConState state);
	EXPORT_API acTrajector1d acGetVTrajZ(acSimBiConState state);

	// -----------------------------------------------------------------------------------------
	// ACE v2 API
	// -----------------------------------------------------------------------------------------

    EXPORT_API acController acCreateTRexController(
        apArticulationDynamicsWorldHandle world,
        apArticulationBaseHandle base,
        int torso, int lHand, int rHand, int lToes, int rToes, int head, int tail,
        apReal* controlParams, int numControlParams,
        apReal* startPose,
        apReal* initDesiredPose,
        bool keepRootPos,
        bool enableConstraintPDFix
		);

	EXPORT_API acController acCreateHumanoidController(
		apArticulationDynamicsWorldHandle world,
		apArticulationBaseHandle base,
		int lowerBack, int upperBack, int neck, int lShoulder, int rShoulder,
		int lToes, int rToes, int lHand, int rHand,
        apReal limbTrackingKp, apReal limbTrackingKd, apReal limbTrackingMaxForce,
		apReal* controlParams, int numControlParams,
		apReal* startPose,
		apReal* initDesiredPose,
        apReal blendGranularity,
        bool keepRootPos,
        bool antiLegCrossing,
        bool useBlendSpace,
        int grfSolverMode,
        bool stepRelativeToCOM,
        bool stepRelativeToRoot,
        bool enableConstraintPDFix,
		const biVector3* bodyForward,
		const biVector3* bodyRight
		);

	EXPORT_API acController acCreateDogController(
		apArticulationDynamicsWorldHandle world,
		apArticulationBaseHandle base,
		int torso, int lHand, int rHand, int lToes, int rToes, int head, int tail,
        apReal limbTrackingKp, apReal limbTrackingKd, apReal limbTrackingMaxforce,
		apReal* controlParams, int numControlParams,
		apReal* startPose,
		apReal* initDesiredPose,
        apReal blendGranularity,
		bool keepRootPos,
        bool useBlendSpace,
        int grfSolverMode,
        bool enableConstraintPDFix,
		const biVector3* bodyForward, const biVector3* bodyRight
		);

    EXPORT_API void acInstallCommonControllerSharedMemoryBuffers(
        acController controller,
        apReal* currentPose, apReal* desiredPose, apReal* controlParams);

    EXPORT_API void acInstallTRexControllerSharedMemoryBuffers(
        acController controller,
        acTRexControllerState pStateSensor,
        biVector3* lEEPosSensor, biVector3* rEEPosSensor);

    EXPORT_API void acInstallHumanoidControllerSharedMemoryBuffers(
        acController controller,
        acHumanoidControllerState pStateSensor);

    EXPORT_API void acInstallDogControllerSharedMemoryBuffers(
        acController controller,
        acDogControllerState pStateSensor);

    EXPORT_API acController acCreateTrackingController(apArticulationBaseHandle base, bool enableConstraintPDFix);

    EXPORT_API acController acCreatePoseController(
        apArticulationDynamicsWorldHandle world,
        apArticulationBaseHandle base,
        bool enableConstraintPDFix);

    EXPORT_API void acAddPoseControlChain(acController controller, int endEffector, int root, apReal* targetPosRot, int mode, apReal posKP, apReal posKD, apReal rotKP, apReal rotKD, apReal maxForce);
    EXPORT_API void acSetPoseControllerToleranceThreshold(acController controller, apReal tolerance);

	EXPORT_API void acSetControllerDead(acController controller, bool dead);
    EXPORT_API void acAddHumanoidBlendSample(
        acController controller, 
        apReal* blendSample, 
        apReal fwdVel, 
        apReal sideVel, 
        apReal turnVel
        );

	EXPORT_API void acInitRootPdParams(
		acController controller,
		bool controlled,
		apReal kp, apReal kd,
		apReal maxAbsTorque,
		apReal scaleX, apReal scaleY, apReal scaleZ,
		bool relToCharFrame
		);

	EXPORT_API void acAddLinkPdParams(
		acController controller,
		bool controlled,
		apReal kp, apReal kd,
		apReal maxAbsTorque,
		apReal scaleX, apReal scaleY, apReal scaleZ,		
		bool relToCharFrame
		);

	EXPORT_API void acUpdateLinkPdParams(
		acController controller,
		int index,
		apReal kp, apReal kd,
		apReal maxAbsTorque
		);

	EXPORT_API void acUpdateLinkPdModifiers(
		acController controller,
		int index,
		apReal kpModifier, apReal kdModifier
		);

    EXPORT_API void acScaleLinkPdParams(
        acController controller,
        int index,
        apReal kpScale, apReal kdScale,
        apReal maxAbsTorqueScale
    );

    EXPORT_API void acAddPdParamSet(
        acController controller, 
        apReal* pdParamSet, 
        int numLinks
        );

	EXPORT_API void acSetDesiredPose(acController controller, apReal* q);
	EXPORT_API void acSetCurrentPose(acController controller, apReal* q);
    EXPORT_API void acSetReducedVirtualAgentState(acController controller, apReal* state, bool keepRootPosAndOri);
    EXPORT_API void acSetMirroredReducedVirtualAgentState(acController controller, apReal* state, bool keepRootPosAndOri);	

	// return center of mass in world frame
	EXPORT_API void acGetCOM(acController controller, biVector3 *com);

	EXPORT_API apReal acGetDesiredHeading(acController controller);
	EXPORT_API void acSetDesiredHeading(acController controller, apReal desiredHeading);
	EXPORT_API void acSetDesiredVelocities(acController controller, apReal forwardVel, apReal sideVel, apReal upVel);
	EXPORT_API void acActiviateSupportLeg(acController controller, int legIndex, bool enabled);
	EXPORT_API void acEnableLeftShoulderSwing(acController controller, bool enabled);
	EXPORT_API void acEnableRightShoulderSwing(acController controller, bool enabled);
	EXPORT_API void acEnableNeckControl(acController controller, bool enabled);
    EXPORT_API void acEnableTorsoControl(acController controller, bool enabled);
	EXPORT_API acIKActuator acRegisterIKActuator(acController controller, int endEffector, int IKRoot, apReal* targetPosWS, int* active);
	EXPORT_API void acUnregisterIKActuator(acController controller, acIKActuator act);
	EXPORT_API void acSetIKActuatorToleranceThreshold(acIKActuator act, apReal tolerance);

	// samcon
	EXPORT_API acBVH acLoadBVH(void* bvhPath, int* numJoints, int* numFrames, float* frameTime);
	EXPORT_API acBVH acLoadBVHFromMemory(void* content, int* numJoints, int* numFrames, float* frameTime);
	EXPORT_API void acGetBasicBVHInfo(acBVH bvhHandle, int* numJoints, int* numFrames, float* frameTime);
	EXPORT_API void acMoveBVHFrameTransformationsToSkeleton(acBVH bvhHandle, int frame);
	EXPORT_API void acDeleteBVH(acBVH bvhHandle);
	EXPORT_API size_t acGetBVHJointNameSize(acBVH bvhHandle, int jointIndex);
	EXPORT_API void acGetBVHJointName(acBVH bvhHandle, int jointIndex, void *nameBuffer);
	EXPORT_API void acGetBVHSkeletonTransforms(acBVH bvhHandle, float* outputPos, float* outputRot);
	EXPORT_API void acGetBVHFrameOrdered(acBVH bvhHandle, apReal frame, int local, float* outputPos, float* outputRot);
	EXPORT_API void acGetBVHJointHierachy(acBVH bvhHandle, int *joints);
	
	EXPORT_API acController acCreateMotionClipGuidedPoseController(apArticulationDynamicsWorldHandle world, apArticulationBaseHandle base);
	EXPORT_API void acConfigureMotionClipGuidedPoseControllerTrackingMode(acController controller, bool trackRootPosDirectly, bool trackRootOriDirectly, bool trackJointsOriDirectly);
	EXPORT_API void acSetMotionClipGuidedPoseControllerRootFixed(acController controller, bool rootFixed);

	EXPORT_API void acSetMotionClipGuidedPoseControllerState(acController controller, apReal* state, bool keepRootPosAndOri);
	EXPORT_API void acSetMotionClipGuidedPoseControllerMotionClipFrameDuration(acController controller, apReal frameDuration);
	EXPORT_API apReal acGetMotionClipGuidedPoseControllerCurrentAnimFrameIndex(acController controller);
	EXPORT_API void acSetMotionClipGuidedPoseControllerCurrentAnimFrameIndex(acController controller, apReal frame);
	EXPORT_API void acSetMotionClipGuidedPoseControllerFrameUpdateModeToNone(acController controller);
	EXPORT_API void acSetMotionClipGuidedPoseControllerFrameUpdateModeToStatic(acController controller, int staticFrameLifetime);
	EXPORT_API void acSetMotionClipGuidedPoseControllerFrameUpdateModeToDynamic(acController controller);
	EXPORT_API void acSetMotionClipGuidedPoseControllertUseInterpolatedFramesAsTarget(acController controller, bool useInterpolated);
	EXPORT_API void acSetMotionClipGuidedPoseControllertUseZeroReferenceVelocities(acController controller, bool useInterpolated);
	EXPORT_API void acSetMotionClipGuidedPoseControllertLoopedTrackingEnabled(acController controller, bool loopedEnabled);
	EXPORT_API void acSetMotionClipGuidedPoseControllertUpdateDesiredPoseFromMotionClip(acController controller, bool update);

	EXPORT_API void acResetControllerTorques(acController controller);

	EXPORT_API bool acApplySAMCONProcessorToMotionClip(
		apArticulationDynamicsWorldHandle world,
		acController controller,
		acMotionClip pInputClip, acMotionClip pOutputClip, acMotionClip pReferenceClip, acMotionClip pDenseTargetClip,
        bool useZeroReferenceTarget,
		apReal introduceArtificialNoise,
		apReal *pInitialCharacterStateArray,
		int initialCharacterStateMode,		// 0 - use pInitialCharacterState (if valid), 1 - apply first ref clip frame (if valid), 2 - apply first in/tgt clip frame
											// if mode == 0 but pInitialCharacterState is invalid, mode = 1 is assumed; if mode == 1 and pReferenceClip is invalid, mode = 2			
		acFeedbackPolicyVector pInputPoliciesVec, bool policiesOverridesDesiredPose,		// vector of linear feedback policies + policy override flag
		// Libin16 state converter settings
		int leftFootLinkIdx, int rightFootLinkIdx,
		int numActionLinkIndices, int *pActionLinksIndices,
		//
		acMotionClip pSimResultsClip,
		acMotionClip pTargetOffsetsClip, acMotionClip pTargetBasesClip,
		acMotionClip pTargetOffetsFromPolicyClip,
		apReal fixedSimTimeStep,
		EndEffectorQuery pEndEffectorQuery,
		PerDofSamplingWindowQuery pWindowQuery, int randomNumberGeneratorType,			// 0 - uniform, 1 - normal, 2 - none
		int stateValidityQueryMode,								// 0 - root ori, 1 - CoM height, 2 - always OK, 3 - always NOK, 4 - external
		PoseVerifierWithExtValidityDef::ExternalStateValidityQuery pStateValidityQuery,
		SAMCONMotionClipProcessor::PostSingleSampleCallback pPostSingleSampleCallback,
		SAMCONMotionClipProcessor::PostSingleIterationCallback pPostSingleIterationCallback,
		int maxNumOfTrials, int numSamplesPerIteration,
		int iterationDurationAsIntMultipleOfSimStep,
		int auxNumberOfIterationsPerTrialLimit,
		apReal characterHeight,
		bool pickDiverse, int numOfEliteSamplesToPick,
		apReal wp, apReal wr, apReal we, apReal wb,
		bool savePartialResultsOnFailure);

	EXPORT_API bool acApplyAveragingProcessorToMotionClip(
		int numRuns, bool prserveFrameDuration,
		acMotionClip pInputClip, acMotionClip pOutputClip
	);

	EXPORT_API bool acApplyFwdDiffProcessorToMotionClip(
		acMotionClip pInputClip, acMotionClip pOutputClip
	);

	EXPORT_API bool acApplyInverseDynamicsProcessorToMotionClip(
		void *world, void *character, void *controller, 
        acMotionClip pInputClip, acMotionClip pOutputClip,
        const char *configStr
	);

	EXPORT_API bool acApplyLoopingProcessorToMotionClip(
		acMotionClip pInputClip, acMotionClip pOutputClip,
		int numCycles, int numExtraFramesToReRun,
        bool withMirror, int *parentIds, int *mirrorIds, 
        int planeOfSymmetry, biVector3 *normalOfPlaneOfSymmetry
	);

    EXPORT_API bool acConcatenateMotionClips(
        acMotionClip pInputClip1, acMotionClip pInputClip2, acMotionClip pOutputClip,
        const char *configStr
    );

	EXPORT_API bool acApplyResamplingProcessorToMotionClip(
		acMotionClip pInputClip, acMotionClip pOutputClip,
		int frequency, bool preserveClipDuration, bool preserveLastFrame
	);

    EXPORT_API bool acEditMotionClip(
        acMotionClip pInputClip, acMotionClip pOutputClip, const char *config
    );

	EXPORT_API bool acApplyClampingProcessorToMotionClip(
		acMotionClip pInputClip, acMotionClip pOutputClip,
		int startIdx, int endIdx
	);

    EXPORT_API bool acApplyFilteringProcessorToMotionClip(
        acMotionClip pInputClip, acMotionClip pOutputClip, const char *configStr
    );

    EXPORT_API acMotionClip acCreateMotionClip(apByte *pClipData, int numBytesInBuffer);
    EXPORT_API acMotionClip acCreateEmptyMotionClipForSkeleton(int numNames, const char **linkNames);
	EXPORT_API void acDeleteMotionClip(acMotionClip pClip);
	EXPORT_API int acMotionClipGetNecessaryRawClipBufferByteSize(acMotionClip pClip);
	EXPORT_API void acMotionClipDumpToRawBuffer(acMotionClip pClip, apByte *pClipData);
	EXPORT_API void acMotionClipSetNumFrames(acMotionClip pClip, int numFrames);
	EXPORT_API int acMotionClipGetNumFrames(acMotionClip pClip);
	EXPORT_API void acMotionClipSetFrameDuration(acMotionClip pClip, apReal frameDuration);
	EXPORT_API apReal acMotionClipGetFrameDuration(acMotionClip pClip);
	//EXPORT_API void acMotionClipSetNumSkeletonLinks(acMotionClip pClip, int numLinks);
	EXPORT_API int acMotionClipGetNumSkeletonLinks(acMotionClip pClip);
    EXPORT_API const char *acMotionClipGetSkeletonLinkName(acMotionClip pClip, int linkIdx);
    EXPORT_API bool acMotionClipRetargetToSkeleton(acMotionClip pClip, const char **names, int numNames);
	EXPORT_API bool acMotionClipSetOneFrameFromRawBuffer(acMotionClip pClip, int frameIdx, apByte *pOneFrameData);
	EXPORT_API int acMotionClipGetNecessaryRawOneFrameBufferByteSize(acMotionClip pClip);
	EXPORT_API bool acMotionClipDumpOneFrameToRawBuffer(acMotionClip pClip, int frameIdx, apByte *pOneFrameData);
	EXPORT_API bool acMotionClipSetFrameRootPosAndOri(acMotionClip pClip, int frameIdx, biVector3 *pRootPos, biQuaternion *pRootOri);
	EXPORT_API bool acMotionClipGetFrameRootPosAndOri(acMotionClip pClip, int frameIdx, biVector3 *pRootPos, biQuaternion *pRootOri);
	EXPORT_API bool acMotionClipSetFrameLinkRelativeOri(acMotionClip pClip, int frameIdx, int linkIdx, biQuaternion *pOri);
	EXPORT_API bool acMotionClipSetFrameLinkRelativeAngVel(acMotionClip pClip, int frameIdx, int linkIdx, biVector3 *pAngVel);
	EXPORT_API bool acMotionClipAddInto(acMotionClip pClip, acMotionClip pClipToBeAdded, acMotionClip pResultClip, bool copyFirstFrameFromBase);
	EXPORT_API bool acMotionClipSubtractInto(acMotionClip pClip, acMotionClip pClipToBeSubtracted, acMotionClip pResultClip, bool copyFirstFrameFromBase);
	EXPORT_API bool acExtractMotionClipFrame(acMotionClip pClip, int frameIdx, apReal *pState);
	EXPORT_API bool acMotionClipModifyRootPosAndOri(
		acMotionClip pClip, bool snapStartFrameRootPosToZeroFirst,
		biVector3 *pRootPosDelta, biQuaternion *pRootOriDelta);

    EXPORT_API bool acMotionClipModifyFrameLinkRelativeOri(acMotionClip pClip, int frameIdx, int linkIdx, biQuaternion *pOri, bool editInLinkLocalFrame);


	EXPORT_API void acSetMotionClipGuidedPoseControllerMotionClip(acController controller, acMotionClip pClip);
	EXPORT_API void acSetMotionClipGuidedPoseControllerDenseTargetClip(acController controller, acMotionClip pClip);
	EXPORT_API void acSetMotionClipGuidedPoseControllerFeedbackPolicyVector(acController controller, acFeedbackPolicyVector pFeedbackPolicyVec);
	EXPORT_API void acSetMotionClipGuidedPoseControllerUseMotionClip(acController controller, bool use);
	EXPORT_API void acSetMotionClipGuidedPoseControllerUseFeedbackPolicies(acController controller, bool use, bool policyOverridesPose);
	EXPORT_API void acSetMotionClipGuidedPoseControllerLibin16StateConverter(
		acController controller, int leftFootLinkIdx, int rightFootLinkIdx,
		int numActionLinkIndices, int *pActionLinksIndices);

	EXPORT_API void acSetArticulationState(apArticulationBaseHandle base, apReal* state);

	EXPORT_API bool acLearnFeedbackPolicies(
		int numLearningIterations, int nuOfInputClipRuns,
		apReal introduceArtificialNoise,
		bool skipSamconAndDoOneIterationOnly,
		apReal regularizer,
		int leftFootLinkIdx, int rightFootLinkIdx,
		int numActionLinkIndices, int *pActionLinksIndices,
		acFeedbackPolicyVector pInInitialPoliciesVec,
		acFeedbackPolicyVector pOutputPoliciesVec, bool policiesOverridesDesiredPose,
		apArticulationDynamicsWorldHandle world,
		acController controller,
		acMotionClip pInputClip, acMotionClip pReferenceClip,
		apReal *pInitialCharacterStateArray,
		int initialCharacterStateMode,		// 0 - use pInitialCharacterState (if valid), 1 - apply first ref clip frame (if valid), 2 - apply first in/tgt clip frame
											// if mode == 0 but pInitialCharacterState is invalid, mode = 1 is assumed; if mode == 1 and pReferenceClip is invalid, mode = 2
		apReal fixedSimTimeStep,
		EndEffectorQuery pEndEffectorQuery,
		PerDofSamplingWindowQuery pWindowQuery, int randomNumberGeneratorType,			// 0 - uniform, 1 - normal, 2 - none
		int stateValidityQueryMode,								// 0 - root ori, 1 - CoM height, 2 - always OK, 3 - always NOK, 4 - external
		PoseVerifierWithExtValidityDef::ExternalStateValidityQuery pStateValidityQuery,
		SAMCONMotionClipProcessor::PostSingleSampleCallback pPostSingleSampleCallback,
		SAMCONMotionClipProcessor::PostSingleIterationCallback pPostSingleIterationCallback,
		int maxNumOfTrials, int numSamplesPerIteration,
		int iterationDurationAsIntMultipleOfSimStep,
		int auxNumberOfIterationsPerTrialLimit,
		apReal characterHeight,
		bool pickDiverse, int numOfEliteSamplesToPick,
		apReal wp, apReal wr, apReal we, apReal wb);

	EXPORT_API acFeedbackPolicyVector acCreateFeedbackPolicyVector(apByte *pPolicyVectorData, int policyType, int numBytesInBuffer);
	EXPORT_API void acDeleteFeedbackPolicyVector(acFeedbackPolicyVector pPolicyVector);
	EXPORT_API int acFeedbackPolicyVectorGetNecessaryRawClipBufferByteSize(acFeedbackPolicyVector pPolicyVector);
	EXPORT_API void acFeedbackPolicyVectorDumpToRawBuffer(acFeedbackPolicyVector pPolicyVector, apByte *pPolicyVectorData);
	EXPORT_API void acFeedbackPolicyVectorSetNumPolicies(acFeedbackPolicyVector pPolicyVector, int numPolicies);
	EXPORT_API int acFeedbackPolicyVectorGetNumPolicies(acFeedbackPolicyVector pPolicyVector);

	// 3 Point Tracking
	EXPORT_API void acUpdateControllerState(
		acController controller,
		biVector3* headPos, biQuaternion* headRotation,
		biVector3* lHandPos, biQuaternion* lHandRotation,
		biVector3* rHandPos, biQuaternion* rHandRotation,
        biVector3* lFootPos, biQuaternion* lFootRotation,
        biVector3* rFootPos, biQuaternion* rFootRotation,
        biVector3* rootPos, biQuaternion* rootRotation,
        apReal limbTrackingKp,
        apReal limbTrackingKd,
        apReal limbMaxTrackingForce,
        apReal deathThresholdLegSwing,
        apReal deathThresholdGRF,
        bool kickIfDead,
        bool antiLegCrossing,
        bool stepRelativeToCOM,
        bool stepRelativeToRoot
	);

    EXPORT_API void acUpdateExtraControllerTrackingInfo(
        acController controller,
        biVector3* lElbowPos, biQuaternion* lElbowRotation,
        biVector3* rElbowPos, biQuaternion* rElbowRotation,
        biVector3* lShoulderPos, biQuaternion* lShoulderRotation,
        biVector3* rShoulderPos, biQuaternion* rShoulderRotation,
        biVector3* lKneePos, biQuaternion* lKneeRotation,
        biVector3* rKneePos, biQuaternion* rKneeRotation,
        biVector3* lHipPos, biQuaternion* lHipRotation,
        biVector3* rHipPos, biQuaternion* rHipRotation
    );

    EXPORT_API acController acCreateSinglePoseTrackingController(
        apArticulationDynamicsWorldHandle world,
        apArticulationBaseHandle base);

    EXPORT_API void acSetAdditionalPoseControllerMotorsOptions(acController controller, bool jointMotorCfmHackEnabled, bool jointMotorFixedSphericalJointAxesEnabled);

    // record the character state into the motion clip
    EXPORT_API void acRecordCharacterStateIntoMotionClip(apArticulationBaseHandle base, acMotionClip pClip);
    // record the target pose into the motion clip
    EXPORT_API void acRecordTargetPoseIntoMotionClip(acController controller, acMotionClip pClip);

    EXPORT_API int acGetNumControllerBodySupportTorques(acController controllerHandle);
    EXPORT_API int acGetControllerBodySupportTorques(acController controllerHandle, biVector3 * buffer, int bufferSize);

    EXPORT_API int acGetGRFSolverMode(acController controllerHandle);
    EXPORT_API void acSetGRFSolverMode(acController controllerHandle, int useLCPQP);


#ifdef __cplusplus
}
#endif

#endif
