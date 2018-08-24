/* Articulated Physics Engine C API */

#ifndef APE_API_H
#define APE_API_H

#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

template<typename T> struct isAPEHandle { static const bool value = false; };

#define AP_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name; \
template<> struct isAPEHandle<name##__*> { static const bool value = true; };

// There is no static_assert, so do compile time assertion like this
#define COMPILE_TIME_ASSERT( x ) \
switch ( (unsigned char)(x) ) \
{ \
case 0: \
break; \
case ( x ): \
break; \
}

/**	Particular physics SDK (C-API) */
AP_DECLARE_HANDLE(apPhysicsSdkHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
AP_DECLARE_HANDLE(apDynamicsWorldHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
AP_DECLARE_HANDLE(apArticulationDynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apRigidBodyHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apJointFeedbackHandle);

/** Articulation base that can be part of a Articulation Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apArticulationBaseHandle);

/** Articulation link that can be part of a Articulation Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apArticulationLinkHandle);

/** A kernel object that contains the skeleton information of an articulation (C-API)*/
AP_DECLARE_HANDLE(apSkeletonInfo);

/** Articulation link motor that can be part of a Articulation Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apArticulationConstraint);

/** Mobilizer data for the per-DOF motor / spring / limit properties (C-API)*/
AP_DECLARE_HANDLE(apMobilizer);

/** Constraint that can be part of a Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apTypedConstraint);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
AP_DECLARE_HANDLE(apCollisionShapeHandle);

/** Triangle Mesh interface (C-API)*/
AP_DECLARE_HANDLE(apMeshInterfaceHandle);

/** Static concave triangle mesh (C-API) */
AP_DECLARE_HANDLE(apTriangleMeshHandle);

/** Broadphase Scene/Proxy Handles (C-API)*/
AP_DECLARE_HANDLE(apCollisionBroadphaseHandle);
AP_DECLARE_HANDLE(apBroadphaseProxyHandle);
AP_DECLARE_HANDLE(apCollisionWorldHandle);

/** Rope Joint **/
AP_DECLARE_HANDLE(apRopeJointHandle);

/** Wind generator can act on bodies in a Dynamics World (C-API)*/
AP_DECLARE_HANDLE(apWindGeneratorHandle);

#ifdef __cplusplus
extern "C" {
#endif

    typedef void(*DebugLogFuncPtr)(const char *);
    typedef void(*DrawPointFuncPtr)(apReal x, apReal y, apReal z, apReal size, apReal r, apReal g, apReal b);
    typedef void(*DrawLineFuncPtr)(apReal x1, apReal y1, apReal z1, apReal x2, apReal y2, apReal z2, apReal width, apReal r, apReal g, apReal b);

    /** Set Unity debug log callback function pointer  (MacOS only) */
    EXPORT_API void SetDebugFunction(DebugLogFuncPtr fp);
    EXPORT_API void SetErrorFunction(DebugLogFuncPtr fp);

    EXPORT_API void SetDrawFunctions(DrawPointFuncPtr drawPoint, DrawLineFuncPtr drawLine);


	/**
	 Create and Delete a Physics SDK
	 */
	EXPORT_API int apNumOfBytesPerReal(void);
    EXPORT_API apPhysicsSdkHandle apGetPhysicsSdk(void); //this could be also another sdk, like ODE, PhysX etc.
    EXPORT_API apPhysicsSdkHandle apNewPhysicsSdk(void); //this could be also another sdk, like ODE, PhysX etc.
	EXPORT_API void apDeletePhysicsSdk(apPhysicsSdkHandle physicsSdk);

    /*
     * mathematics
     */
    EXPORT_API void apTransformInverse(biVector3 *inPos, biQuaternion *inRotation, biVector3 *outPos, biQuaternion *outRotation);
    EXPORT_API void apTransformTimesTransform(biVector3 *inPos1, biQuaternion *inRotation1, biVector3 *inPos2, biQuaternion *inRotation2, biVector3 *outPos, biQuaternion *outRotation);
    EXPORT_API void apTransformInverseTimesTransform(biVector3 *inPos1, biQuaternion *inRotation1, biVector3 *inPos2, biQuaternion *inRotation2, biVector3 *outPos, biQuaternion *outRotation);
    EXPORT_API void apTransformTimesTransformInverse(biVector3 *inPos1, biQuaternion *inRotation1, biVector3 *inPos2, biQuaternion *inRotation2, biVector3 *outPos, biQuaternion *outRotation);
	
	/* Dynamics World */
	EXPORT_API apArticulationDynamicsWorldHandle apCreateDynamicsWorld(apPhysicsSdkHandle physicsSdkHandle,
		int numIterations, apReal convexMeshDistanceMargin, apReal defaultErp, bool restorableSimulationStateMode);
	EXPORT_API void apDeleteDynamicsWorld(apDynamicsWorldHandle world);
	EXPORT_API void apSetGravity(apDynamicsWorldHandle world, const biVector3 *gravity);
	EXPORT_API void apGetGravity(apDynamicsWorldHandle world, biVector3 *gravity);
	EXPORT_API void apSetDynamicsWorldSolverIterationsNumber(apDynamicsWorldHandle world, int numIterations);
	
	EXPORT_API void apResetDynamicsWorldCaches(apDynamicsWorldHandle world);
	EXPORT_API void apStepSimulation(apDynamicsWorldHandle world, apReal timeStep, int maxSubSteps, apReal fixedTimeStep);
	
    EXPORT_API void apSetConvexMeshMargin(apCollisionShapeHandle cshape, apReal margin);

    EXPORT_API apWindGeneratorHandle apAddSimpleWindGenerator(const biVector3* windForce);
    EXPORT_API void apRemoveWindGenerator(apWindGeneratorHandle windGeneratorHandle);
    EXPORT_API void apSetWindForce(apWindGeneratorHandle windGeneratorHandle, const biVector3* windForce);
    EXPORT_API void apInstallWindForceSharedMemoryBuffer(apWindGeneratorHandle windGeneratorHandle, biVector3* windForce);
    EXPORT_API void apUninstallWindForceSharedMemoryBuffer(apWindGeneratorHandle windGeneratorHandle);

	// TBD: Merge apAddRigidBody with apCreateRigidBody since they seem to be always called consecutively
	EXPORT_API void apAddRigidBody(apDynamicsWorldHandle world, apRigidBodyHandle object, bool collidable, int layerID);
	// TBD: Merge apRemoveRigidBody with apDeleteRigidBody since they seem to be always called consecutively
	EXPORT_API void apRemoveRigidBody(apDynamicsWorldHandle world, apRigidBodyHandle object);
	
	EXPORT_API void apAddConstraint(apDynamicsWorldHandle world, apTypedConstraint constraint, bool disableSelfCollision);
	EXPORT_API void apRemoveConstraint(apDynamicsWorldHandle world, apTypedConstraint constraint);
    EXPORT_API void apEnableCollisionBetweenConstrainedRigidBodies(apDynamicsWorldHandle world, apTypedConstraint constraint);
    EXPORT_API void apDisableCollisionBetweenConstrainedRigidBodies(apDynamicsWorldHandle world, apTypedConstraint constraint);
	
	EXPORT_API void apSetCollisionInfoBuffers(CollisionInfo* pCollisionInfoBuffer, unsigned int iCollisionInfoBufferSize
											  , ContactPointInfo* pContactPointInfoBuffer, unsigned int iContactPointInfoBufferSize);
	EXPORT_API void apAddListenerRigidBody(apRigidBodyHandle rigidbody);
	EXPORT_API void apRemoveListenerRigidBody(apRigidBodyHandle rigidbody);
	EXPORT_API void apAddListenerArticulation(apArticulationBaseHandle base);
	EXPORT_API void apAddListenerArticulationLink(apArticulationBaseHandle base, int linkIndex);
	EXPORT_API void apRemoveListenerArticulation(apArticulationBaseHandle base);
	EXPORT_API void apRemoveListenerArticulationLink(apArticulationBaseHandle base, int linkIndex);
	EXPORT_API void apSetRigidBodyLayerID(apRigidBodyHandle rigidBody, int layer);
	EXPORT_API void apSetArticulationLayerID(apArticulationBaseHandle base, int layer);
	EXPORT_API void apSetLinkLayerID(apArticulationBaseHandle base, int linkIndex, int layer);
	EXPORT_API void apResetCollisionCacheForRigidBody(apDynamicsWorldHandle world, apRigidBodyHandle rigidBody);
	EXPORT_API void apResetCollisionCacheForArticulation(apDynamicsWorldHandle world, apArticulationBaseHandle base);
	EXPORT_API void apResetCollisionCacheForLink(apDynamicsWorldHandle world, apArticulationBaseHandle base, int linkIndex);
	EXPORT_API void apSetLayerCollision(int layer0, int layer1, bool collide);
	EXPORT_API void apArticulationEnableSelfCollision(apArticulationBaseHandle base, bool enableSelfCollision);
    EXPORT_API void apArticulationSetPlaneOfSymmetric(apArticulationBaseHandle base, int planeOfSymmetry, biVector3 *normalOfPlaneOfSymmetry);

	/* Rigid Body  */
	EXPORT_API apRigidBodyHandle apCreateRigidBody(const apReal *masses, const apReal *inertias, biVector3 *moi,
												   apCollisionShapeHandle cshape,
												   const biVector3 *position, const biQuaternion *rotation,
												   const biVector3 *linearVelocity, const biVector3 *angularVelocity,
												   apReal friction, apReal rollingFriction, int frictionCombineMode,
                                                   apReal restitution, int restitutionCombineMode);
	
	EXPORT_API void apDeleteRigidBody(apRigidBodyHandle body);
	EXPORT_API void apSetRigidBodyMass(apDynamicsWorldHandle world, apRigidBodyHandle rigidBody, apReal mass);
	EXPORT_API void apSetRigidBodyVelocity(apRigidBodyHandle body, apReal x, apReal y, apReal z,
										   apReal wX, apReal wY, apReal wZ);
	
	EXPORT_API bool apInstallRigidBodySharedMemoryBuffers(
		apRigidBodyHandle body,
		biVector3* position, biQuaternion* rotation,
		biVector3* accumulatedForce, biVector3* accumulatedTorque,
		int* worldIndex, apReal* velocityVars,
		apReal* drag, apReal *angularDrag);

	EXPORT_API void apSetRigidBodyKinematic(apDynamicsWorldHandle world, apRigidBodyHandle rigidBody, bool flag, apReal mass);
	
	
	/* Joint */
	EXPORT_API apTypedConstraint apCreateHingeConstraint	(apRigidBodyHandle pBodyA, apRigidBodyHandle pBodyB,
															 const biVector3* pivotA, const biVector3* pivotB,
															 const biVector3* axisA, const biVector3* axisB,
															 apReal breakingImpulse, int overrideIterations,
															 apJointFeedbackHandle apFB);
	EXPORT_API apTypedConstraint apCreateP2PConstraint	  (apRigidBodyHandle pBodyA, apRigidBodyHandle pBodyB,
															 const biVector3* pivotA, const biVector3* pivotB,
															 apReal breakingImpulse, int overrideIterations,
															 apJointFeedbackHandle apFB);
	EXPORT_API apTypedConstraint apCreateFixedConstraint	(apRigidBodyHandle pBodyA, apRigidBodyHandle pBodyB,
															 const biVector3* pivotA, const biVector3* pivotB,
															 apReal breakingImpulse, int overrideIterations,
															 apJointFeedbackHandle apFB);
	EXPORT_API void apSetP2PConstraintPivotB(apTypedConstraint constraint, apReal x, apReal y, apReal z);
	EXPORT_API void apSetFixedConstraintPivotA(apTypedConstraint constraint, apReal x, apReal y, apReal z);
	EXPORT_API void apSetConstraintBreakingImpulse (apTypedConstraint constraint, apReal breakingImpulse);

	EXPORT_API void apDeleteConstraint(apTypedConstraint constraint);
	
	/* Collision Shape definition */
	
	EXPORT_API apCollisionShapeHandle apNewSphereShape(apReal radius);
	EXPORT_API apCollisionShapeHandle apNewBoxShape(apReal x, apReal y, apReal z);
	EXPORT_API apCollisionShapeHandle apNewCapsuleShape(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewCapsuleShapeX(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewCapsuleShapeZ(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewConeShape(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewConeShapeX(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewConeShapeZ(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewCylinderShape(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewCylinderShapeX(apReal radius, apReal height);
	EXPORT_API apCollisionShapeHandle apNewCylinderShapeZ(apReal radius, apReal height);
	
	/* Convex Hull */
	EXPORT_API apCollisionShapeHandle apNewConvexHullShape(void);
	EXPORT_API void apAddVertex(apCollisionShapeHandle convexHull, apReal x,apReal y,apReal z);
	
	EXPORT_API void apDeleteShape(apCollisionShapeHandle shape);
    EXPORT_API int apGetShapeType(apCollisionShapeHandle shape);
	
	/* Static Concave Triangle Mesh */
	EXPORT_API apMeshInterfaceHandle apNewMeshInterface(void);
	EXPORT_API apTriangleMeshHandle apNewTriangleMesh();
	EXPORT_API void apAddTriangle(apTriangleMeshHandle mesh, const biVector3* v0, const biVector3* v1, const biVector3* v2);
	EXPORT_API apCollisionShapeHandle apNewConcaveMeshShape(apTriangleMeshHandle mesh, bool anchored);
	EXPORT_API void apDeleteConcaveMeshShape(apCollisionShapeHandle handle, bool anchored);

	EXPORT_API apCollisionShapeHandle apNewHeightMapShape(int widthSamples, int lengthSamples, apReal* heightMap,
		apReal maxHeight);
	EXPORT_API apCollisionShapeHandle apNewHeightMapShapeExtended(int widthSamples, int lengthSamples, apReal* heightMap,
		apReal minHeight, apReal maxHeight, bool flipQuadEdges);

	/* Compound Shape */
	EXPORT_API apCollisionShapeHandle apNewCompoundShape(void);
	EXPORT_API void apAddChildShape(apCollisionShapeHandle compoundShape, apCollisionShapeHandle childShape,
											const biVector3* childPos, const biQuaternion* childOrn);
	EXPORT_API void apRemoveChildShape(apCollisionShapeHandle compoundShape, apCollisionShapeHandle childShape);
	EXPORT_API void apDeleteCompoundShape(apCollisionShapeHandle handle);
	
	EXPORT_API void apSetScaling(apCollisionShapeHandle shape, apReal x, apReal y, apReal z);
	
	
	/* SOLID has Response Callback/Table/Management */
	/* PhysX has Triggers, User Callbacks and filtering */
	/* ODE has the typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2); */
	
	/*	typedef void apUpdatedPositionCallback(void* userData, apRigidBodyHandle	rbHandle, const biVecto3* pos); */
	/*	typedef void apUpdatedOrientationCallback(void* userData, apRigidBodyHandle	rbHandle, const biQuaternion* orientation); */
	
	/* get world transform */
	EXPORT_API void apGetRigidBodyPosition(apRigidBodyHandle object, biVector3* position, bool isPosInCoM);
    EXPORT_API void apGetRigidBodyOrientation(apRigidBodyHandle object, biQuaternion* orientation, bool isPosInCoM);
    EXPORT_API void apGetRigidBodyTransform(apRigidBodyHandle object, biVector3* position, biQuaternion* orientation, bool isPosInCoM);
	
	/* set world transform (position/orientation) */
	EXPORT_API void apSetRigidBodyPosition(apRigidBodyHandle object, apReal x, apReal y, apReal z,
	                                       apReal qx, apReal qy, apReal qz, apReal qw, bool isInPosInCoM);
	EXPORT_API void apSetRigidBodyTransform(apRigidBodyHandle object, biVector3 *position, biQuaternion *orientation, bool isPosInCoM);
	
	EXPORT_API bool apRayCast(apDynamicsWorldHandle world, const biVector3 *rayFrom, const biVector3 *rayTo,
							  apRigidBodyHandle* hitBody, apArticulationBaseHandle* hitTntBase, int* hitTntLinkIndex,
							  biVector3* hit, biVector3* hitNormal);
	
	EXPORT_API bool apRayCast2(apDynamicsWorldHandle world, const biVector3 *rayFrom, const biVector3 *direction,
							   apReal maxDistance, int layerMask,
							   biVector3* hitLocation, biVector3* hitNormal, biVector3* hitBaryCentric, int* triangleIndex,
							   int* index, bool* isRigidBody, apRigidBodyHandle* hitBody, apArticulationBaseHandle* hitTntBase, int* hitTntLinkIndex);
	
    EXPORT_API void apEnableStaticMeshCollisionSmoothing(bool shouldSmoothOut);

    EXPORT_API apSkeletonInfo apCreateSkeletonInfo(int numLinks, const char *baseName, const char **linkNames);
    EXPORT_API apSkeletonInfo apCreateSkeletonInfoFromArticulation(apArticulationBaseHandle base);
    EXPORT_API void apDeleteSkeletonInfo(apSkeletonInfo skeletonInfo);


	/* Featherstone API */
	/**
	 * @remark maxAppliedImpulse is kept for backward compatibility of the API and currently doesn't do anything
	 */
	EXPORT_API apArticulationBaseHandle apCreateArticulationBase(int numLinks, 
                                                                 const char *articulationName, const char *baseName,
                                                                 const apReal* masses, const apReal* inertias,
																 biVector3 *moi,
																 apCollisionShapeHandle cshape, bool canSleep, bool multiDof,
																 bool selfCollision, bool highDefIntegrator, bool useGlobalVel,
																 int simulationFreqMultiplier,
																 apReal linearDamping, apReal angularDamping,
																 apReal maxAppliedImpulse, apReal maxCoordinateVelocity,
																 apReal friction, apReal rollingFriction, int frictionCombineMode,
                                                                 apReal restitution, int restitutionCombineMode,
																 const biVector3 *position, const biQuaternion *rotation);


    EXPORT_API void apSetArticulationBaseRawState(apArticulationBaseHandle base,
                                                const biVector3 *basePos, const biQuaternion *baseOri,
                                                const biVector3 *baseVel, const biVector3 *baseAngVel,
                                                bool isInBasePosAndLinVelAtCoM, bool shouldUpdateTransformsAndSensors);

	EXPORT_API void apSetArticulationFullRawState(apArticulationBaseHandle base,
												const biVector3 *basePos, const biQuaternion *baseOri,
												const biVector3 *baseVel, const biVector3 *baseAngVel,
												const apReal *q, const apReal *qd, bool isInBasePosAndLinVelAtCoM, bool shouldUpdateTransformsAndSensors);

    EXPORT_API void apGetArticulationBaseRawState(apArticulationBaseHandle base,
                                                biVector3 *basePos, biQuaternion *baseOri,
                                                biVector3 *baseVel, biVector3 *baseAngVel,
                                                bool isOutBasePosAndLinVelAtCoM);

	EXPORT_API void apGetArticulationFullRawState(apArticulationBaseHandle base,
												biVector3 *basePos, biQuaternion *baseOri,
												biVector3 *baseVel, biVector3 *baseAngVel,
												apReal *q, apReal *qd, bool isOutBasePosAndLinVelAtCoM);

	EXPORT_API int apGetArticulationTotalDofCountExcludingBase(apArticulationBaseHandle base);
	EXPORT_API int apGetArticulationTotalRawPosVarCountExcludingBase(apArticulationBaseHandle base);

	EXPORT_API void apSetLinkRawPosVariable(apMobilizer mobilizer, int posVarIndex, apReal posVar, bool shouldUpdateTransformsAndSensors);
	EXPORT_API apReal apGetLinkRawPosVariable(apMobilizer mobilizer, int posVarIndex);
	EXPORT_API void apSetLinkRawVelocityVariable(apMobilizer mobilizer, int dofIndex, apReal vel, bool shouldUpdateTransformsAndSensors);
	EXPORT_API apReal apGetLinkRawVelocityVariable(apMobilizer mobilizer, int dofIndex);

	EXPORT_API void apUpdateArticulationTransformsAndSensors(apArticulationBaseHandle base);

    EXPORT_API void apGetArticulationBaseTransform(apArticulationBaseHandle base, biVector3* position, biQuaternion* orientation);
    EXPORT_API void apGetArticulationLinkTransform(apMobilizer mobilizer, biVector3* position, biQuaternion* orientation);

    EXPORT_API void apGetArticulationLinkLinearVelocity(apMobilizer mobilizer, biVector3* linearVelocity);
    EXPORT_API void apGetArticulationLinkAngularVelocity(apMobilizer mobilizer, biVector3* angularVelocity);

	EXPORT_API void apSetBaseMass(apArticulationBaseHandle base, apReal mass);
	EXPORT_API void apSetLinkMass(apMobilizer mobilizer, apReal mass);

	EXPORT_API void apSetBaseCollisionShape(apArticulationBaseHandle base, apCollisionShapeHandle cshape);
    EXPORT_API apCollisionShapeHandle apGetBaseCollisionShape(apArticulationBaseHandle base);
	EXPORT_API void apComputeBaseMoIFromCollisionShape(
		apArticulationBaseHandle base,
		const apReal *masses, const apReal* inertias,
		apReal *mass, biVector3* moi);

	EXPORT_API void apSetLinkCollisionShape(apMobilizer mobilizer, apCollisionShapeHandle cshape);
	EXPORT_API void apComputeLinkMoIFromCollisionShape(
		apMobilizer mobilizer,
		const apReal *masses, const apReal* inertias,
		const biVector3 *pivotA, const biVector3 *axisA,
		const biVector3 *pivotB, const biVector3 *axisB,
		apReal *mass, biVector3* moi);

	EXPORT_API void apSetRigidBodyCollisionShape(apRigidBodyHandle bodyHandle, apCollisionShapeHandle cshape);
    EXPORT_API apCollisionShapeHandle apGetRigidBodyCollisionShape(apRigidBodyHandle bodyHandle);
	EXPORT_API void apComputeRigidBodyMoIFromCollisionShape(
		apRigidBodyHandle bodyHandle,
		const apReal *masses, const apReal* inertias,
		apReal *mass, biVector3* moi);

	EXPORT_API void apSetLinkBreakingImpulseThreshold(apMobilizer mobilizer, apReal breakingImpulseThreshold);
	EXPORT_API void apComputeLinkWorldVelocity(apMobilizer mobilierData, biVector3* linVel, biVector3* angVel, bool useCached);
	EXPORT_API int apGetLinkDofCount(apMobilizer mobilierData);
	EXPORT_API int apGetLinkRawPosVarCount(apMobilizer mobilierData);
	
	EXPORT_API apMobilizer apSetupArticulationLink(apArticulationBaseHandle base, int jointType, bool parentCollision,
	                                               int myIndex, int parentIndex, int mirrorIndex,
	                                               const char* name, const apReal *masses, const apReal *inertias,
	                                               biVector3 *moi,
	                                               apCollisionShapeHandle cshape, apReal friction, apReal rollingFriction,
	                                               int frictionCombineMode, apReal restitution,
	                                               int restitutionCombineMode, const biVector3 *position,
	                                               const biQuaternion *rotation, const biVector3 *pivotA,
	                                               const biVector3 *axisA, const biVector3 *pivotB,
	                                               const biVector3 *axisB, const apReal* springStiffness, const apReal* springDamping);
    EXPORT_API apMobilizer apSetupVirtualArticulationLink(apArticulationBaseHandle base, int jointType, bool parentCollision,
                                                          int myIndex, int parentIndex, int mirrorIndex,
                                                          const char* name,
                                                          apCollisionShapeHandle cshape, apReal friction, apReal rollingFriction,
                                                          int frictionCombineMode, apReal restitution,
                                                          int restitutionCombineMode, const biVector3 *position,
                                                          const biQuaternion *rotation, const biVector3 *pivotA,
                                                          const biVector3 *axisA, const biVector3 *pivotB,
                                                          const biVector3 *axisB, const apReal* springStiffness, const apReal* springDamping);

    EXPORT_API void apEnableCollisionBetweenLinkedObjects(apArticulationBaseHandle base, int linkIndex);
    EXPORT_API void apDisableCollisionBetweenLinkedObjects(apArticulationDynamicsWorldHandle world, apArticulationBaseHandle base, int linkIndex);
	
	EXPORT_API void apBreakArticulationLinkOff(apMobilizer mobilizer);

	EXPORT_API bool apInstallArticulationLinkSharedMemoryBuffers(
        apArticulationBaseHandle base,
        int linkIndex,
		biVector3* position, biQuaternion* rotation,
		biVector3* accumulatedForce, biVector3* accumulatedTorque,
		int* worldIndex, apReal* positionVars, apReal* velocityVars,
		apReal* drag, apReal *angularDrag,
		apJointFeedbackHandle feedback);

	EXPORT_API bool apSetupFixedLink(apArticulationBaseHandle base,
	                                 bool parentCollision, int myIndex, int parentIndex, int mirrorIndex,
	                                 const char* name, const apReal *masses, const apReal *inertias,
	                                 biVector3* moi, apCollisionShapeHandle cshape, apReal friction, apReal rollingFriction,
	                                 int frictionCombineMode, apReal restitution, int restitutionCombineMode, const biVector3 *position, const biQuaternion *rotation);
	
	// Add articulation to world
	EXPORT_API void apAddArticulation(apArticulationDynamicsWorldHandle world, apArticulationBaseHandle object, const bool* collidable, int layerID);
	EXPORT_API void apAddOutOfSimArticulation(apArticulationBaseHandle object);
	// Remove articulation from world
	EXPORT_API void apRemoveArticulation(apArticulationDynamicsWorldHandle world, apArticulationBaseHandle object);
	// Free the memory of articulation
	EXPORT_API void apDeleteArticulation(apArticulationBaseHandle base);
	
	EXPORT_API bool apInstallArticulationBaseSharedMemoryBuffers(
		apArticulationBaseHandle base,
		biVector3* position, biQuaternion* rotation,
		biVector3* accumulatedForce, biVector3* accumulatedTorque,
		int* worldIndex, apReal* velocityVars,
		apReal* drag, apReal *angularDrag);

	EXPORT_API void apSetArticulationKinematic(apArticulationBaseHandle base, bool flag, apReal mass);
	
	EXPORT_API void apLinkPosToWorld(apArticulationBaseHandle base, int link, const biVector3* localPos, biVector3* worldPos);
	EXPORT_API void apLinkDirToWorld(apArticulationBaseHandle base, int link, const biVector3* localDir, biVector3* worldDir);
	EXPORT_API void apWorldPosToLink(apArticulationBaseHandle base, int link, const biVector3* worldPos, biVector3* localPos);
	EXPORT_API void apWorldDirToLink(apArticulationBaseHandle base, int link, const biVector3* worldDir, biVector3* localDir);


    EXPORT_API void apComputeArticulationCoMPosition(apArticulationBaseHandle base, biVector3* comPos);
    EXPORT_API void apComputeArticulationCoMVelocity(apArticulationBaseHandle base, biVector3* comVel);
    EXPORT_API void apComputeArticulationAngularMomentum(apArticulationBaseHandle base, biVector3* angMom);
    EXPORT_API void apComputeArticulationHeadingTransform(apArticulationBaseHandle base, biVector3* rootPlannarLocation, biQuaternion *rootHeadingRotation);
	
	EXPORT_API apMobilizer apAddMotor(apArticulationDynamicsWorldHandle world,
										apArticulationBaseHandle base,
										int link, int dof,
										apReal desiredVelocity, apReal desiredPosition,
										apReal maxMotorImpulse,
										bool isPositional,
                                        apReal positionLockThreshold, bool shouldUseAutomaticPositionLockThreshold);
	EXPORT_API void apDeleteMotor(apArticulationDynamicsWorldHandle world, apMobilizer motor, int dof);
	EXPORT_API void apSetMotorDesiredSpeed(apMobilizer mobilierData, int dof, apReal speed);
	EXPORT_API void apSetMotorMaxForce(apMobilizer mobilierData, int dof, apReal maxForce);
	EXPORT_API void apSetMotorIsPostional(apMobilizer mobilizer, int dof, bool positional);
	EXPORT_API void apSetMotorDesiredPosition(apMobilizer mobilizer, int dof, apReal position);
	EXPORT_API void apSetMotorPositionLockThreshold(apMobilizer mobilizer, int dof, apReal threshold);
    EXPORT_API void apSetUseAutomaticPositionLockThreshold(apMobilizer mobilizer, int dof, bool shouldUseAutomaticPositionLockThreshold);
	
	EXPORT_API apMobilizer apAddJointLimits(apArticulationDynamicsWorldHandle world,
											apArticulationBaseHandle base,
											int link, int dof, apReal limitLo, apReal limitHi,
											apReal maxForce);
	EXPORT_API void apDeleteJointLimits(apArticulationDynamicsWorldHandle world, apMobilizer limits, int dof);
	EXPORT_API void apSetJointLimitLo(apMobilizer mobilierData, int dof, apReal limitLo);
	EXPORT_API void apSetJointLimitHi(apMobilizer mobilierData, int dof, apReal limitHi);
	EXPORT_API void apSetJointLimitMaxForce(apMobilizer mobilierData, int dof, apReal maxForce);
	
	EXPORT_API void apSetSpringStiffness(apMobilizer mobilizer, int dof, apReal stiffness);
	EXPORT_API void apSetSpringDamping(apMobilizer mobilizer, int dof, apReal damping);
	EXPORT_API void apSetSpringNeutralPoint(apMobilizer mobilizer, int dof, apReal neutralPoint);

	EXPORT_API void apSetContinuousForceActuator(apMobilizer mobilizer, int dof, apReal force);
	
	EXPORT_API apArticulationConstraint apAddArticulationP2PLink(apArticulationDynamicsWorldHandle world,
																 apArticulationBaseHandle bodyA, int linkA,
																 apArticulationBaseHandle bodyB, int linkB,
																 const biVector3* pivotInA, const biVector3* pivotInB,
																 apReal maxImpulse, apReal breakingImpulse,
																 apJointFeedbackHandle feedback);
	EXPORT_API apArticulationConstraint apAddArticulationRigidBodyP2PLink(apArticulationDynamicsWorldHandle world,
																		  apArticulationBaseHandle bodyA, int linkA,
																		  apRigidBodyHandle bodyB,
																		  const biVector3* pivotInA, const biVector3* pivotInB,
																		  apReal maxImpulse, apReal breakingImpulse,
																		  apJointFeedbackHandle feedback);
	EXPORT_API void apSetP2PLinkBreakingImpulse(apArticulationConstraint constraint, apReal breakingImpulse);
	EXPORT_API void apSetP2PlinkPivotB(apArticulationConstraint constraint, apReal x, apReal y, apReal z);
	EXPORT_API apArticulationConstraint apAddArticulationFixedConstraint(apArticulationDynamicsWorldHandle world,
																 apArticulationBaseHandle bodyA, int linkA,
																 apArticulationBaseHandle bodyB, int linkB,
																 const biVector3* pivotInA, const biVector3* pivotInB,
																 apReal maxImpulse, apReal breakingImpulse,
																 apJointFeedbackHandle feedback);
	EXPORT_API apArticulationConstraint apAddArticulationRigidBodyFixedConstraint(apArticulationDynamicsWorldHandle world,
																		  apArticulationBaseHandle bodyA, int linkA,
																		  apRigidBodyHandle bodyB,
																		  const biVector3* pivotInA, const biVector3* pivotInB,
																		  apReal maxImpulse, apReal breakingImpulse,
																		  apJointFeedbackHandle feedback);
	EXPORT_API void apSetFixedConstraintBreakingImpulse(apArticulationConstraint constraint, apReal breakingImpulse);
    EXPORT_API void apEnableCollisionBetweenConstrainedObjects(apArticulationDynamicsWorldHandle world, apArticulationConstraint constraint);
    EXPORT_API void apDisableCollisionBetweenConstrainedObjects(apArticulationDynamicsWorldHandle world, apArticulationConstraint constraint);
	EXPORT_API void apRemoveArticulationConstraint(apArticulationDynamicsWorldHandle world, apArticulationConstraint constraint);
	EXPORT_API void apDeleteArticulationConstraint(apArticulationDynamicsWorldHandle world, apArticulationConstraint constraint);
	
	// rope joint
	EXPORT_API apRopeJointHandle apAddRopeJointForRigidBodyAndRigidBody(apRigidBodyHandle body1, biVector3* pivotA,
																		apRigidBodyHandle body2, biVector3* pivotB,
																		apReal stiffness, apReal damping,
																		apReal* maxLength, apReal* maxForce, int* broken,
																		apReal* worlVel);
	
	EXPORT_API apRopeJointHandle apAddRopeJointForRigidBodyAndLink(apRigidBodyHandle body, biVector3* pivotA,
																   apArticulationBaseHandle base,
																   int linkIndex,	// negative linkIndex indicates this is base
																   biVector3* pivotB,
																   apReal stiffness, apReal damping,
																   apReal* maxLength, apReal* maxForce, int* broken,
																   apReal* worlVel);
	
	EXPORT_API apRopeJointHandle apAddRopeJointForLinkAndLink(apArticulationBaseHandle base1, int linkIndex1, biVector3* pivotA,
															  apArticulationBaseHandle base2, int linkIndex2, biVector3* pivotB,
															  apReal stiffness, apReal damping,
															  apReal* maxLength, apReal* maxForce, int* broken,
															  apReal* worlVel); // negative linkIndex indicates this is base
	
	
	EXPORT_API void apRemoveRopeJoint(apRopeJointHandle ropeJoint);
	
	// trigger status
	EXPORT_API void apSetRigidBodyTriggerStatus(apArticulationBaseHandle rigidBody, bool flag);
	EXPORT_API void apSetLinkTriggerStatus(apArticulationBaseHandle a_Base, int linkIndex, bool flag);
	
	//overlap sphere
	EXPORT_API int apOverlapSphere(apDynamicsWorldHandle world, biVector3* origin, apReal radius, int collisionFilterGroup, int maxNumOfColliders, int* overlappingColliderWorldIndexArray);
	
	//add force
	EXPORT_API void apAddOneTimeForceToRigidBody(apRigidBodyHandle rigidBody, biVector3* force, biVector3* posWorld, bool ignoreWorldPos);
	EXPORT_API void apAddOneTimeForceToArticulation(apArticulationBaseHandle base, int linkIndex, biVector3* force, biVector3* posWorld, bool ignoreWorldPos);
	EXPORT_API void apAddOneTimeTorqueToRigidBody(apRigidBodyHandle rigidBody, biVector3* torque);
	EXPORT_API void apAddOneTimeTorqueToArticulation(apArticulationBaseHandle base, int linkIndex, biVector3* torque);
	
	//statistics
	EXPORT_API void apSetStatsBuffer(SPhysicsStatsInfo* a_pPhysicsStatsInfo);

	EXPORT_API void apSetArticulationNonSimulated(apArticulationBaseHandle base, bool nonSimulated);

    EXPORT_API void apEnableDirectSolver(apArticulationDynamicsWorldHandle worldHandle, bool enable);
    EXPORT_API void apGetSolverFidelityIndexName(int fidelityIndex, int maxBufferSize, char* outBuffer);
    EXPORT_API void apSetDefaultSolverFidelityIndex(apArticulationDynamicsWorldHandle worldHandle, int fidelityIndex);
    EXPORT_API void apSetArticulationRequiredSolverFidelityIndex(apArticulationBaseHandle baseHandle, int fidelityIndex);
    EXPORT_API void apSetRigidBodyRequiredSolverFidelityIndex(apRigidBodyHandle rigidBody, int fidelityIndex);
    EXPORT_API int apGetNumberOfAvailableFidelityIndices();

    EXPORT_API void apSetArticulationRequiredFrequencyMultiplier(apArticulationBaseHandle baseHandle, int freqMult);
    EXPORT_API void apSetRigidBodyRequiredFrequencyMultiplier(apRigidBodyHandle rigidBody, int freqMult);

    EXPORT_API bool apDumpAvailableProfilingData(const char *pFile, bool resetProfilingData);
    EXPORT_API bool apDumpAvailableProfilingDataToBuffer(char *pBuffer, int bufferSize, int *bytesWritten, bool resetProfilingData);
    EXPORT_API int apGetSnapshot(apArticulationDynamicsWorldHandle worldHandle, apReal *pBuffer);

    EXPORT_API bool apIsRigidBodyAsleep(apRigidBodyHandle rigidBody);
    EXPORT_API bool apIsArticulationBaseAsleep(apArticulationBaseHandle base);
    EXPORT_API bool apIsArticulationLinkAsleep(apMobilizer mobilizer);

    EXPORT_API void apWakeUpRigidBody(apRigidBodyHandle rigidBody);
    EXPORT_API void apWakeUpArticulationBase(apArticulationBaseHandle base);
    EXPORT_API void apWakeUpArticulationLink(apMobilizer mobilizer);

    EXPORT_API void apActivateArticulation(apArticulationBaseHandle base);

    EXPORT_API void apEnableDebugVisualization(bool enable);
    EXPORT_API void apShowDebugVisualizationInDedicatedWindow(bool showDebugVisuWnd);
    // TODO: use camera settings from the editor (convenience) [example func proto]
    // EXPORT_API void apSetDebugCameraTransforms(biVector3 *up, biVector3 *target, apReal distance, apReal pitch, apReal yaw);

    // TODO: consider renaming 'data' -> 'lines points'? 'geom'?
    EXPORT_API void apUpdateDebugVisualizationDataFromWorld(apDynamicsWorldHandle world, bool resetBuffersFirst);
    EXPORT_API void apUpdateDebugVisualizationDataFromCollisionShape(apCollisionShapeHandle shapeHandle, biVector3* pos, biQuaternion* ori, bool resetBuffersFirst);

    // TODO: consider renaming apGetCurrentColliderLinesPoints -> apGetCurrentDebugVisualizationLinesPoints
    EXPORT_API int apGetCurrentColliderLinesPoints(apReal *colliderLinesPoints, apReal *colliderLinesPointsColors);

    EXPORT_API void apEnableConvexColliderVisualizationSimplification(apCollisionShapeHandle collisionShape, bool enable);

#ifdef __cplusplus
}
#endif


#endif //ARTICULATED_PHYSICS__C_API_H

