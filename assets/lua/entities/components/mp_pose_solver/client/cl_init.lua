--[[
    Copyright (C) 2023 Silverlan

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
]]

local Component = util.register_class("ents.MPPoseSolver", BaseEntityComponent)

local mediaExtensions = { "avi", "mp4", "mov" }
Component:RegisterMember("MediaSource", udm.TYPE_STRING, "", {
	specializationType = ents.ComponentInfo.MemberInfo.SPECIALIZATION_TYPE_FILE,
	onChange = function(c)
		c:UpdateMediaSource()
	end,
	metaData = {
		rootPath = "",
		extensions = mediaExtensions,
		stripExtension = true,
	},
})

function Component:Initialize()
	BaseEntityComponent.Initialize(self)

	self:SetTickPolicy(ents.TICK_POLICY_ALWAYS)
	self:BindEvent(ents.AnimatedComponent.EVENT_MAINTAIN_ANIMATIONS, "MaintainAnimations")

	local r = engine.load_library("mediapipe/pr_mediapipe")
	if r ~= true then
		self.m_errMsg = r
		pfm.log("Failed to load mediapipe module: " .. r, pfm.LOG_CATEGORY_PFM_GAME, pfm.LOG_SEVERITY_WARNING)
		self:GetEntity():RemoveSafely()
		return
	end
	self.m_nextMeshUpdate = time.real_time()
end

function Component:ResetPose()
	local ent = self:GetEntity()
	local mdl = ent:GetModel()
	local skel = mdl:GetSkeleton()
	local animC = ent:GetComponent(ents.COMPONENT_ANIMATED)
	local ref = mdl:GetAnimation("reference"):GetFrame(0)
	for i = 0, skel:GetBoneCount() - 1 do
		animC:SetBonePose(i, ref:GetBonePose(i))
	end
end

function Component:MaintainAnimations()
	self:ResetPose()
end

function Component:OnTick()
	if self.m_mcManager == nil then
		return
	end
	self.m_mcManager:LockResultData()
	local coefficientLists = self.m_mcManager:GetBlendShapeCoefficientLists()
	self.m_mcManager:UnlockResultData()
	local err = self.m_mcManager:GetLastError()
	if err ~= nil then
		pfm.log(
			"Failed to retrieve blend shape coefficient lists: " .. err,
			pfm.LOG_CATEGORY_PFM_GAME,
			pfm.LOG_SEVERITY_WARNING
		)
	end
	local coefficientList = coefficientLists[1]

	local ent = self:GetEntity()
	local flexC = ent:GetComponent(ents.COMPONENT_FLEX)
	if flexC == nil then
		return
	end
	local mdl = ent:GetModel()
	if coefficientList ~= nil then
		for i, value in ipairs(coefficientList) do
			local name = mediapipe.get_blend_shape_name(i - 1)
			local flexCId = mdl:LookupFlexController(name)
			if flexCId ~= -1 then
				flexC:SetFlexController(flexCId, value)
			end
		end
	end
	pfm.tag_render_scene_as_dirty()
	local t = time.real_time()
	if t >= self.m_nextMeshUpdate then
		self.m_nextMeshUpdate = t + 0.1
		self:UpdatePose()
		--self:DebugDrawFaceMesh()
		--self:DebugDrawPose()
	end
end

function Component:UpdateMediaSource()
	if self:GetEntity():IsSpawned() == false then
		return
	end

	local path = self:GetMediaSource()
	path = file.remove_file_extension(path, mediaExtensions)
	local foundExtPath
	for _, ext in ipairs(mediaExtensions) do
		local extPath = path .. "." .. ext
		local absPath = file.find_absolute_path(extPath)
		if absPath ~= nil then
			foundExtPath = absPath
			break
		end
	end
	if foundExtPath == nil then
		pfm.log("Failed to locate media source '" .. path .. "'!", pfm.LOG_CATEGORY_PFM_GAME, pfm.LOG_SEVERITY_WARNING)
		return
	end
	local fullPath = util.get_program_path() .. foundExtPath

	local creationInfo = mediapipe.MotionCaptureManager.CreationInfo()
	creationInfo.enabledOutputs = bit.bor(mediapipe.OUTPUT_DEFAULT, mediapipe.OUTPUT_FACE_GEOMETRY)
	creationInfo.smoothingFilterSettings.beta = 10.0 --
	creationInfo.smoothingFilterSettings.minCutoff = 0.05
	creationInfo.smoothingFilterSettings.derivateCutoff = 1 --
	creationInfo.smoothingFilterSettings.disableValueScaling = false
	creationInfo.smoothingFilterSettings.frequency = 30.0
	creationInfo.smoothingFilterSettings.minAllowedObjectScale = 1e-06

	local man, err = mediapipe.MotionCaptureManager.create_from_video(fullPath, creationInfo)

	--[[local man, err = mediapipe.MotionCaptureManager.create_from_camera(
		2,
		creationInfo
	)]]
	if man == false then
		pfm.log("Failed to create motion capture manager: " .. err, pfm.LOG_CATEGORY_PFM_GAME, pfm.LOG_SEVERITY_WARNING)
		return
	end
	local res, err = man:Start()
	if res == false then
		pfm.log("Failed to start motion capture: " .. err, pfm.LOG_CATEGORY_PFM_GAME, pfm.LOG_SEVERITY_WARNING)
		return
	end
	self.m_mcManager = man
end

function Component:OnEntitySpawn()
	self:UpdateMediaSource()
end

local function get_pose_transform()
	return math.ScaledTransform(Vector(0, 0, 0), EulerAngles(180, 180, 0), Vector(50, 50, 50))
end

GetQuaternionBetweenNormalizedVectors = function(v1, v2)
	local dot
	dot = v1:DotProduct(v2)
	local axis
	axis = v1:Cross(v2)
	--For non-normal vectors, the multiplying the axes length squared would be necessary:
	--float w = dot + (float)Math.Sqrt(v1.LengthSquared() * v2.LengthSquared());
	local q = Quaternion()
	if dot < -0.9999 then --parallel, opposing direction
		q = Quaternion(0.0, -v1.z, v1.y, v1.x)
	else
		q = Quaternion(dot + 1, axis.x, axis.y, axis.z)
	end
	q:Normalize()
	return q
end

get_axis_rotation_to_target = function(rot, axis, targetDir)
	--local rotDiff = GetQuaternionBetweenNormalizedVectors(rot0:GetForward(), targetDir)
	--return rot:GetForward():GetRotation(targetDir)

	local dir
	if axis == math.AXIS_X then
		dir = rot:GetForward()
	elseif axis == math.AXIS_SIGNED_X then
		dir = -rot:GetForward()
	elseif axis == math.AXIS_Y then
		dir = rot:GetUp()
	elseif axis == math.AXIS_SIGNED_Y then
		dir = -rot:GetUp()
	elseif axis == math.AXIS_Z then
		dir = rot:GetRight()
	elseif axis == math.AXIS_SIGNED_Z then
		dir = -rot:GetRight()
	end
	return GetQuaternionBetweenNormalizedVectors(dir, targetDir)
end

function Component:DetermineHeadRotation() end
function Component:UpdatePose()
	-- Use hips as reference
	local ent = self:GetEntity()
	local mdl = ent:GetModel()
	local skel = mdl:GetSkeleton()
	local hipsBoneId = skel:LookupBone("J_Bip_C_Hips")
	local handIk = "J_Bip_R_Hand"

	local landmarks = self.m_mcManager:GetPoseWorldLandmarks(0)
	if landmarks == nil then
		return
	end

	local points = {}
	local pose = get_pose_transform()
	for _, landmark in ipairs(landmarks) do
		local pos = pose * landmark.pos
		table.insert(points, pos)
	end

	local animC = ent:GetComponent(ents.COMPONENT_ANIMATED)

	local mpPosHipLeft = points[mediapipe.POSE_LANDMARK_RIGHT_HIP + 1]
	local mpPosHipRight = points[mediapipe.POSE_LANDMARK_LEFT_HIP + 1]
	local mpCenterHipPos = (mpPosHipLeft + mpPosHipRight) * 0.5

	-- DotProduct = Yaw rotation?
	local dir = (mpPosHipRight - mpPosHipLeft):GetNormal()
	local dot = dir:DotProduct(vector.RIGHT)
	local ang = math.deg(math.acos(dot))
	local poseHips = animC:GetLocalBonePose(hipsBoneId)
	local ref = mdl:GetReferencePose()
	local rot = ref:GetBonePose(hipsBoneId):GetRotation() --poseHips:GetRotation()
	local dbgInfo = debug.DrawInfo()
	dbgInfo:SetDuration(0.15)
	dbgInfo:SetColor(Color.Lime)
	local rotTest = rot:GetRight():GetRotation(dir)
	rot = rot * rotTest
	debug.draw_line(poseHips:GetOrigin(), poseHips:GetOrigin() + rot:GetRight() * 50, dbgInfo)
	-- Rotate hip bone so that it aligns with the left/right hip of the pose
	animC:SetBonePose(hipsBoneId, math.Transform(ref:GetBonePose(hipsBoneId):GetOrigin(), rot))

	-- Make everything relative to hip
	for i, p in ipairs(points) do
		points[i] = points[i] - mpCenterHipPos
		points[i]:Rotate(rotTest:GetInverse())
	end

	local mpPosLeftHand = points[mediapipe.POSE_LANDMARK_RIGHT_WRIST + 1]
	local poseHips = animC:GetLocalBonePose(hipsBoneId)
	local posHandTarget = poseHips * mpPosLeftHand
	local ikC = ent:GetComponent(ents.COMPONENT_IK_SOLVER)

	local pointsInActorSpace = {}
	for i = 1, #points do
		local pos = poseHips * points[i]
		table.insert(pointsInActorSpace, pos)
	end

	local dbgEntPoints = {}
	for i = 1, #points do
		table.insert(dbgEntPoints, ent:GetPose() * pointsInActorSpace[i])
	end
	self:DrawPose(dbgEntPoints)

	-- Calculate forearm position
	local function get_bone_length(bone)
		local boneId = skel:LookupBone(bone)
		local parent = skel:GetBone(boneId):GetParent():GetID()
		local pos = ref:GetBonePose(boneId)
		local posParent = ref:GetBonePose(parent)
		return pos:GetOrigin():Distance(posParent:GetOrigin())
	end
	local function calc_rel_pos(parentName, childName, parentLandmark, childLandmark)
		local dirUpArmToLowArm = points[childLandmark + 1] - points[parentLandmark + 1]
		dirUpArmToLowArm:Normalize()
		return animC:GetLocalBonePose(skel:LookupBone(parentName)):GetOrigin()
			+ dirUpArmToLowArm * get_bone_length(childName)
	end
	local posLowerArmTarget = calc_rel_pos(
		"J_Bip_R_UpperArm",
		"J_Bip_R_LowerArm",
		mediapipe.POSE_LANDMARK_RIGHT_SHOULDER,
		mediapipe.POSE_LANDMARK_RIGHT_ELBOW
	)
	local posHandTarget = calc_rel_pos(
		"J_Bip_R_LowerArm",
		"J_Bip_R_Hand",
		mediapipe.POSE_LANDMARK_RIGHT_ELBOW,
		mediapipe.POSE_LANDMARK_RIGHT_WRIST
	)
	local posLowerArmTargetL = calc_rel_pos(
		"J_Bip_L_UpperArm",
		"J_Bip_L_LowerArm",
		mediapipe.POSE_LANDMARK_LEFT_SHOULDER,
		mediapipe.POSE_LANDMARK_LEFT_ELBOW
	)
	local posHandTargetL = calc_rel_pos(
		"J_Bip_L_LowerArm",
		"J_Bip_L_Hand",
		mediapipe.POSE_LANDMARK_LEFT_ELBOW,
		mediapipe.POSE_LANDMARK_LEFT_WRIST
	)

	local idx = ikC:GetMemberIndex("control/" .. handIk .. "/position")
	ikC:SetTransformMemberPos(idx, math.COORDINATE_SPACE_OBJECT, posHandTarget)
	local idx = ikC:GetMemberIndex("control/" .. "J_Bip_L_Hand" .. "/position")
	ikC:SetTransformMemberPos(idx, math.COORDINATE_SPACE_OBJECT, posHandTargetL)

	local mpPosLeftElbow = points[mediapipe.POSE_LANDMARK_RIGHT_ELBOW + 1]
	local dirToHand = (mpPosLeftHand - mpPosLeftElbow)

	local mpPosLeftElbow = points[mediapipe.POSE_LANDMARK_RIGHT_ELBOW + 1]
	local lowerArmIk = "J_Bip_R_LowerArm"
	--local posLowerArmTarget = poseHips * mpPosLeftElbow
	local idxLowerArmIk = ikC:GetMemberIndex("control/" .. lowerArmIk .. "/position")
	ikC:SetTransformMemberPos(idxLowerArmIk, math.COORDINATE_SPACE_OBJECT, posLowerArmTarget)
	local idxLowerArmIk = ikC:GetMemberIndex("control/" .. "J_Bip_L_LowerArm" .. "/position")
	ikC:SetTransformMemberPos(idxLowerArmIk, math.COORDINATE_SPACE_OBJECT, posLowerArmTargetL)
	-- TODO: Calculate rotation and apply to hand

	local dbgInfo = debug.DrawInfo()
	dbgInfo:SetDuration(0.15)
	dbgInfo:SetColor(Color.Blue)
	debug.draw_line(ent:GetPos() + posLowerArmTarget, ent:GetPos() + posLowerArmTarget + Vector(0, 10, 0), dbgInfo)
	debug.draw_line(ent:GetPos() + posLowerArmTargetL, ent:GetPos() + posLowerArmTargetL + Vector(0, 10, 0), dbgInfo)

	dbgInfo:SetColor(Color.Red)
	debug.draw_line(ent:GetPos() + posHandTarget, ent:GetPos() + posHandTarget + Vector(0, 10, 0), dbgInfo)
	debug.draw_line(ent:GetPos() + posHandTargetL, ent:GetPos() + posHandTargetL + Vector(0, 10, 0), dbgInfo)
	ent:SetColor(Color(255, 255, 255, 255))

	local mpPosLeftEar = points[mediapipe.POSE_LANDMARK_LEFT_EAR + 1]
	local mpPosRightEar = points[mediapipe.POSE_LANDMARK_RIGHT_EAR + 1]
	local mpPosLeftEye = points[mediapipe.POSE_LANDMARK_LEFT_EYE + 1]
	local mpPosRightEye = points[mediapipe.POSE_LANDMARK_RIGHT_EYE + 1]
	local earCenter = (mpPosLeftEar + mpPosRightEar) / 2.0
	local earNose = points[mediapipe.POSE_LANDMARK_NOSE + 1] - earCenter
	local eyeLine = mpPosRightEye - mpPosLeftEye
	local rotAxis = eyeLine:Cross(earNose):GetNormal()
	local cosAngle = eyeLine:GetNormal():DotProduct(earNose:GetNormal())
	local angle = math.acos(cosAngle)
	local rot = Quaternion(rotAxis, angle) * EulerAngles(0, -90, 0):ToQuaternion()
	local headPose = animC:GetLocalBonePose(skel:LookupBone("J_Bip_C_Head"))
	--headPose:SetRotation(rot)
	local mpPosNose = points[mediapipe.POSE_LANDMARK_NOSE + 1]
	dbgInfo:SetColor(Color.Aqua)
	debug.draw_line(ent:GetPos() + mpPosNose, ent:GetPos() + mpPosNose + Vector(0, 10, 0), dbgInfo)

	local verts, indices, matrixData = self.m_mcManager:GetFaceGeometry(0)
	if verts ~= nil then
		local mat = Mat4()
		mat:Set(0, 0, matrixData[1])
		mat:Set(0, 1, matrixData[2])
		mat:Set(0, 2, matrixData[3])
		mat:Set(0, 3, matrixData[4])

		mat:Set(1, 0, matrixData[5])
		mat:Set(1, 1, matrixData[6])
		mat:Set(1, 2, matrixData[7])
		mat:Set(1, 3, matrixData[8])

		mat:Set(2, 0, matrixData[9])
		mat:Set(2, 1, matrixData[10])
		mat:Set(2, 2, matrixData[11])
		mat:Set(2, 3, matrixData[12])

		mat:Set(3, 0, matrixData[13])
		mat:Set(3, 1, matrixData[14])
		mat:Set(3, 2, matrixData[15])
		mat:Set(3, 3, matrixData[16])
		local scale, rot, pos, skew, perspective = mat:Decompose()
		pos = pose * pos

		--[[pos = pos - mpCenterHipPos
		pos:Rotate(rotTest:GetInverse())
		pos = poseHips * pos
		debug.draw_line(Vector(), pos, dbgInfo)
		print("matrixData: ", pos)]]
	end
	local headRot = self:DebugDrawFaceMesh()
	if headRot ~= nil then
		animC:SetBonePose(
			skel:LookupBone("J_Bip_C_Head"),
			mdl:GetAnimation("reference"):GetFrame(0):GetBonePose(skel:LookupBone("J_Bip_C_Head"))
		)
		headRot = headRot * EulerAngles(0, -90, 0):ToQuaternion()
		local headPose = animC:GetLocalBonePose(skel:LookupBone("J_Bip_C_Head"))
		headPose:SetRotation(headPose:GetRotation() * headRot)
		animC:SetLocalBonePose(skel:LookupBone("J_Bip_C_Head"), headPose)
	end

	-- Hand
	local handDef = {
		{
			landmark = mediapipe.HAND_LANDMARK_WRIST,
			bone = "J_Bip_R_Hand",
			children = {
				{
					landmark = mediapipe.HAND_LANDMARK_THUMB_CMC,
					bone = "J_Bip_R_Thumb1",
					children = {
						{
							landmark = mediapipe.HAND_LANDMARK_THUMB_MCP,
							bone = "J_Bip_R_Thumb2",
							children = {
								{
									landmark = mediapipe.HAND_LANDMARK_THUMB_IP,
									bone = "J_Bip_R_Thumb3",
									children = {
										{
											landmark = mediapipe.HAND_LANDMARK_THUMB_TIP,
										},
									},
								},
							},
						},
					},
				},
				{
					landmark = mediapipe.HAND_LANDMARK_INDEX_FINGER_MCP,
					bone = "J_Bip_R_Index1",
					children = {
						{
							landmark = mediapipe.HAND_LANDMARK_INDEX_FINGER_PIP,
							bone = "J_Bip_R_Index2",
							children = {
								{
									landmark = mediapipe.HAND_LANDMARK_INDEX_FINGER_DIP,
									bone = "J_Bip_R_Index3",
									children = {
										{
											landmark = mediapipe.HAND_LANDMARK_INDEX_FINGER_TIP,
										},
									},
								},
							},
						},
					},
				},
				{
					landmark = mediapipe.HAND_LANDMARK_MIDDLE_FINGER_MCP,
					bone = "J_Bip_R_Middle1",
					children = {
						{
							landmark = mediapipe.HAND_LANDMARK_MIDDLE_FINGER_PIP,
							bone = "J_Bip_R_Middle2",
							children = {
								{
									landmark = mediapipe.HAND_LANDMARK_MIDDLE_FINGER_DIP,
									bone = "J_Bip_R_Middle3",
									children = {
										{
											landmark = mediapipe.HAND_LANDMARK_MIDDLE_FINGER_TIP,
										},
									},
								},
							},
						},
					},
				},
				{
					landmark = mediapipe.HAND_LANDMARK_RING_FINGER_MCP,
					bone = "J_Bip_R_Ring1",
					children = {
						{
							landmark = mediapipe.HAND_LANDMARK_RING_FINGER_PIP,
							bone = "J_Bip_R_Ring2",
							children = {
								{
									landmark = mediapipe.HAND_LANDMARK_RING_FINGER_DIP,
									bone = "J_Bip_R_Ring3",
									children = {
										{
											landmark = mediapipe.HAND_LANDMARK_RING_FINGER_TIP,
										},
									},
								},
							},
						},
					},
				},
				{
					landmark = mediapipe.HAND_LANDMARK_PINKY_MCP,
					bone = "J_Bip_R_Little1",
					children = {
						{
							landmark = mediapipe.HAND_LANDMARK_PINKY_PIP,
							bone = "J_Bip_R_Little2",
							children = {
								{
									landmark = mediapipe.HAND_LANDMARK_PINKY_DIP,
									bone = "J_Bip_R_Little3",
									children = {
										{
											landmark = mediapipe.HAND_LANDMARK_PINKY_TIP,
										},
									},
								},
							},
						},
					},
				},
			},
		},
	}

	local handLines = {
		{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_THUMB_CMC },
		{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_INDEX_FINGER_MCP },
		{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_MCP },
		{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_RING_FINGER_MCP },
		{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_PINKY_MCP },
		{ mediapipe.HAND_LANDMARK_THUMB_CMC, mediapipe.HAND_LANDMARK_THUMB_MCP },
		{ mediapipe.HAND_LANDMARK_THUMB_MCP, mediapipe.HAND_LANDMARK_THUMB_IP },
		{ mediapipe.HAND_LANDMARK_THUMB_IP, mediapipe.HAND_LANDMARK_THUMB_TIP },
		{ mediapipe.HAND_LANDMARK_INDEX_FINGER_MCP, mediapipe.HAND_LANDMARK_INDEX_FINGER_PIP },
		{ mediapipe.HAND_LANDMARK_INDEX_FINGER_PIP, mediapipe.HAND_LANDMARK_INDEX_FINGER_DIP },
		{ mediapipe.HAND_LANDMARK_INDEX_FINGER_DIP, mediapipe.HAND_LANDMARK_INDEX_FINGER_TIP },
		{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_MCP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_PIP },
		{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_PIP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_DIP },
		{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_DIP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_TIP },
		{ mediapipe.HAND_LANDMARK_RING_FINGER_MCP, mediapipe.HAND_LANDMARK_RING_FINGER_PIP },
		{ mediapipe.HAND_LANDMARK_RING_FINGER_PIP, mediapipe.HAND_LANDMARK_RING_FINGER_DIP },
		{ mediapipe.HAND_LANDMARK_RING_FINGER_DIP, mediapipe.HAND_LANDMARK_RING_FINGER_TIP },
		{ mediapipe.HAND_LANDMARK_PINKY_MCP, mediapipe.HAND_LANDMARK_PINKY_PIP },
		{ mediapipe.HAND_LANDMARK_PINKY_PIP, mediapipe.HAND_LANDMARK_PINKY_DIP },
		{ mediapipe.HAND_LANDMARK_PINKY_DIP, mediapipe.HAND_LANDMARK_PINKY_TIP },
	}

	local handLandmarks = self.m_mcManager:GetHandWorldLandmarks(0)
	if handLandmarks == nil then
		return
	end

	local handPoints = {}
	for _, landmark in ipairs(handLandmarks) do
		local pos = pose * landmark.pos
		table.insert(handPoints, pos)
	end

	local handOrigin = handPoints[mediapipe.HAND_LANDMARK_WRIST + 1]
	local handOriginRot = Quaternion() -- TODO
	local handPose = math.Transform(handOrigin, handOriginRot)
	local invHandPose = handPose:GetInverse()

	for i, p in ipairs(handPoints) do
		handPoints[i] = invHandPose * p
	end

	local poseHandChar = animC:GetLocalBonePose(skel:LookupBone("J_Bip_R_Hand"))
	for i, p in ipairs(handPoints) do
		handPoints[i] = poseHandChar * p
	end

	local poseForarm = animC:GetLocalBonePose(skel:LookupBone("J_Bip_R_LowerArm"))
	local idxR = ikC:GetMemberIndex("control/" .. handIk .. "/rotation")
	ikC:SetTransformMemberRot(idxR, math.COORDINATE_SPACE_OBJECT, poseForarm:GetRotation())

	local poseForarm = animC:GetLocalBonePose(skel:LookupBone("J_Bip_L_LowerArm"))
	local idxL = ikC:GetMemberIndex("control/" .. "J_Bip_L_Hand" .. "/rotation")
	ikC:SetTransformMemberRot(idxL, math.COORDINATE_SPACE_OBJECT, poseForarm:GetRotation())

	local function apply_x(def, parent, parentBone)
		local landmark = def.landmark
		if parent ~= nil and def.bone ~= nil and parentBone ~= nil then
			if parentBone == "J_Bip_R_Hand" then
				--[[animC:SetBonePose(
					skel:LookupBone(parentBone),
					mdl:GetAnimation("reference"):GetFrame(0):GetBonePose(skel:LookupBone(parentBone))
				)]]
			else
				local posParent = handPoints[parent + 1]
				local posChild = handPoints[landmark + 1]
				local dir = (posChild - posParent)
				dir:Normalize()

				-- Reset
				animC:SetBonePose(
					skel:LookupBone(parentBone),
					mdl:GetAnimation("reference"):GetFrame(0):GetBonePose(skel:LookupBone(parentBone))
				)

				local tmpPos = animC:GetLocalBonePose(skel:LookupBone(parentBone))
				debug.draw_line(tmpPos:GetOrigin(), tmpPos:GetOrigin() + dir * get_bone_length(def.bone), dbgInfo)

				local twistAxis = mdl:FindBoneTwistAxis(skel:LookupBone(parentBone))
				local refPose0 = animC:GetLocalBonePose(skel:LookupBone(parentBone))
				local rotDiff = get_axis_rotation_to_target(refPose0, twistAxis, dir)

				local curPose = animC:GetLocalBonePose(skel:LookupBone(parentBone))
				curPose:SetRotation(rotDiff * curPose:GetRotation())
				animC:SetLocalBonePose(skel:LookupBone(parentBone), curPose)
			end
		end
		local children = def.children or {}
		for _, child in ipairs(children) do
			apply_x(child, landmark, def.bone)
		end
	end

	local function apply_hand(lmWrist, lmPinky, lmRightIndex, boneName, ikIdx)
		local mpWrist = pointsInActorSpace[lmWrist + 1]
		local mpPinky = pointsInActorSpace[lmPinky + 1]
		local mpRightIndex = pointsInActorSpace[lmRightIndex + 1]
		local tri = {
			mpWrist,
			mpPinky,
			mpRightIndex,
		}
		local forward = ((mpPinky + mpRightIndex) / 2.0) - mpWrist
		forward:Normalize()
		local up = geometry.calc_face_normal(tri[1], tri[2], tri[3])
		local right = forward:Cross(up)
		right:Normalize()
		local rot = Quaternion(forward, -right, -up)
		local twistAxis = mdl:FindBoneTwistAxis(skel:GetBone(skel:LookupBone(boneName)):GetParent():GetID())
		local rotOffset = game.Model.get_twist_axis_rotation_offset(twistAxis)
		rot = rot * rotOffset
		ikC:SetTransformMemberRot(ikIdx, math.COORDINATE_SPACE_OBJECT, rot)

		debug.draw_mesh({ tri[1], tri[2], tri[3], tri[1], tri[3], tri[2] }, dbgInfo)
		dbgInfo:SetColor(Color.Magenta)
		--[[debug.draw_line(Vector(), tri[1], dbgInfo)
		debug.draw_line(Vector(), tri[2], dbgInfo)
		debug.draw_line(Vector(), tri[3], dbgInfo)]]
		local center = (tri[1] + tri[2] + tri[3]) / 3.0

		dbgInfo:SetColor(Color.Red)
		debug.draw_line(center, center + forward * 10, dbgInfo)
		dbgInfo:SetColor(Color.Green)
		debug.draw_line(center, center + right * 10, dbgInfo)
		dbgInfo:SetColor(Color.Blue)
		debug.draw_line(center, center + up * 10, dbgInfo)
	end
	-- Unreliable
	--[[apply_hand(
		mediapipe.POSE_LANDMARK_RIGHT_WRIST,
		mediapipe.POSE_LANDMARK_RIGHT_PINKY,
		mediapipe.POSE_LANDMARK_RIGHT_INDEX,
		"J_Bip_R_Hand",
		idxR
	)
	apply_hand(
		mediapipe.POSE_LANDMARK_LEFT_WRIST,
		mediapipe.POSE_LANDMARK_LEFT_PINKY,
		mediapipe.POSE_LANDMARK_LEFT_INDEX,
		"J_Bip_L_Hand",
		idxL
	)]]

	--pose:SetRotation(EulerAngles())
	--animC:SetBonePose(skel:LookupBone("J_Bip_R_Hand"), pose)
	-- TODO: Set hand rotation same as forearm
	for _, child in ipairs(handDef) do
		apply_x(child)
	end

	local handOffset = Vector(0, 0, -0)
	for _, pair in ipairs(handLines) do
		local pos0 = handPoints[pair[1] + 1] + handOffset
		local pos1 = handPoints[pair[2] + 1] + handOffset
		debug.draw_line(pos0, pos1, dbgInfo)
	end

	--[[
	    {"HAND_LANDMARK_THUMB_CMC", umath::to_integral(mpw::HandLandmark::ThumbCMC)},
	    {"HAND_LANDMARK_THUMB_MCP", umath::to_integral(mpw::HandLandmark::ThumbMCP)},
	    {"HAND_LANDMARK_THUMB_IP", umath::to_integral(mpw::HandLandmark::ThumbIP)},
	    {"HAND_LANDMARK_THUMB_TIP", umath::to_integral(mpw::HandLandmark::ThumbTip)},
	    {"HAND_LANDMARK_INDEX_FINGER_MCP", umath::to_integral(mpw::HandLandmark::IndexFingerMCP)},
	    {"HAND_LANDMARK_INDEX_FINGER_PIP", umath::to_integral(mpw::HandLandmark::IndexFingerPIP)},
	    {"HAND_LANDMARK_INDEX_FINGER_DIP", umath::to_integral(mpw::HandLandmark::IndexFingerDIP)},
	    {"HAND_LANDMARK_INDEX_FINGER_TIP", umath::to_integral(mpw::HandLandmark::IndexFingerTip)},
	    {"HAND_LANDMARK_MIDDLE_FINGER_MCP", umath::to_integral(mpw::HandLandmark::MiddleFingerMCP)},
	    {"HAND_LANDMARK_MIDDLE_FINGER_PIP", umath::to_integral(mpw::HandLandmark::MiddleFingerPIP)},
	    {"HAND_LANDMARK_MIDDLE_FINGER_DIP", umath::to_integral(mpw::HandLandmark::MiddleFingerDIP)},
	    {"HAND_LANDMARK_MIDDLE_FINGER_TIP", umath::to_integral(mpw::HandLandmark::MiddleFingerTip)},
	    {"HAND_LANDMARK_RING_FINGER_MCP", umath::to_integral(mpw::HandLandmark::RingFingerMCP)},
	    {"HAND_LANDMARK_RING_FINGER_PIP", umath::to_integral(mpw::HandLandmark::RingFingerPIP)},
	    {"HAND_LANDMARK_RING_FINGER_DIP", umath::to_integral(mpw::HandLandmark::RingFingerDIP)},
	    {"HAND_LANDMARK_RING_FINGER_TIP", umath::to_integral(mpw::HandLandmark::RingFingerTip)},
	    {"HAND_LANDMARK_PINKY_MCP", umath::to_integral(mpw::HandLandmark::PinkyMCP)},
	    {"HAND_LANDMARK_PINKY_PIP", umath::to_integral(mpw::HandLandmark::PinkyPIP)},
	    {"HAND_LANDMARK_PINKY_DIP", umath::to_integral(mpw::HandLandmark::PinkyDIP)},
	    {"HAND_LANDMARK_PINKY_TIP", umath::to_integral(mpw::HandLandmark::PinkyTip)},
	    {"HAND_LANDMARK_COUNT", umath::to_integral(mpw::HandLandmark::Count)},
]]

	--[[
75 = Bone[Name:J_Bip_R_Index1][Id:J_Bip_R_Index1][Children:1][Parent:J_Bip_R_Hand]
76 = Bone[Name:J_Bip_R_Index2][Id:J_Bip_R_Index2][Children:1][Parent:J_Bip_R_Index1]
77 = Bone[Name:J_Bip_R_Index3][Id:J_Bip_R_Index3][Children:0][Parent:J_Bip_R_Index2]
78 = Bone[Name:J_Bip_R_Little1][Id:J_Bip_R_Little1][Children:1][Parent:J_Bip_R_Hand]
79 = Bone[Name:J_Bip_R_Little2][Id:J_Bip_R_Little2][Children:1][Parent:J_Bip_R_Little1]
80 = Bone[Name:J_Bip_R_Little3][Id:J_Bip_R_Little3][Children:0][Parent:J_Bip_R_Little2]
81 = Bone[Name:J_Bip_R_Middle1][Id:J_Bip_R_Middle1][Children:1][Parent:J_Bip_R_Hand]
82 = Bone[Name:J_Bip_R_Middle2][Id:J_Bip_R_Middle2][Children:1][Parent:J_Bip_R_Middle1]
83 = Bone[Name:J_Bip_R_Middle3][Id:J_Bip_R_Middle3][Children:0][Parent:J_Bip_R_Middle2]
84 = Bone[Name:J_Bip_R_Ring1][Id:J_Bip_R_Ring1][Children:1][Parent:J_Bip_R_Hand]
85 = Bone[Name:J_Bip_R_Ring2][Id:J_Bip_R_Ring2][Children:1][Parent:J_Bip_R_Ring1]
86 = Bone[Name:J_Bip_R_Ring3][Id:J_Bip_R_Ring3][Children:0][Parent:J_Bip_R_Ring2]
87 = Bone[Name:J_Bip_R_Thumb1][Id:J_Bip_R_Thumb1][Children:1][Parent:J_Bip_R_Hand]
88 = Bone[Name:J_Bip_R_Thumb2][Id:J_Bip_R_Thumb2][Children:1][Parent:J_Bip_R_Thumb1]
89 = Bone[Name:J_Bip_R_Thumb3][Id:J_Bip_R_Thumb3][Children:0][Parent:J_Bip_R_Thumb2]
]]

	--

	--{ mediapipe.POSE_LANDMARK_LEFT_HIP, mediapipe.POSE_LANDMARK_RIGHT_HIP, Color.Orange },
	-- Rotate pose?
end

--[[
1 = Bone[Name:Root][Id:Root][Children:1][Parent:NULL]
2 = Bone[Name:J_Bip_C_Hips][Id:J_Bip_C_Hips][Children:3][Parent:Root]
3 = Bone[Name:J_Bip_C_Spine][Id:J_Bip_C_Spine][Children:1][Parent:J_Bip_C_Hips]
4 = Bone[Name:J_Bip_C_Chest][Id:J_Bip_C_Chest][Children:1][Parent:J_Bip_C_Spine]
5 = Bone[Name:J_Bip_C_UpperChest][Id:J_Bip_C_UpperChest][Children:5][Parent:J_Bip_C_Chest]
6 = Bone[Name:J_Sec_L_Bust1][Id:J_Sec_L_Bust1][Children:1][Parent:J_Bip_C_UpperChest]
7 = Bone[Name:J_Sec_L_Bust2][Id:J_Sec_L_Bust2][Children:0][Parent:J_Sec_L_Bust1]
8 = Bone[Name:J_Sec_R_Bust1][Id:J_Sec_R_Bust1][Children:1][Parent:J_Bip_C_UpperChest]
9 = Bone[Name:J_Sec_R_Bust2][Id:J_Sec_R_Bust2][Children:0][Parent:J_Sec_R_Bust1]
10 = Bone[Name:J_Bip_C_Neck][Id:J_Bip_C_Neck][Children:1][Parent:J_Bip_C_UpperChest]
11 = Bone[Name:J_Bip_C_Head][Id:J_Bip_C_Head][Children:12][Parent:J_Bip_C_Neck]
12 = Bone[Name:J_Adj_L_FaceEye][Id:J_Adj_L_FaceEye][Children:0][Parent:J_Bip_C_Head]
13 = Bone[Name:J_Adj_R_FaceEye][Id:J_Adj_R_FaceEye][Children:0][Parent:J_Bip_C_Head]
14 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-1][Children:1][Parent:J_Bip_C_Head]
15 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-1]
16 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-afdfb0febab443d39c27f925bfbce69d-2]
17 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-1][Children:1][Parent:J_Bip_C_Head]
18 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-1]
19 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-89d78044bc474df78d9af65645e9a012-2]
20 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-1][Children:1][Parent:J_Bip_C_Head]
21 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-1]
22 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bc61df07f474ac5a9f1d60863892299-2]
23 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-1][Children:1][Parent:J_Bip_C_Head]
24 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-1]
25 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-6db76c980d4c44b893fda6a67c50aac1-2]
26 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-1][Children:1][Parent:J_Bip_C_Head]
27 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-1]
28 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-51750dc851914bcc95b1292b31f4f29e-2]
29 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-1][Children:1][Parent:J_Bip_C_Head]
30 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-1]
31 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-45780f656cad4bb1a42e0c5079cf5d29-2]
32 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-1][Children:1][Parent:J_Bip_C_Head]
33 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-1]
34 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-962ca7e6dffd43f1a4a67366f70be5e6-2]
35 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-1][Children:1][Parent:J_Bip_C_Head]
36 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-1]
37 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-cac5e11f9023454aab89ab4a86f8b2c1-2]
38 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-1][Children:1][Parent:J_Bip_C_Head]
39 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-1]
40 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-e41844db1bca474b984b22bb0da2000a-2]
41 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-1][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-1][Children:1][Parent:J_Bip_C_Head]
42 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-2][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-2][Children:1][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-1]
43 = Bone[Name:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-3][Id:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-3][Children:0][Parent:00bc98e6f978815227007109537446317ab7edb3.transferable_HairJoint-5bf3601bc9944a4fa772549c9c67380f-2]
44 = Bone[Name:J_Bip_L_Shoulder][Id:J_Bip_L_Shoulder][Children:1][Parent:J_Bip_C_UpperChest]
45 = Bone[Name:J_Bip_L_UpperArm][Id:J_Bip_L_UpperArm][Children:3][Parent:J_Bip_L_Shoulder]
46 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmInside][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmInside][Children:1][Parent:J_Bip_L_UpperArm]
47 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmInside_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmInside_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmInside]
48 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmOutside][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmOutside][Children:1][Parent:J_Bip_L_UpperArm]
49 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmOutside_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmOutside_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperArmOutside]
50 = Bone[Name:J_Bip_L_LowerArm][Id:J_Bip_L_LowerArm][Children:1][Parent:J_Bip_L_UpperArm]
51 = Bone[Name:J_Bip_L_Hand][Id:J_Bip_L_Hand][Children:5][Parent:J_Bip_L_LowerArm]
52 = Bone[Name:J_Bip_L_Index1][Id:J_Bip_L_Index1][Children:1][Parent:J_Bip_L_Hand]
53 = Bone[Name:J_Bip_L_Index2][Id:J_Bip_L_Index2][Children:1][Parent:J_Bip_L_Index1]
54 = Bone[Name:J_Bip_L_Index3][Id:J_Bip_L_Index3][Children:0][Parent:J_Bip_L_Index2]
55 = Bone[Name:J_Bip_L_Little1][Id:J_Bip_L_Little1][Children:1][Parent:J_Bip_L_Hand]
56 = Bone[Name:J_Bip_L_Little2][Id:J_Bip_L_Little2][Children:1][Parent:J_Bip_L_Little1]
57 = Bone[Name:J_Bip_L_Little3][Id:J_Bip_L_Little3][Children:0][Parent:J_Bip_L_Little2]
58 = Bone[Name:J_Bip_L_Middle1][Id:J_Bip_L_Middle1][Children:1][Parent:J_Bip_L_Hand]
59 = Bone[Name:J_Bip_L_Middle2][Id:J_Bip_L_Middle2][Children:1][Parent:J_Bip_L_Middle1]
60 = Bone[Name:J_Bip_L_Middle3][Id:J_Bip_L_Middle3][Children:0][Parent:J_Bip_L_Middle2]
61 = Bone[Name:J_Bip_L_Ring1][Id:J_Bip_L_Ring1][Children:1][Parent:J_Bip_L_Hand]
62 = Bone[Name:J_Bip_L_Ring2][Id:J_Bip_L_Ring2][Children:1][Parent:J_Bip_L_Ring1]
63 = Bone[Name:J_Bip_L_Ring3][Id:J_Bip_L_Ring3][Children:0][Parent:J_Bip_L_Ring2]
64 = Bone[Name:J_Bip_L_Thumb1][Id:J_Bip_L_Thumb1][Children:1][Parent:J_Bip_L_Hand]
65 = Bone[Name:J_Bip_L_Thumb2][Id:J_Bip_L_Thumb2][Children:1][Parent:J_Bip_L_Thumb1]
66 = Bone[Name:J_Bip_L_Thumb3][Id:J_Bip_L_Thumb3][Children:0][Parent:J_Bip_L_Thumb2]
67 = Bone[Name:J_Bip_R_Shoulder][Id:J_Bip_R_Shoulder][Children:1][Parent:J_Bip_C_UpperChest]
68 = Bone[Name:J_Bip_R_UpperArm][Id:J_Bip_R_UpperArm][Children:3][Parent:J_Bip_R_Shoulder]
69 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmInside][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmInside][Children:1][Parent:J_Bip_R_UpperArm]
70 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmInside_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmInside_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmInside]
71 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmOutside][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmOutside][Children:1][Parent:J_Bip_R_UpperArm]
72 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmOutside_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmOutside_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperArmOutside]
73 = Bone[Name:J_Bip_R_LowerArm][Id:J_Bip_R_LowerArm][Children:1][Parent:J_Bip_R_UpperArm]
74 = Bone[Name:J_Bip_R_Hand][Id:J_Bip_R_Hand][Children:5][Parent:J_Bip_R_LowerArm]
75 = Bone[Name:J_Bip_R_Index1][Id:J_Bip_R_Index1][Children:1][Parent:J_Bip_R_Hand]
76 = Bone[Name:J_Bip_R_Index2][Id:J_Bip_R_Index2][Children:1][Parent:J_Bip_R_Index1]
77 = Bone[Name:J_Bip_R_Index3][Id:J_Bip_R_Index3][Children:0][Parent:J_Bip_R_Index2]
78 = Bone[Name:J_Bip_R_Little1][Id:J_Bip_R_Little1][Children:1][Parent:J_Bip_R_Hand]
79 = Bone[Name:J_Bip_R_Little2][Id:J_Bip_R_Little2][Children:1][Parent:J_Bip_R_Little1]
80 = Bone[Name:J_Bip_R_Little3][Id:J_Bip_R_Little3][Children:0][Parent:J_Bip_R_Little2]
81 = Bone[Name:J_Bip_R_Middle1][Id:J_Bip_R_Middle1][Children:1][Parent:J_Bip_R_Hand]
82 = Bone[Name:J_Bip_R_Middle2][Id:J_Bip_R_Middle2][Children:1][Parent:J_Bip_R_Middle1]
83 = Bone[Name:J_Bip_R_Middle3][Id:J_Bip_R_Middle3][Children:0][Parent:J_Bip_R_Middle2]
84 = Bone[Name:J_Bip_R_Ring1][Id:J_Bip_R_Ring1][Children:1][Parent:J_Bip_R_Hand]
85 = Bone[Name:J_Bip_R_Ring2][Id:J_Bip_R_Ring2][Children:1][Parent:J_Bip_R_Ring1]
86 = Bone[Name:J_Bip_R_Ring3][Id:J_Bip_R_Ring3][Children:0][Parent:J_Bip_R_Ring2]
87 = Bone[Name:J_Bip_R_Thumb1][Id:J_Bip_R_Thumb1][Children:1][Parent:J_Bip_R_Hand]
88 = Bone[Name:J_Bip_R_Thumb2][Id:J_Bip_R_Thumb2][Children:1][Parent:J_Bip_R_Thumb1]
89 = Bone[Name:J_Bip_R_Thumb3][Id:J_Bip_R_Thumb3][Children:0][Parent:J_Bip_R_Thumb2]
90 = Bone[Name:J_Bip_L_UpperLeg][Id:J_Bip_L_UpperLeg][Children:4][Parent:J_Bip_C_Hips]
91 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegBack][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegBack][Children:1][Parent:J_Bip_L_UpperLeg]
92 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegBack]
93 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegFront][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegFront][Children:1][Parent:J_Bip_L_UpperLeg]
94 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegFront_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegFront_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegFront]
95 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegSide][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegSide][Children:1][Parent:J_Bip_L_UpperLeg]
96 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegSide_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegSide_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_L_TopsUpperLegSide]
97 = Bone[Name:J_Bip_L_LowerLeg][Id:J_Bip_L_LowerLeg][Children:1][Parent:J_Bip_L_UpperLeg]
98 = Bone[Name:J_Bip_L_Foot][Id:J_Bip_L_Foot][Children:1][Parent:J_Bip_L_LowerLeg]
99 = Bone[Name:J_Bip_L_ToeBase][Id:J_Bip_L_ToeBase][Children:0][Parent:J_Bip_L_Foot]
100 = Bone[Name:J_Bip_R_UpperLeg][Id:J_Bip_R_UpperLeg][Children:4][Parent:J_Bip_C_Hips]
101 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack][Children:1][Parent:J_Bip_R_UpperLeg]
102 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack_end 1][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack_end 1][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegBack]
103 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegFront][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegFront][Children:1][Parent:J_Bip_R_UpperLeg]
104 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegFront_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegFront_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegFront]
105 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegSide][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegSide][Children:1][Parent:J_Bip_R_UpperLeg]
106 = Bone[Name:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegSide_end][Id:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegSide_end][Children:0][Parent:57404e3635e204cb8b191063dc96b26f7308f3b4.transferable_J_Sec_R_TopsUpperLegSide]
107 = Bone[Name:J_Bip_R_LowerLeg][Id:J_Bip_R_LowerLeg][Children:1][Parent:J_Bip_R_UpperLeg]
108 = Bone[Name:J_Bip_R_Foot][Id:J_Bip_R_Foot][Children:1][Parent:J_Bip_R_LowerLeg]
109 = Bone[Name:J_Bip_R_ToeBase][Id:J_Bip_R_ToeBase][Children:0][Parent:J_Bip_R_Foot]
]]

function Component:DrawPose(points)
	local debugLines = {
		{ mediapipe.POSE_LANDMARK_LEFT_EAR, mediapipe.POSE_LANDMARK_RIGHT_EAR, Color.Blue },
		{ mediapipe.POSE_LANDMARK_LEFT_EYE_INNER, mediapipe.POSE_LANDMARK_LEFT_EYE_OUTER, Color.Red },
		{ mediapipe.POSE_LANDMARK_RIGHT_EYE_INNER, mediapipe.POSE_LANDMARK_RIGHT_EYE_OUTER, Color.Red },
		{ mediapipe.POSE_LANDMARK_LEFT_EYE, mediapipe.POSE_LANDMARK_RIGHT_EYE, Color.Purple },
		{ mediapipe.POSE_LANDMARK_MOUTH_LEFT, mediapipe.POSE_LANDMARK_MOUTH_RIGHT, Color.Yellow },
		{ mediapipe.POSE_LANDMARK_NOSE, mediapipe.POSE_LANDMARK_MOUTH_LEFT, Color.Lime },
		{ mediapipe.POSE_LANDMARK_NOSE, mediapipe.POSE_LANDMARK_MOUTH_RIGHT, Color.Lime },
		{ mediapipe.POSE_LANDMARK_LEFT_SHOULDER, mediapipe.POSE_LANDMARK_RIGHT_SHOULDER, Color.Green },
		{ mediapipe.POSE_LANDMARK_LEFT_SHOULDER, mediapipe.POSE_LANDMARK_LEFT_ELBOW, Color.Green },
		{ mediapipe.POSE_LANDMARK_RIGHT_SHOULDER, mediapipe.POSE_LANDMARK_RIGHT_ELBOW, Color.Green },
		{ mediapipe.POSE_LANDMARK_LEFT_ELBOW, mediapipe.POSE_LANDMARK_LEFT_WRIST, Color.Green },

		{ mediapipe.POSE_LANDMARK_LEFT_WRIST, mediapipe.POSE_LANDMARK_LEFT_PINKY, Color.Red },
		{ mediapipe.POSE_LANDMARK_LEFT_WRIST, mediapipe.POSE_LANDMARK_LEFT_INDEX, Color.Red },
		{ mediapipe.POSE_LANDMARK_LEFT_WRIST, mediapipe.POSE_LANDMARK_LEFT_THUMB, Color.Red },

		{ mediapipe.POSE_LANDMARK_RIGHT_ELBOW, mediapipe.POSE_LANDMARK_RIGHT_WRIST, Color.Green },

		{ mediapipe.POSE_LANDMARK_RIGHT_WRIST, mediapipe.POSE_LANDMARK_RIGHT_PINKY, Color.Red },
		{ mediapipe.POSE_LANDMARK_RIGHT_WRIST, mediapipe.POSE_LANDMARK_RIGHT_INDEX, Color.Red },
		{ mediapipe.POSE_LANDMARK_RIGHT_WRIST, mediapipe.POSE_LANDMARK_RIGHT_THUMB, Color.Red },

		{ mediapipe.POSE_LANDMARK_LEFT_HIP, mediapipe.POSE_LANDMARK_RIGHT_HIP, Color.Orange },
		{ mediapipe.POSE_LANDMARK_LEFT_SHOULDER, mediapipe.POSE_LANDMARK_LEFT_HIP, Color.Orange },
		{ mediapipe.POSE_LANDMARK_RIGHT_SHOULDER, mediapipe.POSE_LANDMARK_RIGHT_HIP, Color.Orange },
		{ mediapipe.POSE_LANDMARK_LEFT_HIP, mediapipe.POSE_LANDMARK_LEFT_KNEE, Color.Orange },
		{ mediapipe.POSE_LANDMARK_RIGHT_HIP, mediapipe.POSE_LANDMARK_RIGHT_KNEE, Color.Orange },
		{ mediapipe.POSE_LANDMARK_LEFT_KNEE, mediapipe.POSE_LANDMARK_LEFT_ANKLE, Color.Orange },
		{ mediapipe.POSE_LANDMARK_RIGHT_KNEE, mediapipe.POSE_LANDMARK_RIGHT_ANKLE, Color.Orange },
		{ mediapipe.POSE_LANDMARK_LEFT_ANKLE, mediapipe.POSE_LANDMARK_LEFT_HEEL, Color.Orange },
		{ mediapipe.POSE_LANDMARK_RIGHT_ANKLE, mediapipe.POSE_LANDMARK_RIGHT_HEEL, Color.Orange },
		{ mediapipe.POSE_LANDMARK_LEFT_HEEL, mediapipe.POSE_LANDMARK_LEFT_FOOT_INDEX, Color.Orange },
		{ mediapipe.POSE_LANDMARK_RIGHT_HEEL, mediapipe.POSE_LANDMARK_RIGHT_FOOT_INDEX, Color.Orange },
	}
	local dbgInfo = debug.DrawInfo()
	dbgInfo:SetDuration(0.15)
	local offset = Vector(30, 0, 0)
	for _, lp in ipairs(debugLines) do
		dbgInfo:SetColor(lp[3] or Color.Red)
		debug.draw_line(points[lp[1] + 1], points[lp[2] + 1], dbgInfo)
		debug.draw_line(points[lp[1] + 1] + offset, points[lp[2] + 1] + offset, dbgInfo)
	end
	debug.draw_mesh({
		points[mediapipe.POSE_LANDMARK_LEFT_WRIST + 1] + offset,
		points[mediapipe.POSE_LANDMARK_LEFT_PINKY + 1] + offset,
		points[mediapipe.POSE_LANDMARK_LEFT_INDEX + 1] + offset,
		points[mediapipe.POSE_LANDMARK_LEFT_PINKY + 1] + offset,
		points[mediapipe.POSE_LANDMARK_LEFT_WRIST + 1] + offset,
		points[mediapipe.POSE_LANDMARK_LEFT_INDEX + 1] + offset,
	}, dbgInfo)
end

function Component:DebugDrawPose()
	local landmarks = self.m_mcManager:GetPoseWorldLandmarks(0)
	if landmarks == nil then
		return
	end

	local points = {}
	local pose = get_pose_transform()
	for _, landmark in ipairs(landmarks) do
		local pos = pose * landmark.pos
		table.insert(points, pos)
	end

	self:DrawPose(points)
	local handPositions = {
		points[mediapipe.POSE_LANDMARK_LEFT_WRIST + 1],
		points[mediapipe.POSE_LANDMARK_RIGHT_WRIST + 1],
	}
	self:DebugDrawHands(handPositions)
end
function Component:DebugDrawHands(handPositions)
	for i, handLandmarks in ipairs(self.m_mcManager:GetHandWorldLandmarkLists()) do
		local debugLines = {
			{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_THUMB_CMC, Color.Red },
			{ mediapipe.HAND_LANDMARK_THUMB_CMC, mediapipe.HAND_LANDMARK_THUMB_MCP, Color.Red },
			{ mediapipe.HAND_LANDMARK_THUMB_MCP, mediapipe.HAND_LANDMARK_THUMB_IP, Color.Red },
			{ mediapipe.HAND_LANDMARK_THUMB_IP, mediapipe.HAND_LANDMARK_THUMB_TIP, Color.Red },
			{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_INDEX_FINGER_MCP, Color.Blue },
			{ mediapipe.HAND_LANDMARK_INDEX_FINGER_MCP, mediapipe.HAND_LANDMARK_INDEX_FINGER_PIP, Color.Blue },
			{ mediapipe.HAND_LANDMARK_INDEX_FINGER_PIP, mediapipe.HAND_LANDMARK_INDEX_FINGER_DIP, Color.Blue },
			{ mediapipe.HAND_LANDMARK_INDEX_FINGER_DIP, mediapipe.HAND_LANDMARK_INDEX_FINGER_TIP, Color.Blue },
			{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_MCP, Color.Green },
			{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_MCP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_PIP, Color.Green },
			{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_PIP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_DIP, Color.Green },
			{ mediapipe.HAND_LANDMARK_MIDDLE_FINGER_DIP, mediapipe.HAND_LANDMARK_MIDDLE_FINGER_TIP, Color.Green },
			{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_RING_FINGER_MCP, Color.Yellow },
			{ mediapipe.HAND_LANDMARK_RING_FINGER_MCP, mediapipe.HAND_LANDMARK_RING_FINGER_PIP, Color.Yellow },
			{ mediapipe.HAND_LANDMARK_RING_FINGER_PIP, mediapipe.HAND_LANDMARK_RING_FINGER_DIP, Color.Yellow },
			{ mediapipe.HAND_LANDMARK_RING_FINGER_DIP, mediapipe.HAND_LANDMARK_RING_FINGER_TIP, Color.Yellow },
			{ mediapipe.HAND_LANDMARK_WRIST, mediapipe.HAND_LANDMARK_PINKY_MCP, Color.Purple },
			{ mediapipe.HAND_LANDMARK_PINKY_MCP, mediapipe.HAND_LANDMARK_PINKY_PIP, Color.Purple },
			{ mediapipe.HAND_LANDMARK_PINKY_PIP, mediapipe.HAND_LANDMARK_PINKY_DIP, Color.Purple },
			{ mediapipe.HAND_LANDMARK_PINKY_DIP, mediapipe.HAND_LANDMARK_PINKY_TIP, Color.Purple },
		}

		local points = {}
		local pose = get_pose_transform()
		pose:SetOrigin(handPositions[i])
		for _, landmark in ipairs(handLandmarks) do
			local pos = pose * landmark.pos
			table.insert(points, pos)
		end

		local dbgInfo = debug.DrawInfo()
		dbgInfo:SetDuration(0.15)
		for _, lp in ipairs(debugLines) do
			dbgInfo:SetColor(lp[3] or Color.Red)
			debug.draw_line(points[lp[1] + 1], points[lp[2] + 1], dbgInfo)
		end
	end
end

function Component:DebugDrawFaceMesh()
	local verts, indices, matrixData = self.m_mcManager:GetFaceGeometry(0)
	if verts == nil then
		return
	end
	local dbgVerts = {}
	local maxIdx = 0
	local pose = math.ScaledTransform(Vector(-10, 65, 0), EulerAngles(0, 150, 0), Vector(0.6, 0.6, 0.6))
	for _, idx in ipairs(indices) do
		local v = verts[idx + 1]
		v = pose * v
		table.insert(dbgVerts, v)
		maxIdx = math.max(maxIdx, idx)
	end

	util.remove(self.m_dbgFaceMesh)
	local drawInfo = debug.DrawInfo()
	drawInfo:SetColor(Color(0, 0, 0, 255))
	drawInfo:SetOutlineColor(Color.White)
	self.m_dbgFaceMesh = debug.draw_mesh(dbgVerts, drawInfo)

	local dbgInfo = debug.DrawInfo()
	dbgInfo:SetDuration(0.15)
	dbgInfo:SetColor(Color.Magenta)
	if verts ~= nil then
		local mat = Mat4()
		--[[mat:Set(0, 0, matrixData[1])
		mat:Set(0, 1, matrixData[2])
		mat:Set(0, 2, matrixData[3])
		mat:Set(0, 3, matrixData[4])

		mat:Set(1, 0, matrixData[5])
		mat:Set(1, 1, matrixData[6])
		mat:Set(1, 2, matrixData[7])
		mat:Set(1, 3, matrixData[8])

		mat:Set(2, 0, matrixData[9])
		mat:Set(2, 1, matrixData[10])
		mat:Set(2, 2, matrixData[11])
		mat:Set(2, 3, matrixData[12])

		mat:Set(3, 0, matrixData[13])
		mat:Set(3, 1, matrixData[14])
		mat:Set(3, 2, matrixData[15])
		mat:Set(3, 3, matrixData[16])]]

		mat:Set(0, 0, matrixData[1])
		mat:Set(1, 0, matrixData[2])
		mat:Set(2, 0, matrixData[3])
		mat:Set(3, 0, matrixData[4])

		mat:Set(0, 1, matrixData[5])
		mat:Set(1, 1, matrixData[6])
		mat:Set(2, 1, matrixData[7])
		mat:Set(3, 1, matrixData[8])

		mat:Set(0, 2, matrixData[9])
		mat:Set(1, 2, matrixData[10])
		mat:Set(2, 2, matrixData[11])
		mat:Set(3, 2, matrixData[12])

		mat:Set(0, 3, matrixData[13])
		mat:Set(1, 3, matrixData[14])
		mat:Set(2, 3, matrixData[15])
		mat:Set(3, 3, matrixData[16])

		local scale, rot, pos, skew, perspective = mat:Decompose()
		rot = Quaternion(-rot.w, -rot.z, -rot.y, rot.x)
		pos = pose * pos
		rot = EulerAngles(0, 90, 0):ToQuaternion() * rot

		print(rot:ToEulerAngles())
		debug.draw_line(Vector(), pos, dbgInfo)
		return rot
	end
end

function Component:OnRemove()
	if self.m_mcManager ~= nil then
		self.m_mcManager:Stop()
	end
	util.remove(self.m_dbgFaceMesh)
end
ents.COMPONENT_MP_POSE_SOLVER = ents.register_component("mp_pose_solver", Component)
