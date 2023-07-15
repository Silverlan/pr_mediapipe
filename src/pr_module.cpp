#include "pr_module.hpp"
#include <pragma/lua/luaapi.h>
#include <pragma/lua/converters/optional_converter_t.hpp>
#include <pragma/lua/converters/pair_converter_t.hpp>
#include <pragma/lua/converters/vector_converter_t.hpp>
#include <pragma/console/conout.h>
#include <luainterface.hpp>
#include <mediapipe_pragma_wrapper.h>

#pragma optimize("", off)

extern "C" {

DLLEXPORT void pragma_initialize_lua(Lua::Interface &lua)
{
	auto path = util::Path::CreatePath(util::get_program_path()) + "modules/mediapipe/";
	mpw::init(path.GetString().c_str(), "/");

	auto &libMediapipe = lua.RegisterLibrary("mediapipe");
	libMediapipe[luabind::def("get_blend_shape_name", &mpw::get_blend_shape_name)];
	libMediapipe[luabind::def("get_blend_shape_enum", &mpw::get_blend_shape_enum)];
	libMediapipe[luabind::def("get_pose_landmark_name", &mpw::get_pose_landmark_name)];
	libMediapipe[luabind::def("get_pose_landmark_enum", &mpw::get_pose_landmark_enum)];
	libMediapipe[luabind::def("get_hand_landmark_name", &mpw::get_hand_landmark_name)];
	libMediapipe[luabind::def("get_hand_landmark_enum", &mpw::get_hand_landmark_enum)];

	auto classDefSmoothFilter = luabind::class_<mpw::SmoothingFilterSettings>("SmoothingFilterSettings");
	classDefSmoothFilter.def(
	  "__tostring", +[](const mpw::SmoothingFilterSettings &smoothingFilterSettings) -> std::string {
		  std::stringstream ss;
		  ss << "MPWSmoothingFilterSettings";
		  return ss.str();
	  });
	classDefSmoothFilter.def_readwrite("beta", &mpw::SmoothingFilterSettings::beta);
	classDefSmoothFilter.def_readwrite("disableValueScaling", &mpw::SmoothingFilterSettings::disableValueScaling);
	classDefSmoothFilter.def_readwrite("frequency", &mpw::SmoothingFilterSettings::frequency);
	classDefSmoothFilter.def_readwrite("minCutoff", &mpw::SmoothingFilterSettings::minCutoff);
	classDefSmoothFilter.def_readwrite("derivateCutoff", &mpw::SmoothingFilterSettings::derivateCutoff);
	classDefSmoothFilter.def_readwrite("minAllowedObjectScale", &mpw::SmoothingFilterSettings::minAllowedObjectScale);
	libMediapipe[classDefSmoothFilter];

	auto classDefLm = luabind::class_<mpw::MotionCaptureManager::LandmarkData>("LandmarkData");
	classDefLm.def(
	  "__tostring", +[](const mpw::MotionCaptureManager::LandmarkData &landmarkData) -> std::string {
		  std::stringstream ss;
		  ss << "LandmarkData";
		  ss << "[" << landmarkData.pos[0] << "," << landmarkData.pos[1] << "," << landmarkData.pos[2] << "]";
		  ss << "[Presence:" << landmarkData.presence << "]";
		  ss << "[Vis:" << landmarkData.visibility << "]";
		  return ss.str();
	  });
	classDefLm.def_readonly("presence", &mpw::MotionCaptureManager::LandmarkData::presence);
	classDefLm.def_readonly("visibility", &mpw::MotionCaptureManager::LandmarkData::visibility);
	classDefLm.property(
	  "pos", +[](lua_State *l, mpw::MotionCaptureManager::LandmarkData &landmarkData) {
		  return Vector3 {landmarkData.pos[0], landmarkData.pos[1], landmarkData.pos[2]};
	  });
	libMediapipe[classDefLm];

	auto classDef = luabind::class_<mpw::MotionCaptureManager>("MotionCaptureManager");
	auto defCreationInfo = luabind::class_<mpw::MotionCaptureManager::CreationInfo>("CreationInfo");
	defCreationInfo.def(
	  "__tostring", +[](const mpw::MotionCaptureManager::CreationInfo &creationInfo) -> std::string {
		  std::stringstream ss;
		  ss << "MPWCreationInfo";
		  return ss.str();
	  });
	defCreationInfo.def(luabind::constructor<>());
	defCreationInfo.def_readwrite("enabledOutputs", &mpw::MotionCaptureManager::CreationInfo::enabledOutputs);
	defCreationInfo.def_readwrite("smoothingFilterSettings", &mpw::MotionCaptureManager::CreationInfo::smoothingFilterSettings);
	classDef.scope[defCreationInfo];
	classDef.def(
	  "__tostring", +[](const mpw::MotionCaptureManager &capMan) -> std::string {
		  std::stringstream ss;
		  ss << "MotionCaptureManager";
		  return ss.str();
	  });
	classDef.scope[luabind::def(
	  "create_from_image", +[](lua_State *l, const std::string &source, mpw::MotionCaptureManager::CreationInfo &creationInfo) -> Lua::var<mpw::MotionCaptureManager, std::pair<bool, std::string>> {
		  std::string err;
		  auto manager = mpw::MotionCaptureManager::CreateFromImage(source, err, creationInfo);
		  if(!manager)
			  return luabind::object {l, std::pair<bool, std::string> {false, err}};
		  return luabind::object {l, manager};
	  })];
	classDef.scope[luabind::def(
	  "create_from_video", +[](lua_State *l, const std::string &source, mpw::MotionCaptureManager::CreationInfo &creationInfo) -> Lua::var<mpw::MotionCaptureManager, std::pair<bool, std::string>> {
		  std::string err;
		  auto manager = mpw::MotionCaptureManager::CreateFromVideo(source, err, creationInfo);
		  if(!manager)
			  return luabind::object {l, std::pair<bool, std::string> {false, err}};
		  return luabind::object {l, manager};
	  })];
	classDef.scope[luabind::def(
	  "create_from_camera", +[](lua_State *l, mpw::CameraDeviceId devId, mpw::MotionCaptureManager::CreationInfo &creationInfo) -> Lua::var<mpw::MotionCaptureManager, std::pair<bool, std::string>> {
		  std::string err;
		  auto manager = mpw::MotionCaptureManager::CreateFromCamera(devId, err, creationInfo);
		  if(!manager)
			  return luabind::object {l, std::pair<bool, std::string> {false, err}};
		  return luabind::object {l, manager};
	  })];
	classDef.def(
	  "Start", +[](mpw::MotionCaptureManager &manager) -> std::pair<bool, std::optional<std::string>> {
		  std::string err;
		  auto res = manager.Start(err);
		  if(!res)
			  return {res, {}};
		  return {res, err};
	  });
	classDef.def("Stop", &mpw::MotionCaptureManager::Stop);
	classDef.def("LockResultData", &mpw::MotionCaptureManager::LockResultData);
	classDef.def("UnlockResultData", &mpw::MotionCaptureManager::UnlockResultData);
	classDef.def("GetLastError", &mpw::MotionCaptureManager::GetLastError);

	classDef.def("GetBlendShapeCollectionCount", &mpw::MotionCaptureManager::GetBlendShapeCollectionCount);
	classDef.def(
	  "GetBlendShapeCoefficient", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex, mpw::BlendShape blendShape) -> std::optional<float> {
		  float coefficient;
		  auto res = manager.GetBlendShapeCoefficient(collectionIndex, blendShape, coefficient);
		  return res ? coefficient : std::optional<float> {};
	  });
	classDef.def(
	  "GetBlendShapeCoefficients", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex) -> std::optional<std::vector<float>> {
		  std::vector<float> coefficients;
		  auto res = manager.GetBlendShapeCoefficients(collectionIndex, coefficients);
		  return res ? coefficients : std::optional<std::vector<float>> {};
	  });
	classDef.def(
	  "GetBlendShapeCoefficientLists", +[](mpw::MotionCaptureManager &manager) -> std::vector<std::vector<float>> {
		  std::vector<std::vector<float>> coefficients;
		  manager.GetBlendShapeCoefficientLists(coefficients);
		  return coefficients;
	  });

	classDef.def("GetFaceGeometryCount", &mpw::MotionCaptureManager::GetFaceGeometryCount);
	classDef.def(
	  "GetFaceGeometry", +[](mpw::MotionCaptureManager & manager, size_t index) -> std::optional < std::tuple<std::vector<Vector3>, std::vector<uint32_t>, std::array<float,16>>> {
		  mpw::MeshData meshData;
		  auto res = manager.GetFaceGeometry(index, meshData);
		  if(!res)
			  return {};
		  std::vector<Vector3> verts;
		  verts.resize(meshData.vertices.size());
		  auto n = verts.size();
		  for(auto i = decltype(n) {0u}; i < n; ++i) {
			  auto &v = meshData.vertices[i];
			  verts[i] = {v[0], v[1], v[2]};
		  }
		  return {std::tuple<std::vector<Vector3>, std::vector<uint32_t>, std::array<float, 16>> {verts, meshData.indices, meshData.poseMatrix}};
	  });

	classDef.def("GetPoseCollectionCount", &mpw::MotionCaptureManager::GetPoseCollectionCount);
	classDef.def(
	  "GetPoseWorldLandmark", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex, mpw::PoseLandmark poseLandmark) -> std::optional<mpw::MotionCaptureManager::LandmarkData> {
		  mpw::MotionCaptureManager::LandmarkData landmark;
		  auto res = manager.GetPoseWorldLandmark(collectionIndex, poseLandmark, landmark);
		  if(res)
			  return landmark;
		  return {};
	  });
	classDef.def(
	  "GetPoseWorldLandmarks", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex) -> std::optional<std::vector<mpw::MotionCaptureManager::LandmarkData>> {
		  std::vector<mpw::MotionCaptureManager::LandmarkData> coefficients;
		  auto res = manager.GetPoseWorldLandmarks(collectionIndex, coefficients);
		  return res ? coefficients : std::optional<std::vector<mpw::MotionCaptureManager::LandmarkData>> {};
	  });
	classDef.def(
	  "GetPoseWorldLandmarkLists", +[](mpw::MotionCaptureManager &manager) -> std::vector<std::vector<mpw::MotionCaptureManager::LandmarkData>> {
		  std::vector<std::vector<mpw::MotionCaptureManager::LandmarkData>> coefficients;
		  manager.GetPoseWorldLandmarkLists(coefficients);
		  return coefficients;
	  });

	classDef.def("GetHandCollectionCount", &mpw::MotionCaptureManager::GetHandCollectionCount);
	classDef.def(
	  "GetHandWorldLandmark", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex, mpw::HandLandmark handLandmark) -> std::optional<mpw::MotionCaptureManager::LandmarkData> {
		  mpw::MotionCaptureManager::LandmarkData landmark;
		  auto res = manager.GetHandWorldLandmark(collectionIndex, handLandmark, landmark);
		  if(res)
			  return landmark;
		  return {};
	  });
	classDef.def(
	  "GetHandWorldLandmarks", +[](mpw::MotionCaptureManager &manager, size_t collectionIndex) -> std::optional<std::vector<mpw::MotionCaptureManager::LandmarkData>> {
		  std::vector<mpw::MotionCaptureManager::LandmarkData> coefficients;
		  auto res = manager.GetHandWorldLandmarks(collectionIndex, coefficients);
		  return res ? coefficients : std::optional<std::vector<mpw::MotionCaptureManager::LandmarkData>> {};
	  });
	classDef.def(
	  "GetHandWorldLandmarkLists", +[](mpw::MotionCaptureManager &manager) -> std::vector<std::vector<mpw::MotionCaptureManager::LandmarkData>> {
		  std::vector<std::vector<mpw::MotionCaptureManager::LandmarkData>> coefficients;
		  manager.GetHandWorldLandmarkLists(coefficients);
		  return coefficients;
	  });

	classDef.def("IsFrameComplete", &mpw::MotionCaptureManager::IsFrameComplete);
	classDef.def("WaitForFrame", &mpw::MotionCaptureManager::WaitForFrame);
	libMediapipe[classDef];

	Lua::RegisterLibraryEnums(lua.GetState(), "mediapipe",
	  {
	    {"OUTPUT_NONE", umath::to_integral(mpw::MotionCaptureManager::Output::None)},
	    {"OUTPUT_BLEND_SHAPE_COEFFICIENTS", umath::to_integral(mpw::MotionCaptureManager::Output::BlendShapeCoefficients)},
	    {"OUTPUT_FACE_GEOMETRY", umath::to_integral(mpw::MotionCaptureManager::Output::FaceGeometry)},
	    {"OUTPUT_POSE_WORLD_LANDMARKS", umath::to_integral(mpw::MotionCaptureManager::Output::PoseWorldLandmarks)},
	    {"OUTPUT_HAND_WORLD_LANDMARKS", umath::to_integral(mpw::MotionCaptureManager::Output::HandWorldLandmarks)},
	    {"OUTPUT_DEFAULT", umath::to_integral(mpw::MotionCaptureManager::Output::Default)},
	  });

	Lua::RegisterLibraryEnums(lua.GetState(), "mediapipe",
	  {
	    {"BLEND_SHAPE_NEUTRAL", umath::to_integral(mpw::BlendShape::Neutral)},
	    {"BLEND_SHAPE_BROW_DOWN_LEFT", umath::to_integral(mpw::BlendShape::BrowDownLeft)},
	    {"BLEND_SHAPE_BROW_DOWN_RIGHT", umath::to_integral(mpw::BlendShape::BrowDownRight)},
	    {"BLEND_SHAPE_BROW_INNER_UP", umath::to_integral(mpw::BlendShape::BrowInnerUp)},
	    {"BLEND_SHAPE_BROW_OUTER_UP_LEFT", umath::to_integral(mpw::BlendShape::BrowOuterUpLeft)},
	    {"BLEND_SHAPE_BROW_OUTER_UP_RIGHT", umath::to_integral(mpw::BlendShape::BrowOuterUpRight)},
	    {"BLEND_SHAPE_CHEEK_PUFF", umath::to_integral(mpw::BlendShape::CheekPuff)},
	    {"BLEND_SHAPE_CHEEK_SQUINT_LEFT", umath::to_integral(mpw::BlendShape::CheekSquintLeft)},
	    {"BLEND_SHAPE_CHEEK_SQUINT_RIGHT", umath::to_integral(mpw::BlendShape::CheekSquintRight)},
	    {"BLEND_SHAPE_EYE_BLINK_LEFT", umath::to_integral(mpw::BlendShape::EyeBlinkLeft)},
	    {"BLEND_SHAPE_EYE_BLINK_RIGHT", umath::to_integral(mpw::BlendShape::EyeBlinkRight)},
	    {"BLEND_SHAPE_EYE_LOOK_DOWN_LEFT", umath::to_integral(mpw::BlendShape::EyeLookDownLeft)},
	    {"BLEND_SHAPE_EYE_LOOK_DOWN_RIGHT", umath::to_integral(mpw::BlendShape::EyeLookDownRight)},
	    {"BLEND_SHAPE_EYE_LOOK_IN_LEFT", umath::to_integral(mpw::BlendShape::EyeLookInLeft)},
	    {"BLEND_SHAPE_EYE_LOOK_IN_RIGHT", umath::to_integral(mpw::BlendShape::EyeLookInRight)},
	    {"BLEND_SHAPE_EYE_LOOK_OUT_LEFT", umath::to_integral(mpw::BlendShape::EyeLookOutLeft)},
	    {"BLEND_SHAPE_EYE_LOOK_OUT_RIGHT", umath::to_integral(mpw::BlendShape::EyeLookOutRight)},
	    {"BLEND_SHAPE_EYE_LOOK_UP_LEFT", umath::to_integral(mpw::BlendShape::EyeLookUpLeft)},
	    {"BLEND_SHAPE_EYE_LOOK_UP_RIGHT", umath::to_integral(mpw::BlendShape::EyeLookUpRight)},
	    {"BLEND_SHAPE_EYE_SQUINT_LEFT", umath::to_integral(mpw::BlendShape::EyeSquintLeft)},
	    {"BLEND_SHAPE_EYE_SQUINT_RIGHT", umath::to_integral(mpw::BlendShape::EyeSquintRight)},
	    {"BLEND_SHAPE_EYE_WIDE_LEFT", umath::to_integral(mpw::BlendShape::EyeWideLeft)},
	    {"BLEND_SHAPE_EYE_WIDE_RIGHT", umath::to_integral(mpw::BlendShape::EyeWideRight)},
	    {"BLEND_SHAPE_JAW_FORWARD", umath::to_integral(mpw::BlendShape::JawForward)},
	    {"BLEND_SHAPE_JAW_LEFT", umath::to_integral(mpw::BlendShape::JawLeft)},
	    {"BLEND_SHAPE_JAW_OPEN", umath::to_integral(mpw::BlendShape::JawOpen)},
	    {"BLEND_SHAPE_JAW_RIGHT", umath::to_integral(mpw::BlendShape::JawRight)},
	    {"BLEND_SHAPE_MOUTH_CLOSE", umath::to_integral(mpw::BlendShape::MouthClose)},
	    {"BLEND_SHAPE_MOUTH_DIMPLE_LEFT", umath::to_integral(mpw::BlendShape::MouthDimpleLeft)},
	    {"BLEND_SHAPE_MOUTH_DIMPLE_RIGHT", umath::to_integral(mpw::BlendShape::MouthDimpleRight)},
	    {"BLEND_SHAPE_MOUTH_FROWN_LEFT", umath::to_integral(mpw::BlendShape::MouthFrownLeft)},
	    {"BLEND_SHAPE_MOUTH_FROWN_RIGHT", umath::to_integral(mpw::BlendShape::MouthFrownRight)},
	    {"BLEND_SHAPE_MOUTH_FUNNEL", umath::to_integral(mpw::BlendShape::MouthFunnel)},
	    {"BLEND_SHAPE_MOUTH_LEFT", umath::to_integral(mpw::BlendShape::MouthLeft)},
	    {"BLEND_SHAPE_MOUTH_LOWER_DOWN_LEFT", umath::to_integral(mpw::BlendShape::MouthLowerDownLeft)},
	    {"BLEND_SHAPE_MOUTH_LOWER_DOWN_RIGHT", umath::to_integral(mpw::BlendShape::MouthLowerDownRight)},
	    {"BLEND_SHAPE_MOUTH_PRESS_LEFT", umath::to_integral(mpw::BlendShape::MouthPressLeft)},
	    {"BLEND_SHAPE_MOUTH_PRESS_RIGHT", umath::to_integral(mpw::BlendShape::MouthPressRight)},
	    {"BLEND_SHAPE_MOUTH_PUCKER", umath::to_integral(mpw::BlendShape::MouthPucker)},
	    {"BLEND_SHAPE_MOUTH_RIGHT", umath::to_integral(mpw::BlendShape::MouthRight)},
	    {"BLEND_SHAPE_MOUTH_ROLL_LOWER", umath::to_integral(mpw::BlendShape::MouthRollLower)},
	    {"BLEND_SHAPE_MOUTH_ROLL_UPPER", umath::to_integral(mpw::BlendShape::MouthRollUpper)},
	    {"BLEND_SHAPE_MOUTH_SHRUG_LOWER", umath::to_integral(mpw::BlendShape::MouthShrugLower)},
	    {"BLEND_SHAPE_MOUTH_SHRUG_UPPER", umath::to_integral(mpw::BlendShape::MouthShrugUpper)},
	    {"BLEND_SHAPE_MOUTH_SMILE_LEFT", umath::to_integral(mpw::BlendShape::MouthSmileLeft)},
	    {"BLEND_SHAPE_MOUTH_SMILE_RIGHT", umath::to_integral(mpw::BlendShape::MouthSmileRight)},
	    {"BLEND_SHAPE_MOUTH_STRETCH_LEFT", umath::to_integral(mpw::BlendShape::MouthStretchLeft)},
	    {"BLEND_SHAPE_MOUTH_STRETCH_RIGHT", umath::to_integral(mpw::BlendShape::MouthStretchRight)},
	    {"BLEND_SHAPE_MOUTH_UPPER_UP_LEFT", umath::to_integral(mpw::BlendShape::MouthUpperUpLeft)},
	    {"BLEND_SHAPE_MOUTH_UPPER_UP_RIGHT", umath::to_integral(mpw::BlendShape::MouthUpperUpRight)},
	    {"BLEND_SHAPE_NOSE_SNEER_LEFT", umath::to_integral(mpw::BlendShape::NoseSneerLeft)},
	    {"BLEND_SHAPE_NOSE_SNEER_RIGHT", umath::to_integral(mpw::BlendShape::NoseSneerRight)},
	    {"BLEND_SHAPE_COUNT", umath::to_integral(mpw::BlendShape::Count)},
	  });
	static_assert(umath::to_integral(mpw::BlendShape::Count) == 52);

	Lua::RegisterLibraryEnums(lua.GetState(), "mediapipe",
	  {
	    {"POSE_LANDMARK_NOSE", umath::to_integral(mpw::PoseLandmark::Nose)},
	    {"POSE_LANDMARK_LEFT_EYE_INNER", umath::to_integral(mpw::PoseLandmark::LeftEyeInner)},
	    {"POSE_LANDMARK_LEFT_EYE", umath::to_integral(mpw::PoseLandmark::LeftEye)},
	    {"POSE_LANDMARK_LEFT_EYE_OUTER", umath::to_integral(mpw::PoseLandmark::LeftEyeOuter)},
	    {"POSE_LANDMARK_RIGHT_EYE_INNER", umath::to_integral(mpw::PoseLandmark::RightEyeInner)},
	    {"POSE_LANDMARK_RIGHT_EYE", umath::to_integral(mpw::PoseLandmark::RightEye)},
	    {"POSE_LANDMARK_RIGHT_EYE_OUTER", umath::to_integral(mpw::PoseLandmark::RightEyeOuter)},
	    {"POSE_LANDMARK_LEFT_EAR", umath::to_integral(mpw::PoseLandmark::LeftEar)},
	    {"POSE_LANDMARK_RIGHT_EAR", umath::to_integral(mpw::PoseLandmark::RightEar)},
	    {"POSE_LANDMARK_MOUTH_LEFT", umath::to_integral(mpw::PoseLandmark::MouthLeft)},
	    {"POSE_LANDMARK_MOUTH_RIGHT", umath::to_integral(mpw::PoseLandmark::MouthRight)},
	    {"POSE_LANDMARK_LEFT_SHOULDER", umath::to_integral(mpw::PoseLandmark::LeftShoulder)},
	    {"POSE_LANDMARK_RIGHT_SHOULDER", umath::to_integral(mpw::PoseLandmark::RightShoulder)},
	    {"POSE_LANDMARK_LEFT_ELBOW", umath::to_integral(mpw::PoseLandmark::LeftElbow)},
	    {"POSE_LANDMARK_RIGHT_ELBOW", umath::to_integral(mpw::PoseLandmark::RightElbow)},
	    {"POSE_LANDMARK_LEFT_WRIST", umath::to_integral(mpw::PoseLandmark::LeftWrist)},
	    {"POSE_LANDMARK_RIGHT_WRIST", umath::to_integral(mpw::PoseLandmark::RightWrist)},
	    {"POSE_LANDMARK_LEFT_PINKY", umath::to_integral(mpw::PoseLandmark::LeftPinky)},
	    {"POSE_LANDMARK_RIGHT_PINKY", umath::to_integral(mpw::PoseLandmark::RightPinky)},
	    {"POSE_LANDMARK_LEFT_INDEX", umath::to_integral(mpw::PoseLandmark::LeftIndex)},
	    {"POSE_LANDMARK_RIGHT_INDEX", umath::to_integral(mpw::PoseLandmark::RightIndex)},
	    {"POSE_LANDMARK_LEFT_THUMB", umath::to_integral(mpw::PoseLandmark::LeftThumb)},
	    {"POSE_LANDMARK_RIGHT_THUMB", umath::to_integral(mpw::PoseLandmark::RightThumb)},
	    {"POSE_LANDMARK_LEFT_HIP", umath::to_integral(mpw::PoseLandmark::LeftHip)},
	    {"POSE_LANDMARK_RIGHT_HIP", umath::to_integral(mpw::PoseLandmark::RightHip)},
	    {"POSE_LANDMARK_LEFT_KNEE", umath::to_integral(mpw::PoseLandmark::LeftKnee)},
	    {"POSE_LANDMARK_RIGHT_KNEE", umath::to_integral(mpw::PoseLandmark::RightKnee)},
	    {"POSE_LANDMARK_LEFT_ANKLE", umath::to_integral(mpw::PoseLandmark::LeftAnkle)},
	    {"POSE_LANDMARK_RIGHT_ANKLE", umath::to_integral(mpw::PoseLandmark::RightAnkle)},
	    {"POSE_LANDMARK_LEFT_HEEL", umath::to_integral(mpw::PoseLandmark::LeftHeel)},
	    {"POSE_LANDMARK_RIGHT_HEEL", umath::to_integral(mpw::PoseLandmark::RightHeel)},
	    {"POSE_LANDMARK_LEFT_FOOT_INDEX", umath::to_integral(mpw::PoseLandmark::LeftFootIndex)},
	    {"POSE_LANDMARK_RIGHT_FOOT_INDEX", umath::to_integral(mpw::PoseLandmark::RightFootIndex)},
	    {"POSE_LANDMARK_COUNT", umath::to_integral(mpw::PoseLandmark::Count)},
	  });
	static_assert(umath::to_integral(mpw::PoseLandmark::Count) == 33);

	Lua::RegisterLibraryEnums(lua.GetState(), "mediapipe",
	  {
	    {"HAND_LANDMARK_WRIST", umath::to_integral(mpw::HandLandmark::Wrist)},
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
	  });
	static_assert(umath::to_integral(mpw::HandLandmark::Count) == 21);
}
};
