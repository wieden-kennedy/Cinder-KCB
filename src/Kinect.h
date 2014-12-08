/*
* 
* Copyright (c) 2014, Ban the Rewind, Wieden+Kennedy
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#pragma once
#if defined( _DEBUG )
#pragma comment( lib, "comsuppwd.lib" )
#else
#pragma comment( lib, "comsuppw.lib" )
#endif
#pragma comment( lib, "wbemuuid.lib" )

#include "cinder/Cinder.h"
#include "cinder/Exception.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Rect.h"
#include "cinder/Surface.h"
#include "cinder/TriMesh.h"
#include "FaceTrackLib.h"
#include <functional>
#include "KinectCommonBridgeLib.h"
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include "ole2.h"
#include <thread>
#include <vector>

//! Kinect SDK wrapper for Cinder
namespace MsKinect
{
class Device;
class DeviceOptions;
typedef NUI_SKELETON_BONE_ROTATION				BoneRotation;
typedef NUI_IMAGE_RESOLUTION					ImageResolution;
typedef NUI_SKELETON_POSITION_INDEX				JointName;
typedef NUI_SKELETON_POSITION_TRACKING_STATE	JointTrackingState;
typedef KINECT_SKELETON_SELECTION_MODE			SkeletonSelectionMode;
typedef std::shared_ptr<Device>					DeviceRef;
typedef std::shared_ptr<class FaceTracker>		FaceTrackerRef;

//////////////////////////////////////////////////////////////////////////////////////////////

/*! Animation units representing a subset of Candide3 model's 
	action units.
	http://msdn.microsoft.com/en-us/library/jj130970.aspx 
	http://www.icg.isy.liu.se/candide/ */
enum : size_t
{
	AU0_UPPER_LIP_RAISER, AU1_JAW_LOWERER, AU2_LIP_STRETCHER, 
	AU3_BROW_LOWERER, AU4_LIP_CORNER_DEPRESSOR, AU5_OUTER_BROW_RAISER	
} typedef AnimationUnit;
//! Tyoe definition for animation unit map.
typedef std::map<AnimationUnit, float>	AnimationUnitMap;

//////////////////////////////////////////////////////////////////////////////////////////////

/*! Skeleton smoothing enumeration. Smoother transform improves skeleton accuracy, 
	but increases latency. */
enum : uint_fast8_t
{
	TRANSFORM_NONE, TRANSFORM_DEFAULT, TRANSFORM_SMOOTH, TRANSFORM_VERY_SMOOTH, TRANSFORM_MAX
} typedef								SkeletonTransform;

//////////////////////////////////////////////////////////////////////////////////////////////

class Bone
{
public:
	//! Returns rotation of the bone relative to camera coordinates.
	const ci::Quatf&					getAbsoluteRotation() const;
	//! Returns rotation matrix of the bone relative to camera coordinates.
	const ci::Matrix44f&				getAbsoluteRotationMatrix() const;
	//! Returns index of end joint.
	JointName							getEndJoint() const;
	//! Returns position of the bone's start joint.
	const ci::Vec3f&					getPosition() const;
	//! Returns rotation of the bone relative to the parent bone.
	const ci::Quatf&					getRotation() const;
	//! Returns rotation matrix of the bone relative to the parent bone.
	const ci::Matrix44f&				getRotationMatrix() const;
	//! Returns index of start joint.
	JointName							getStartJoint() const;
	//! Returns joint tracking state.
	JointTrackingState					getTrackingState() const;
private:
	Bone( const Vector4& position, const _NUI_SKELETON_BONE_ORIENTATION& bone, JointTrackingState trackingState );
	ci::Matrix44f						mAbsRotMat;
	ci::Quatf							mAbsRotQuat;
	JointName							mJointEnd;
	JointName							mJointStart;
	ci::Vec3f							mPosition;
	ci::Matrix44f						mRotMat;
	ci::Quatf							mRotQuat;
	JointTrackingState					mTrackingState;

	friend class						Device;
};
typedef std::map<JointName, Bone>		Skeleton;

//////////////////////////////////////////////////////////////////////////////////////////////

class DeviceOptions
{
public:
	//! Default settings
	DeviceOptions();

	//! Returns resolution of color image.
	ImageResolution						getColorResolution() const; 
	//! Returns size of color image.
	const ci::Vec2i&					getColorSize() const; 
	//! Returns resolution of depth image.
	ImageResolution						getDepthResolution() const; 
	//! Returns size of depth image.
	const ci::Vec2i&					getDepthSize() const;
	//! Returns KCB handle for this device.
	KCBHANDLE							getDeviceHandle() const;
	//! Returns unique ID for this device.
	const std::string&					getDeviceId() const;
	//! Returns 0-index for this device.
	int32_t								getDeviceIndex() const;
	//! Returns resolution of infrared image.
	ImageResolution						getInfraredResolution() const; 
	//! Returns size of infrared image.
	const ci::Vec2i&					getInfraredSize() const;
	//! Returns skeleton selection mode for this device.
	SkeletonSelectionMode				getSkeletonSelectionMode() const;
	//! Returns skeleton transform for this device.
	SkeletonTransform					getSkeletonTransform() const;

	//! Returns true if color video stream is enabled.
	bool								isColorEnabled() const;
	//! Returns true if depth tracking is enabled.
	bool								isDepthEnabled() const;
	//! Returns true if face tracking is enabled.
	bool								isFaceTrackingEnabled() const;
	//! Returns true if infrared stream is enabled.
	bool								isInfraredEnabled() const;
	//! Returns true if near mode is enabled.
	bool								isNearModeEnabled() const;
	//! Returns true if seated mode is enabled.
	bool								isSeatedModeEnabled() const;
	//! Returns true if user tracking is enabled.
	bool								isUserTrackingEnabled() const;

	//! Enables color stream. Disables infrared stream.
	DeviceOptions&						enableColor( bool enable = true );
	//! Enables depth stream.
	DeviceOptions&						enableDepth( bool enable = true );
	//! Enables face tracking.
	DeviceOptions&						enableFaceTracking( bool enable = true );
	//! Enables infrared stream. Disables color stream.
	DeviceOptions&						enableInfrared( bool enable = true );
	//! Enables near mode. Kinect for Windows only.
	DeviceOptions&						enableNearMode( bool enable = true );
	//! Enables seated mode. Kinect for Windows only.
	DeviceOptions&						enableSeatedMode( bool enable = true );
	/*! Enables user tracking. */
	DeviceOptions&						enableUserTracking( bool enable = true );
	
	//! Sets resolution of color image.
	DeviceOptions&						setColorResolution( const ImageResolution& resolution = ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	//! Sets resolution of depth image.
	DeviceOptions&						setDepthResolution( const ImageResolution& resolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240 ); 
	//! Starts device with this unique ID.
	DeviceOptions&						setDeviceId( const std::string& id = "" ); 
	//! Starts device with this 0-index.
	DeviceOptions&						setDeviceIndex( int32_t index = 0 ); 
	//! Sets resolution of infrared image.
	DeviceOptions&						setInfraredResolution( const ImageResolution& resolution = ImageResolution::NUI_IMAGE_RESOLUTION_320x240 ); 
	//! Set skeleton selection mode to \a mode.
	DeviceOptions& 						setSkeletonSelectionMode( SkeletonSelectionMode mode );
	//! Set skeleton transform to \a tranform.
	DeviceOptions& 						setSkeletonTransform( SkeletonTransform tranform );
protected:
	bool								mEnabledColor;
	bool								mEnabledDepth;
	bool								mEnabledFaceTracking;
	bool								mEnabledInfrared;
	bool								mEnabledNearMode;
	bool								mEnabledSeatedMode;
	bool								mEnabledUserTracking;

	SkeletonSelectionMode				mSkeletonSelectionMode;
	SkeletonTransform					mSkeletonTransform;
	
	ImageResolution						mColorResolution;
	ci::Vec2i							mColorSize;
	ImageResolution						mDepthResolution;
	ci::Vec2i							mDepthSize;
	ImageResolution						mInfraredResolution;
	ci::Vec2i							mInfraredSize;

	std::string							mDeviceId;
	int32_t								mDeviceIndex;
	KCBHANDLE							mDeviceHandle;

	friend class						Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class DepthProcessOptions
{
public:
	DepthProcessOptions();

	/*! Enables binary mode where background is black and users 
		are white. Set \a inverted to true to reverse. Enabling 
		binary mode also enables background removal. */
	DepthProcessOptions&				enableBinary( bool enable = true, bool inverted = false );
	//! Normalizes non-user pixels.
	DepthProcessOptions&				enableRemoveBackground( bool enable = true );
	//! Colorizes user pixels.
	DepthProcessOptions&				enableUserColor( bool enable = true );

	//! Returns true if image is black and white.
	bool								isBinaryEnabled() const;
	//! Returns true if black and white image is inverted.
	bool								isBinaryInverted() const;
	//! Returns true if background removal is enabled.
	bool								isRemoveBackgroundEnabled() const;
	//! Returns true if user colorization is enabled.
	bool								isUserColorEnabled() const;
protected:
	bool								mBinary;
	bool								mBinaryInverted;
	bool								mRemoveBackground;
	bool								mUserColor;
};

//////////////////////////////////////////////////////////////////////////////////////////////

//! Structure containing face data.
class Face
{
public:
	Face();

	/*! Returns animation unit (AU) map. First value is an 
		AnimationUnit enumerator. The second value is a float 
		between -1.0 and 1.0. See:
		http://msdn.microsoft.com/en-us/library/jj130970.aspx */
	const AnimationUnitMap&				getAnimationUnits() const;
	//! Returns rectangle of face location in pixels inside the color image.
	const ci::Rectf&				getBounds() const;
	/*! Returns 3D TriMesh of face in world space. FaceTracker must 
		have mesh calculation enabled. */
	const ci::TriMesh&					getMesh() const;
	/*! Returns 2D TriMesh of face. Coordinates are projected into color image.
		FaceTracker must have 2D mesh calculation enabled. */
	const ci::TriMesh2d&				getMesh2d() const;
	//! Returns transform matrix of face's pose.
	const ci::Matrix44f&				getPoseMatrix() const;
	//! Returns ID provided in FaceTracker::findFaces().
	size_t								getUserId() const;
protected:
	AnimationUnitMap					mAnimationUnits;
	ci::Rectf							mBounds;
	ci::TriMesh							mMesh;
	ci::TriMesh2d						mMesh2d;
	ci::Matrix44f						mPoseMatrix;
	size_t								mUserId;

	friend class						FaceTracker;
};

//////////////////////////////////////////////////////////////////////////////////////////////

//! Microsoft FaceTracking API wrapper for use with the Kinect.
class FaceTracker
{
protected:
	typedef std::shared_ptr<std::thread> ThreadRef;
public:
	/*! Creates pointer to instance of FaceTracker. Tracks one face at a 
		time. For multiple faces, create multiple FaceTracker instances 
		and use hinting (pass head and neck points of skeleton) when 
		finding faces. */
	static FaceTrackerRef				create();
	~FaceTracker();

	//! Returns pointer to native IFTFaceTracker.
	IFTFaceTracker*						getFaceTracker() const;
	//! Returns pointer to native IFTModel.
	IFTModel*							getModel() const;
	//! Returns pointer to native IFTResult.
	IFTResult*							getResult() const;

	/*! Enables 3D mesh calculation. Face will be returned with 
		empty TriMesh if disabled. */
	void								enableCalcMesh( bool enabled = true );
	/*! Enables 2D mesh calculation. Face will be returned with 
		empty TriMesh2d if disabled. TriMesh2d coordinates are 
		projected into color image. */
	void								enableCalcMesh2d( bool enabled = true );
	//! Returns true if 3D mesh calculation is enabled.
	bool								isCalcMeshEnabled() const;
	//! Returns true if 2D mesh calculation is enabled.
	bool								isCalcMesh2dEnabled() const;

	//! Returns true if face tracker is running.
	bool								isTracking() const;

	//! Start face tracking, allocating buffers based on \a deviceOptions.
	virtual void						start( const DeviceOptions& deviceOptions = DeviceOptions() );
	//! Stop face tracking
	virtual void						stop();
	
	/*! Update \a color and \a depth images from Kinect. Pass head and 
		neck points together, in order, through \a headPoints to target a user. 
		The value passed to \a userId will be returned from Face::getUserId() in 
		the event handler's face argument. */
	virtual void						update( const ci::Surface8u& color, const ci::Channel16u& depth, 
		const ci::Vec3f headPoints[ 2 ] = 0, size_t userId = 0 );
	
	//! Set event handler to method with signature void( FaceTracker::Face ).
	template<typename T, typename Y> 
	inline void							connectEventHander( T eventHandler, Y* obj )
	{
		connectEventHander( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	//! Set event handler to method with signature void( FaceTracker::Face ).
	void								connectEventHander( const std::function<void( Face )>& eventHandler );
protected:
	FaceTracker();

	std::function<void( Face )>			mEventHandler;
	std::mutex							mMutex;
	volatile bool						mNewFace;
	volatile bool						mRunning;
	ThreadRef							mThread;
	virtual void						run();

	bool								mCalcMesh;
	bool								mCalcMesh2d;
	ci::Channel16u						mChannelDepth;
	FT_CAMERA_CONFIG					mConfigColor;
	FT_CAMERA_CONFIG					mConfigDepth;
	Face								mFace;
	IFTFaceTracker*						mFaceTracker;
	std::vector<ci::Vec3f>				mHeadPoints;
	KCBHANDLE							mKinect;
	IFTModel*							mModel;
	IFTResult*							mResult;
	FT_SENSOR_DATA						mSensorData;
	bool								mSuccess;
	ci::Surface8u						mSurfaceColor;
	size_t								mUserId;
public:

	//////////////////////////////////////////////////////////////////////////////////////////////

	class Exception : public cinder::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char							mMessage[ 2048 ];
		friend class					FaceTracker;
	};

	//! Exception representing failure to create FaceTracker.
	class ExcFaceTrackerCreate : public Exception 
	{
	public:
		ExcFaceTrackerCreate() throw();
	};

	//! Exception representing failure to create image buffer.
	class ExcFaceTrackerCreateImage : public Exception 
	{
	public:
		ExcFaceTrackerCreateImage( long hr ) throw();
	};
	
	//! Exception representing failure to create FaceTracker result.
	class ExcFaceTrackerCreateResult : public Exception 
	{
	public:
		  ExcFaceTrackerCreateResult( long hr ) throw();
	};

	//! Exception representing failure to initialize FaceTracker.
	class ExcFaceTrackerInit : public Exception 
	{
	public:
		ExcFaceTrackerInit( long hr ) throw();
	};
};

//////////////////////////////////////////////////////////////////////////////////////////////

/*! Class representing Kinect frame data. A frame only contains data 
	for enabled features (e.g., skeletons are empty if skeleton tracking 
	is disabled). */
class Frame
{
public:
	Frame();

	//! Returns color surface for this frame.
	const ci::Surface8u&				getColorSurface() const;
	//! Returns depth channel for this frame.
	const ci::Channel16u&				getDepthChannel() const;
	//! Returns unique identifier for the sensor that generated the frame.
	const std::string&					getDeviceId() const;
	//! Returns face if tracking is enabled.
	const Face&							getFace() const;
	//! Returns Vec4f representing floor clip plane when skeleton is present.
	const ci::Vec4f&					getFloorClipPlane() const;
	//! Returns unique, sequential frame ID.
	long long							getFrameId() const;
	//! Returns infrared channel for this frame.
	const ci::Channel16u&				getInfraredChannel() const;
	//! Returns normal to gravity vector when skeleton is present.
	const ci::Vec3f&					getNormalToGravity() const;
	//! Returns skeletons for this frame.
	const std::vector<Skeleton>&		getSkeletons() const;
protected:
	Frame( long long frameId, const std::string& deviceId, const ci::Surface8u& color, 
		const ci::Channel16u& depth, const ci::Channel16u& infrared, 
		const std::vector<Skeleton>& skeletons, const Face& face, 
		const ci::Vec4f& floorClipPlane, const ci::Vec3f& normalToGravity );

	ci::Surface8u						mColorSurface;
	ci::Channel16u						mDepthChannel;
	std::string							mDeviceId;
	MsKinect::Face						mFace;
	ci::Vec4f							mFloorClipPlane;
	long long							mFrameId;
	ci::Channel16u						mInfraredChannel;
	ci::Vec3f							mNormalToGravity;
	std::vector<Skeleton>				mSkeletons;

	friend class						Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

// Kinect sensor interface
class Device
{
public:
	//! Maximum number of devices supported by the Kinect SDK.
	static const int32_t				MAXIMUM_DEVICE_COUNT	= 8;
	//! Maximum device tilt angle in positive or negative degrees.
	static const int32_t				MAXIMUM_TILT_ANGLE		= 28;

	~Device();
	
	//! Creates pointer to instance of Kinect
	static DeviceRef					create();		
	
	//! Start capturing using settings specified in \a deviceOptions .
	void								start( const DeviceOptions& deviceOptions = DeviceOptions() );
	//! Stop capture.
	void								stop();
	
	//! Enables verbose error reporting in debug console. Default is true.
	void								enableVerbose( bool enable = true );

	//! Returns coordinate mapper for this device.
	INuiCoordinateMapper*				getCoordinateMapper() const;
	//! Returns options object for this device.
	const DeviceOptions&				getDeviceOptions() const;
	//! Returns the face tracker for this device.
	FaceTrackerRef&						getFaceTracker();
	//! Returns the face tracker for this device.
	const FaceTrackerRef&				getFaceTracker() const;
	//! Returns accelerometer reading.
	ci::Quatf							getOrientation() const;
	//! Returns current device angle in degrees between -28 and 28.
	int32_t								getTilt();
	//! Returns number of tracked users. Depth resolution must be no more than 320x240 with user tracking enabled.
	int32_t								getUserCount();

	//! Returns true is actively capturing.
	bool								isCapturing() const;

	//! Sets device angle to \a degrees. Default is 0.
	void								setTilt( int32_t degrees = 0 );

	//! Returns pixel location of color position in depth image.
	ci::Vec2i							mapColorCoordToDepth( const ci::Vec2i& v );
	//! Returns pixel location of color position in depth image.
	ci::Vec2i							mapDepthCoordToColor( const ci::Vec2i& v );
	//! Returns pixel location of skeleton position in color image.
	ci::Vec2i							mapSkeletonCoordToColor( const ci::Vec3f& v );
	//! Returns pixel location of skeleton position in depth image.
	ci::Vec2i							mapSkeletonCoordToDepth( const ci::Vec3f& v );

	//! Sets frame event handler. Signature is void( Frame ).
	template<typename T, typename Y> 
	inline void							connectEventHandler( T eventHandler, Y *obj )
	{
		connectEventHandler( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}
	//! Sets frame event handler. Signature is void( Frame ).
	void								connectEventHandler( const std::function<void ( Frame )>& eventHandler );
protected:
	static const int32_t				WAIT_TIME = 100;

	Device();
	
	void								init( bool reset = false );
	virtual void						update();

	std::function<void ( Frame frame )>	mEventHandler;
	
	DeviceOptions						mDeviceOptions;
	
	INuiCoordinateMapper*				mCoordinateMapper;
	INuiSensor*							mNuiSensor;

	uint8_t*							mBufferColor;
	uint8_t*							mBufferDepth;
	uint8_t*							mBufferInfrared;
	ci::Channel16u						mChannelDepth;
	ci::Channel16u						mChannelInfrared;
	KINECT_IMAGE_FRAME_FORMAT			mFormatColor;
	KINECT_IMAGE_FRAME_FORMAT			mFormatDepth;
	KINECT_IMAGE_FRAME_FORMAT			mFormatInfrared;
	long long							mFrameId;
	std::vector<Skeleton>				mSkeletons;
	ci::Surface8u						mSurfaceColor;
	
	MsKinect::Face						mFace;
	MsKinect::FaceTrackerRef			mFaceTracker;

	bool								mCapture;
	bool								mIsSkeletonDevice;
	bool								mVerbose;

	double								mTiltRequestTime;
	
	void								errorNui( long hr );
	void								statusKinect( KINECT_SENSOR_STATUS status );
	std::string							wcharToString( wchar_t* v );
	
	friend void __stdcall				deviceStatus( long hr, const wchar_t* instanceName, const wchar_t* deviceId, void* data );

	//////////////////////////////////////////////////////////////////////////////////////////////

public:
	class Exception : public ci::Exception
	{
	public:
		const char* what() const throw();
	protected:
		char			mMessage[ 2048 ];
		friend class	Device;
	};

	//! Exception representing failure to create device.
	class ExcDeviceCreate : public Exception 
	{
	public:
		ExcDeviceCreate( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to initialize device.
	class ExcDeviceInit : public Exception 
	{
	public:
		ExcDeviceInit( long hr, const std::string& id ) throw();
	};

	//! Exception representing attempt to create device with invalid index or ID.
	class ExcDeviceInvalid : public Exception 
	{
	public:
		ExcDeviceInvalid( long hr, const std::string& id ) throw();
	};

	//! Exception representing a lack of attached devices.
	class ExcDeviceUnavailable : public Exception 
	{
	public:
		ExcDeviceUnavailable() throw();
	};

	//! Exception representing attempt to get coordinate mapper.
	class ExcGetCoordinateMapper : public Exception 
	{
	public:
		ExcGetCoordinateMapper( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to open color stream.
	class ExcOpenStreamColor : public Exception
	{
	public:
		ExcOpenStreamColor( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to open depth stream.
	class ExcOpenStreamDepth : public Exception
	{
	public:
		ExcOpenStreamDepth( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to open infrared stream.
	class ExcOpenStreamInfrared : public Exception
	{
	public:
		ExcOpenStreamInfrared( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to start open streams.
	class ExcStreamStart : public Exception
	{
	public:
		ExcStreamStart( long hr, const std::string& id ) throw();
	};

	//! Exception representing failure to enable user tracking.
	class ExcUserTrackingEnable : public Exception
	{
	public:
		ExcUserTrackingEnable( long hr, const std::string& id ) throw();
	};
};

//////////////////////////////////////////////////////////////////////////////////////////////

//! Counts the number of users in \a depth.
size_t									calcNumUsersFromDepth( const ci::Channel16u& depth );
/*! Calculates confidence of \a skeleton between 0.0 and 1.0. Joints 
	are weighted by distance to torso when \a weighted is true. */
float									calcSkeletonConfidence( const Skeleton& skeleton, bool weighted = false );
//! Shifts 16-bit depth data to make it visible.
ci::Channel8u							channel16To8( const ci::Channel16u& channel );
//! Creates a surface with colorized users from \a depth.
ci::Surface16u							depthChannelToSurface( const ci::Channel16u& depth, 
															  const DepthProcessOptions& depthProcessOptions = DepthProcessOptions() );
//! Returns depth value as 0.0 - 1.0 float for pixel at \a pos.
float									getDepthAtCoord( const ci::Channel16u& depth, const ci::Vec2i& v );
//! Returns number of Kinect devices.
size_t									getDeviceCount();
//! Returns use color for user ID \a id.
ci::Colorf								getUserColor( uint32_t id );
//! Returns user ID for pixel at \a coord in \a depth. 0 is no user.
uint16_t								userIdFromDepthCoord( const ci::Channel16u& depth, const ci::Vec2i& v );

}
 