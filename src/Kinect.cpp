/*
* 
* Copyright (c) 2013, Ban the Rewind
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

#include "Kinect.h"

#include "cinder/app/App.h"
#include "cinder/Utilities.h"	

#include <comutil.h>

namespace MsKinect
{

using namespace ci;
using namespace ci::app;
using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////

const double kTiltRequestInterval = 1.5;

Matrix44f toMatrix44f( const Matrix4& m ) 
{
	return Matrix44f( Vec4f( m.M11, m.M12, m.M13, m.M14 ), 
		Vec4f( m.M21, m.M22, m.M23, m.M24 ), 
		Vec4f( m.M31, m.M32, m.M33, m.M34 ), 
		Vec4f( m.M41, m.M42, m.M43, m.M44 ) );
}
Quatf toQuatf( const Vector4& v ) 
{
	return Quatf( v.w, v.x, v.y, v.z );
}
Vec3f toVec3f( const Vector4& v ) 
{
	return Vec3f( v.x, v.y, v.z );
}

void CALLBACK deviceStatus( long hr, const WCHAR *instanceName, const WCHAR *deviceId, void *data )
{
	Device* device = reinterpret_cast<Device*>( data );
	if ( SUCCEEDED( hr ) ) {
		device->start( device->getDeviceOptions() );
	} else {
		device->errorNui( hr );
		reinterpret_cast<Device*>( data )->stop();
	}
}

const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformNone			= { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformDefault		= { 0.5f, 0.5f, 0.5f, 0.05f, 0.04f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformSmooth		= { 0.5f, 0.1f, 0.5f, 0.1f, 0.1f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformVerySmooth	= { 0.7f, 0.3f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformMax			= { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformParams[ 5 ]	= 
{ kTransformNone, kTransformDefault, kTransformSmooth, kTransformVerySmooth, kTransformMax };

//////////////////////////////////////////////////////////////////////////////////////////////

Bone::Bone( const Vector4& position, const _NUI_SKELETON_BONE_ORIENTATION& bone )
{
	mAbsRotQuat	= toQuatf( bone.absoluteRotation.rotationQuaternion );
	mAbsRotMat	= toMatrix44f( bone.absoluteRotation.rotationMatrix );
	mJointEnd	= bone.endJoint;
	mJointStart	= bone.startJoint;
	mPosition	= toVec3f( position );
	mRotQuat	= toQuatf( bone.hierarchicalRotation.rotationQuaternion );
	mRotMat		= toMatrix44f( bone.hierarchicalRotation.rotationMatrix );
}

const Quatf& Bone::getAbsoluteRotation() const 
{ 
	return mAbsRotQuat; 
}
const Matrix44f& Bone::getAbsoluteRotationMatrix() const 
{ 
	return mAbsRotMat; 
}
JointName Bone::getEndJoint() const
{
	return mJointEnd;
}
const Vec3f& Bone::getPosition() const 
{ 
	return mPosition; 
}
const Quatf& Bone::getRotation() const 
{ 
	return mRotQuat; 
}
const Matrix44f& Bone::getRotationMatrix() const 
{ 
	return mRotMat; 
}
JointName Bone::getStartJoint() const
{
	return mJointStart;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceOptions::DeviceOptions()
{
	mDeviceId				= "";
	mDeviceIndex			= 0;
	mEnabledColor			= true;
	mEnabledDepth			= true;
	mEnabledNearMode		= false;
	mEnabledSeatedMode		= false;
	mEnabledUserTracking	= true;
	mSkeletonTransform		= SkeletonTransform::TRANSFORM_DEFAULT;
	mSkeletonSelectionMode	= SkeletonSelectionMode::SkeletonSelectionModeDefault;
	setColorResolution( ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	setDepthResolution( ImageResolution::NUI_IMAGE_RESOLUTION_320x240 );
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableNearMode( bool enable )
{
	mEnabledNearMode = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
{
	mEnabledUserTracking = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}
	
ImageResolution DeviceOptions::getColorResolution() const
{
	return mColorResolution;
}

const Vec2i& DeviceOptions::getColorSize() const 
{
	return mColorSize;
}

ImageResolution DeviceOptions::getDepthResolution() const
{
	return mDepthResolution;
}
	
const Vec2i& DeviceOptions::getDepthSize() const
{
	return mDepthSize;
}

const string& DeviceOptions::getDeviceId() const
{
	return mDeviceId;
}
	
int32_t DeviceOptions::getDeviceIndex() const
{
	return mDeviceIndex;
}

SkeletonSelectionMode DeviceOptions::getSkeletonSelectionMode() const
{
	return mSkeletonSelectionMode;
}

SkeletonTransform DeviceOptions::getSkeletonTransform() const
{
	return mSkeletonTransform;
}

bool DeviceOptions::isDepthEnabled() const
{
	return mEnabledDepth;
}
	
bool DeviceOptions::isNearModeEnabled() const
{
	return mEnabledNearMode;
}
	
bool DeviceOptions::isSeatedModeEnabled() const
{
	return mEnabledSeatedMode;
}

bool DeviceOptions::isUserTrackingEnabled() const
{
	return mEnabledUserTracking;
}

bool DeviceOptions::isColorEnabled() const
{
	return mEnabledColor;
}

DeviceOptions& DeviceOptions::setColorResolution( const ImageResolution& resolution )
{
	mColorResolution = resolution;
	switch ( mColorResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		mColorSize = Vec2i( 1280, 960 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mColorSize = Vec2i( 640, 480 );
		break;
	default:
		mColorResolution = NUI_IMAGE_RESOLUTION_INVALID;
		mColorSize = Vec2i::zero();
		mEnabledColor = false;
		break;
	}
	return *this;
}

DeviceOptions& DeviceOptions::setDepthResolution( const ImageResolution& resolution )
{
	mDepthResolution = resolution;
	switch ( mDepthResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mDepthSize					= Vec2i( 640, 480 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mDepthSize = Vec2i( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mDepthSize = Vec2i( 80, 60 );
		break;
	default:
		mDepthResolution = NUI_IMAGE_RESOLUTION_INVALID;
		mDepthSize = Vec2i::zero();
		mEnabledDepth = false;
		break;
	}
	return *this;
}

DeviceOptions& DeviceOptions::setDeviceId( const std::string& id )
{
	mDeviceId = id;
	return *this;
}
	
DeviceOptions& DeviceOptions::setDeviceIndex( int32_t index )
{
	mDeviceIndex = index;
	return *this;
}

DeviceOptions& DeviceOptions::setSkeletonSelectionMode( SkeletonSelectionMode mode )
{
	mSkeletonSelectionMode = mode;
	return *this;
}

DeviceOptions& DeviceOptions::setSkeletonTransform( SkeletonTransform transform )
{
	mSkeletonTransform = transform;
	return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame()
	: mDeviceId( "" ), mFrameId( 0 )
{
}

Frame::Frame( long long frameId, const std::string& deviceId, 
		const ci::Surface8u& color, const ci::Channel16u& depth, const std::vector<Skeleton>& skeletons )
	: mColorSurface( color ), mDepthChannel( depth ), mDeviceId( deviceId ), mFrameId( frameId ), mSkeletons( skeletons )
{
}

const Surface8u& Frame::getColorSurface() const
{
	return mColorSurface;
}

const Channel16u& Frame::getDepthChannel() const
{
	return mDepthChannel;
}

const std::string& Frame::getDeviceId() const
{
	return mDeviceId;
}

long long Frame::getFrameId() const
{
	return mFrameId;
}

const vector<Skeleton>&	Frame::getSkeletons() const
{
	return mSkeletons;
}

//////////////////////////////////////////////////////////////////////////////////////////////

vector<Colorf> Device::sUserColors = getUserColors();

ci::Colorf Device::getUserColor( uint32_t id ) 
{ 
	return sUserColors.at( ci::math<uint32_t>::clamp( id, 0, 5 ) ); 
}

vector<Colorf> Device::getUserColors()
{
	if ( sUserColors.size() == NUI_SKELETON_COUNT ) {
		return sUserColors;
	}

	vector<Colorf> colors;
	colors.push_back( Colorf( 0.0f, 1.0f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 0.0f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 1.0f, 0.0f ) );
	colors.push_back( Colorf( 0.0f, 0.5f, 1.0f ) );
	colors.push_back( Colorf( 0.0f, 1.0f, 0.5f ) );
	colors.push_back( Colorf( 0.0f, 0.5f, 0.5f ) );
	return colors;
}

DeviceRef Device::create()
{
	return DeviceRef( new Device( ) );
}

Device::Device()
{
	NuiSetDeviceStatusCallback( &MsKinect::deviceStatus, this );
	init();
	for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
		mSkeletons.push_back( Skeleton() );
	}
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
	if ( mNuiSensor != 0 ) {
		mNuiSensor->NuiShutdown();
		if ( mNuiSensor ) {
			mNuiSensor->Release();
			mNuiSensor = 0;
		}
	}
}

void Device::connectEventHandler( const function<void ( Frame )>& eventHandler )
{
	mEventHandler = eventHandler;
}

void Device::deactivateUsers()
{
	for ( uint32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
		mActiveUsers[ i ] = false;
	}
}

void Device::enableBinaryMode( bool enable, bool invertImage )
{
	mBinary		= enable;
	mInverted	= invertImage;
}

void Device::enableUserColor( bool enable )
{
	mGreyScale = !enable;
}

void Device::enableVerbose( bool enable )
{
	mVerbose = enable;
}

void Device::errorNui( long hr ) {
	if ( !mVerbose ) {
		return;
	}
	switch ( hr ) {
	case E_POINTER:
		console() << "Bad pointer.";
		break;
	case E_INVALIDARG:
		console() << "Invalid argument.";
		break;
	case E_NUI_DEVICE_NOT_READY:
		console() << "Device not ready.";
		break;
	case E_NUI_FEATURE_NOT_INITIALIZED:
		console() << "Feature not initialized.";
		break;
	case E_NUI_NOTCONNECTED:
		console() << "Unable to connect to device.";
		break;
	case E_FAIL:
		console() << "Attempt failed.";
		break;
	case E_NUI_IMAGE_STREAM_IN_USE:
		console() << "Image stream already in use.";
		break;
	case E_NUI_FRAME_NO_DATA:
		console() << "No frame data available";
		break;
	case E_OUTOFMEMORY:
		console() << "Out of memory (maximum number of Kinect devices may have been reached).";
		break;
	case ERROR_TOO_MANY_CMDS:
		console() << "Too many commands sent. Angle change requests must be made at least 1.5s apart.";
		break;
	case ERROR_RETRY:
		console() << "Device is busy.  Retry in a moment.";
		break;
	case S_FALSE:
		console() << "Data not available.";
	case S_OK:
		break;
	default:
		console() << "Unknown error (Code " + toString( hr ) + ")";
	}
	console() << endl;
}

Vec2i Device::getColorDepthPos( const ci::Vec2i& v )
{
	long x;
	long y;
	mNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
		mDeviceOptions.getColorResolution(), 
		mDeviceOptions.getDepthResolution(), 
		0, v.x, v.y, 
		mChannelDepth.getValue( v ), &x, &y ); 
	return Vec2i( (int32_t)x, (int32_t)y );
}

float Device::getDepthAt( const ci::Vec2i& pos ) const
{
	float depthNorm		= 0.0f;
	if ( mChannelDepth ) {
		uint16_t depth	= 0x10000 - mChannelDepth.getValue( pos );
		depth			= depth << 2;
		depthNorm		= 1.0f - (float)depth / 65535.0f;
	}
	return depthNorm;
}
	
int32_t Device::getDeviceCount()
{
	int32_t deviceCount = 0;
	NuiGetSensorCount( &deviceCount );
	return deviceCount;
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

Quatf Device::getOrientation() const
{
	Vector4 v;
	if ( mNuiSensor != 0 ) {
		mNuiSensor->NuiAccelerometerGetCurrentReading( &v );
	}
	return toQuatf( v );
}

Vec2i Device::getSkeletonDepthPos( const ci::Vec3f& position )
{
	float x;
	float y;
	Vector4 pos;
	pos.x = position.x;
	pos.y = position.y;
	pos.z = position.z;
	pos.w = 1.0f;
	NuiTransformSkeletonToDepthImage( pos, &x, &y, mDeviceOptions.getDepthResolution() );
	return Vec2i( (int32_t)x, (int32_t)y );
}

Vec2i Device::getSkeletonColorPos( const ci::Vec3f& position )
{
	float x;
	float y;
	Vector4 pos;
	pos.x = position.x;
	pos.y = position.y;
	pos.z = position.z;
	pos.w = 1.0f;
	NuiTransformSkeletonToDepthImage( pos, &x, &y, mDeviceOptions.getColorResolution() );
	return Vec2i( (int32_t)x, (int32_t)y );
}

int32_t Device::getTilt()
{
	long degrees = 0L;
	if ( mCapture && mNuiSensor != 0 ) {
		long hr = mNuiSensor->NuiCameraElevationGetAngle( &degrees );
		if ( FAILED( hr ) ) {
			console() << "Unable to retrieve device angle:" << endl;
			errorNui( hr );
		}
	}
	return (int32_t)degrees;
}

int32_t Device::getUserCount()
{
	return mDeviceOptions.isDepthEnabled() ? mUserCount : 0;
}

void Device::init( bool reset )
{
	if ( !reset ) {
		mBinary				= false;
		mEventHandler		= nullptr;
		mFlipped			= false;
		mGreyScale			= false;
		mInverted			= false;
		mRemoveBackground	= false;
		mVerbose			= true;
	}

	mBufferColor			= nullptr;
	mBufferDepth			= nullptr;
	mCapture				= false;
	mFrameId				= 0;
	mKinect					= nullptr;
	mNuiSensor				= 0;
	mIsSkeletonDevice		= false;
	mTiltRequestTime		= 0.0;
	mUserCount				= 0;

	if ( mChannelDepth ) {
		mChannelDepth.reset();
	}
	if ( mSurfaceColor ) {
		mSurfaceColor.reset();
	}
	for ( vector<Skeleton>::iterator iter = mSkeletons.begin(); iter != mSkeletons.end(); ++iter ) {
		iter->clear();
	}
	
	deactivateUsers();
}

bool Device::isCapturing() const 
{
	return mCapture; 
}

bool Device::isFlipped() const 
{ 
	return mFlipped; 
}

void Device::removeBackground( bool remove )
{
	mRemoveBackground = remove;
}

void Device::setFlipped( bool flipped ) 
{
	mFlipped = flipped;
}

void Device::setTilt( int32_t degrees )
{
	double elapsedSeconds = getElapsedSeconds();
	if ( mCapture && mNuiSensor != 0 && elapsedSeconds - mTiltRequestTime > kTiltRequestInterval ) {
		long hr = mNuiSensor->NuiCameraElevationSetAngle( (long)math<int32_t>::clamp( degrees, -MAXIMUM_TILT_ANGLE, MAXIMUM_TILT_ANGLE ) );
		if ( FAILED( hr ) ) {
			console() << "Unable to change device angle: " << endl;
			errorNui( hr );
		}
		mTiltRequestTime = elapsedSeconds;
	}
}

void Device::start( const DeviceOptions& deviceOptions ) 
{
	if ( !mCapture ) {
		long hr = S_OK;
		
		mDeviceOptions	= deviceOptions;
		string deviceId	= mDeviceOptions.getDeviceId();
		int32_t index	= mDeviceOptions.getDeviceIndex();
		if ( index >= 0 ) {
			index = math<int32_t>::clamp( index, 0, math<int32_t>::max( getDeviceCount() - 1, 0 ) );
		}

		wchar_t portId[ KINECT_MAX_PORTID_LENGTH ];
		size_t count = KinectGetSensorCount();
		if ( index >= 0 ) {
			if ( KinectGetPortIDByIndex( index, _countof( portId ), portId ) ) {
				mKinect = KinectOpenSensor( portId );
				hr		= NuiCreateSensorById( portId, &mNuiSensor );
				mDeviceOptions.setDeviceId( wcharToString( portId ) );
			}
		} else if ( deviceId.length() > 0 ) {
			_bstr_t id = deviceId.c_str();
			hr = NuiCreateSensorById( id, &mNuiSensor );
			for ( size_t i = 0; i < count; ++i ) {
				if ( deviceId == wcharToString( portId ) && KinectGetPortIDByIndex( i, _countof( portId ), portId ) ) {
					mKinect = KinectOpenSensor( portId );
					hr		= NuiCreateSensorById( portId, &mNuiSensor );
					mDeviceOptions.setDeviceIndex( i );
					break;
				} 
			}

		}

		if ( mKinect == nullptr ) {
			if ( index >= 0 ) {
				console() << "Unable to create device instance " + toString( index ) + ": " << endl;
			} else if ( deviceId.length() > 0 ) {
				console() << "Unable to create device instance " + deviceId + ":" << endl;
			} else {
				console() << "Invalid device name or index." << endl;
			}
			errorNui( hr );
			mDeviceOptions.setDeviceIndex( -1 );
			mDeviceOptions.setDeviceId( "" );
			return;
		}

		KINECT_SENSOR_STATUS status = KinectGetKinectSensorStatus( mKinect );
		statusKinect( status );

		if ( status > 1 ) {
			return;
		}
		
		if ( mDeviceOptions.isColorEnabled() && mDeviceOptions.getColorResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatColor						= format;
			hr = KinectEnableColorStream( mKinect, mDeviceOptions.getColorResolution(), &mFormatColor );
			if ( SUCCEEDED( hr ) ) {
				mBufferColor	= new uint8_t[ mFormatColor.cbBufferSize ];
			} else {
				console() << "Unable to initialize color stream: ";
				errorNui( hr );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() && mDeviceOptions.getDepthResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatDepth						= format;
			hr = KinectEnableDepthStream( mKinect, mDeviceOptions.isNearModeEnabled(), mDeviceOptions.getDepthResolution(), &mFormatDepth );
			if ( SUCCEEDED( hr ) ) {
				mBufferDepth	= new uint8_t[ mFormatDepth.cbBufferSize ];
			} else {
				console() << "Unable to initialize depth stream: ";
				errorNui( hr );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() && HasSkeletalEngine( mNuiSensor ) ) {
			hr = KinectEnableSkeletalStream( mKinect, mDeviceOptions.isSeatedModeEnabled(), SkeletonSelectionModeDefault, nullptr );
			if ( SUCCEEDED( hr ) ) {
				mIsSkeletonDevice = true;
			} else {
				console() << "Unable to initialize user tracking: ";
				errorNui( hr );
			}
		}

		mSkeletons.clear();
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
			mSkeletons.push_back( Skeleton() );
		}

		mCapture = true;
	}
}

void Device::statusKinect( KINECT_SENSOR_STATUS status )
{
	if ( !mVerbose ) {
		return;
	}
	switch ( status ) {
	case KINECT_SENSOR_STATUS::KinectSensorStatusConflict:
		console() << "Device conflict.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusError:
		console() << "A sensor error occurred.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusInitializing:
		console() << "Device initializing.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusInsufficientBandwidth:
		console() << "Insufficient bandwidth.";
		break;
	case  KINECT_SENSOR_STATUS::KinectSensorStatusNotGenuine:
		console() << "Sensor is not a genuine device.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusNotPowered:
		console() << "Sensor not powered.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusNotSupported:
		console() << "Sensor not supported.";
		break;
	case KINECT_SENSOR_STATUS::KinectSensorStatusStarted:
		console() << "Sensor started";
		break;
	default:
		break;
	}
	console() << endl;
}

void Device::stop()
{
	mCapture = false;
	if ( mBufferDepth != nullptr ) {
		delete [] mBufferDepth;
	}
	if ( mBufferColor != nullptr ) {
		delete [] mBufferColor;
	}
	KinectCloseSensor( mKinect );
	init( true );
}

void Device::update()
{
	if ( !KinectAreAllFramesReady( mKinect ) ) {
		return;
	}

	if ( mSurfaceColor ) {
		mSurfaceColor.reset();
	}
	if ( mChannelDepth ) {
		mChannelDepth.reset();
	}

	long long timestamp;
	NUI_SKELETON_FRAME skeletonFrame;
	if ( mDeviceOptions.isColorEnabled() && 
		KinectIsColorFrameReady( mKinect ) && 
		SUCCEEDED( KinectGetColorFrame( mKinect, mFormatColor.cbBufferSize, mBufferColor, &timestamp ) ) ) {
		mSurfaceColor = Surface8u( mBufferColor, 
			(int32_t)mFormatColor.dwWidth, (int32_t)mFormatColor.dwHeight, 
			(int32_t)mFormatColor.dwWidth *(int32_t) mFormatColor.cbBytesPerPixel, 
			SurfaceChannelOrder::BGRX );
	}
	if ( mDeviceOptions.isDepthEnabled() && 
		KinectIsDepthFrameReady( mKinect ) && 
		SUCCEEDED( KinectGetDepthFrame( mKinect, mFormatDepth.cbBufferSize, mBufferDepth, &timestamp ) ) ) {
		mChannelDepth = Channel16u( (int32_t)mFormatDepth.dwWidth, (int32_t)mFormatDepth.dwHeight, 
			(int32_t)mFormatDepth.dwWidth * (int32_t)mFormatColor.cbBytesPerPixel, 1, 
			reinterpret_cast<uint16_t*>( mBufferDepth ) );
    }
	if ( mDeviceOptions.isUserTrackingEnabled() && 
		KinectIsSkeletonFrameReady( mKinect ) && 
		SUCCEEDED( KinectGetSkeletonFrame( mKinect, &skeletonFrame ) ) ) {

		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {

			mSkeletons.at( i ).clear();

			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[ i ].eTrackingState;
			if ( trackingState == NUI_SKELETON_TRACKED || trackingState == NUI_SKELETON_POSITION_ONLY ) {

				if ( mFlipped ) {
					( skeletonFrame.SkeletonData + i )->Position.x *= -1.0f;
					for ( int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; ++j ) {
						( skeletonFrame.SkeletonData + i )->SkeletonPositions[ j ].x *= -1.0f;
					}
				}

				_NUI_SKELETON_BONE_ORIENTATION bones[ NUI_SKELETON_POSITION_COUNT ];
				long hr = NuiSkeletonCalculateBoneOrientations( skeletonFrame.SkeletonData + i, bones );
				if ( FAILED( hr ) ) {
					errorNui( hr );
				}

				for ( int32_t j = 0; j < (int32_t)NUI_SKELETON_POSITION_COUNT; ++j ) {
					Bone bone( *( ( skeletonFrame.SkeletonData + i )->SkeletonPositions + j ), *( bones + j ) );
					( mSkeletons.begin() + i )->insert( std::pair<JointName, Bone>( (JointName)j, bone ) );
				}

			}

		}
	}

	Frame frame( mFrameId, mDeviceOptions.getDeviceId(), mSurfaceColor, mChannelDepth, mSkeletons );
	mEventHandler( frame );
	++mFrameId;

	return;
}

string Device::wcharToString( wchar_t* v )
{
	string str = "";
	wchar_t* id = ::SysAllocString( v );
	_bstr_t idStr( id );
	if ( idStr.length() > 0 ) {
		str = string( idStr );
	}
	::SysFreeString( id );
	return str;
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* Device::Exception::what() const throw() 
{ 
	return mMessage; 
}

Device::ExcDeviceCreate::ExcDeviceCreate( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to create device. ID or index: %s. Error: %i", id, hr );
}

Device::ExcDeviceInit::ExcDeviceInit( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to initialize device. ID or index: %s. Error: %i", id, hr );
}

Device::ExcDeviceInvalid::ExcDeviceInvalid( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Invalid device ID or index: %s. Error: %i", id, hr );
}

Device::ExcOpenStreamColor::ExcOpenStreamColor( long hr )
{
	sprintf( mMessage, "Unable to open color stream. Error: %i", hr );
}

Device::ExcOpenStreamDepth::ExcOpenStreamDepth( long hr )
{
	sprintf( mMessage, "Unable to open depth stream. Error: %i", hr );
}

Device::ExcSkeletonTrackingEnable::ExcSkeletonTrackingEnable( long hr )
{
	sprintf( mMessage, "Unable to enable skeleton tracking. Error: %i", hr );
}
}
 