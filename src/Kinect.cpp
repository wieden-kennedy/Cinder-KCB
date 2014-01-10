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

//////////////////////////////////////////////////////////////////////////////////////////////

DepthProcessOptions::DepthProcessOptions()
	: mBinary( false ), mBinaryInverted( false ), mRemoveBackground( false ), mUserColor( false )
{
}

DepthProcessOptions& DepthProcessOptions::enableBinary( bool enable, bool inverted )
{
	mBinary				= enable;
	mBinaryInverted		= inverted;
	mRemoveBackground	= true;
	return *this;
}

DepthProcessOptions& DepthProcessOptions::enableRemoveBackground( bool enable )
{
	mRemoveBackground = enable;
	return *this;
}

DepthProcessOptions& DepthProcessOptions::enableUserColor( bool enable )
{
	mUserColor = enable;
	return *this;
}

bool DepthProcessOptions::isBinaryEnabled() const
{
	return mBinary;
}

bool DepthProcessOptions::isBinaryInverted() const
{
	return mBinaryInverted;
}

bool DepthProcessOptions::isRemoveBackgroundEnabled() const
{
	return mRemoveBackground;
}

bool DepthProcessOptions::isUserColorEnabled() const
{
	return mUserColor;
}

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

size_t calcNumUsersFromDepth( const Channel16u& depth )
{
	if ( !depth ) {
		return 0;
	}
	map<uint16_t, bool> users;
	Surface16u surface( depth );
	Surface16u::ConstIter iter = surface.getIter();
	while ( iter.line() ) {
		while ( iter.pixel() ) {
			uint16_t id = NuiDepthPixelToPlayerIndex( iter.r() );
			if ( id > 0 && id < 7 ) {
				users[ id ] = true;
			}
		}
	}
	return users.size();
}

float calcSkeletonConfidence( const Skeleton& skeleton, const DeviceRef& device )
{
	float c = 0.0f;
	for ( Skeleton::const_iterator iter = skeleton.begin(); iter != skeleton.end(); ++iter ) {
		Vec2i v = mapSkeletonCoordToDepth( iter->second.getPosition(), device );
		if ( v.x >= 0 && v.x < device->getDeviceOptions().getDepthSize().x &&
			 v.y >= 0 && v.y < device->getDeviceOptions().getDepthSize().y ) {
			c += 1.0f;
		}
	}
	c /= (float)JointName::NUI_SKELETON_POSITION_COUNT;
	return c;
}

Surface16u depthChannelToSurface( const Channel16u& depth, const DepthProcessOptions& depthProcessOptions )
{
	Surface16u surface( depth );
	Surface16u::Iter iter = surface.getIter();
	while ( iter.line() ) {
		while ( iter.pixel() ) {
			uint16_t v	= 0x10000 * ( ( iter.r() & 0xFFF8 ) >> 3 ) / 0x0FFF;
			uint16_t id = NuiDepthPixelToPlayerIndex( iter.r() );
			if ( depthProcessOptions.isBinaryEnabled() ) {
				if ( id < 1 || id > 6 ) {
					iter.r() = iter.g() = iter.b() = depthProcessOptions.isBinaryInverted() ? 0xFFFF : 0;
				} else {
					iter.r() = iter.g() = iter.b() = depthProcessOptions.isBinaryInverted() ? 0 : 0xFFFF;
				}
			} else if ( depthProcessOptions.isUserColorEnabled() ) {
				if ( id < 1 || id > 6 ) {
					if ( !depthProcessOptions.isRemoveBackgroundEnabled() ) {
						iter.r() = iter.g() = iter.b() = v;
					} else {
						iter.r() = iter.g() = iter.b() = 0xFFFF;
					}
				} else {
					Colorf color	= getUserColor( id );
					iter.r()		= (uint16_t)( (float)v * color.r );
					iter.g()		= (uint16_t)( (float)v * color.g );
					iter.b()		= (uint16_t)( (float)v * color.b );
				}
			} else if ( depthProcessOptions.isRemoveBackgroundEnabled() ) {
				if ( id < 1 || id > 6 ) {
					iter.r() = iter.g() = iter.b() = 0xFFFF;
				} else {
					iter.r() = iter.g() = iter.b() = v;
				}
			} else {
				iter.r() = iter.g() = iter.b() = v;
			}
		}
	}
	return surface;
}

void CALLBACK deviceStatus( long hr, const wchar_t *instanceName, const wchar_t *deviceId, void *data )
{
	Device* device = reinterpret_cast<Device*>( data );
	if ( SUCCEEDED( hr ) ) {
		device->start( device->getDeviceOptions() );
	} else {
		device->errorNui( hr );
		device->stop();
	}
}

float getDepthAtCoord( const Channel16u& depth, const Vec2i& v ) 
{
	float depthNorm	= 0.0f;
	if ( depth ) {
		uint16_t d	= depth.getValue( v );
		d			= 0x10000 * ( ( d & 0xFFF8 ) >> 3 ) / 0x0FFF;
		d			= 0x10000 - d;
		d			= d << 2;
		depthNorm	= 1.0f - (float)d / 65535.0f;
	}
	return depthNorm;
}

size_t getDeviceCount()
{
	return KinectGetPortIDCount();
}

Colorf getUserColor( uint32_t id ) 
{
	switch ( id ) {
	case 0:
		return Colorf::black();
	case 1:
		return Colorf( 1.0f, 0.0f, 0.0f );
	case 2:
		return Colorf( 0.0f, 1.0f, 0.0f );
	case 3:
		return Colorf( 0.0f, 0.0f, 1.0f );
	case 4:
		return Colorf( 1.0f, 1.0f, 0.0f );
	case 5:
		return Colorf( 0.0f, 1.0f, 1.0f );
	case 6:
		return Colorf( 1.0f, 0.0f, 1.0f );
	default:
		return Colorf::white();
	}
}

Vec2i mapColorCoordToDepth( const Vec2i& v, const Channel16u& depth, const DeviceRef& device )
{
	long x;
	long y;
	if ( depth && device ) {
		NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
			device->getDeviceOptions().getColorResolution(), 
			device->getDeviceOptions().getDepthResolution(), 
			0, v.x, v.y, 
			depth.getValue( v ), &x, &y );
	}
	return Vec2i( (int32_t)x, (int32_t)y );
}

Vec2i mapDepthCoordToColor( const Vec2i& v, const Channel16u& depth, const DeviceRef& device )
{
	NUI_COLOR_IMAGE_POINT mapped;
	if ( depth && device && device->getCoordinateMapper() ) {
		NUI_DEPTH_IMAGE_POINT p;
		p.x		= v.x;
		p.y		= v.y;
		p.depth	= depth.getValue( v );
		long hr = device->getCoordinateMapper()->MapDepthPointToColorPoint( 
			device->getDeviceOptions().getDepthResolution(), &p, 
			NUI_IMAGE_TYPE::NUI_IMAGE_TYPE_COLOR_INFRARED, 
			device->getDeviceOptions().getColorResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			device->errorNui( hr );
		}
	}
	return Vec2i( mapped.x, mapped.y );
}

Vec2i mapSkeletonCoordToColor( const Vec3f& v, const DeviceRef& device )
{
	NUI_COLOR_IMAGE_POINT mapped;
	if ( device && device->getCoordinateMapper() ) {
		Vector4 p;
		p.x		= v.x;
		p.y		= v.y;
		p.z		= v.z;
		p.w		= 0.0f;
		long hr	= device->getCoordinateMapper()->MapSkeletonPointToColorPoint( 
			&p, NUI_IMAGE_TYPE::NUI_IMAGE_TYPE_COLOR, 
			device->getDeviceOptions().getColorResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			device->errorNui( hr );
		}
	}
	return Vec2i( mapped.x, mapped.y );
}

Vec2i mapSkeletonCoordToDepth( const Vec3f& v, const DeviceRef& device )
{
	NUI_DEPTH_IMAGE_POINT mapped;
	if ( device && device->getCoordinateMapper() ) {
		Vector4 p;
		p.x		= v.x;
		p.y		= v.y;
		p.z		= v.z;
		p.w		= 0.0f;
		long hr	= device->getCoordinateMapper()->MapSkeletonPointToDepthPoint( 
			&p, device->getDeviceOptions().getDepthResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			device->errorNui( hr );
		}
	}
	return Vec2i( mapped.x, mapped.y );
}

uint16_t userIdFromDepthCoord( const Channel16u& depth, const Vec2i& v )
{
	return NuiDepthPixelToPlayerIndex( depth.getValue( v ) );
}

const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformNone			= { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformDefault		= { 0.5f, 0.5f, 0.5f, 0.05f, 0.04f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformSmooth		= { 0.5f, 0.1f, 0.5f, 0.1f, 0.1f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformVerySmooth	= { 0.7f, 0.3f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformMax			= { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
const NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformParams[ 5 ]	= 
{ kTransformNone, kTransformDefault, kTransformSmooth, kTransformVerySmooth, kTransformMax };

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceOptions::DeviceOptions()
: mDeviceId( "" ), mDeviceIndex( 0 ), mEnabledColor( true ), mEnabledDepth( true ),
mEnabledInfrared( false ), mEnabledNearMode( false ), mEnabledSeatedMode( false ),
mEnabledUserTracking( true ), mSkeletonTransform( SkeletonTransform::TRANSFORM_DEFAULT ),
mSkeletonSelectionMode( SkeletonSelectionMode::SkeletonSelectionModeDefault )
{
	setColorResolution( ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	setDepthResolution( ImageResolution::NUI_IMAGE_RESOLUTION_320x240 );
	setInfraredResolution( ImageResolution::NUI_IMAGE_RESOLUTION_320x240 );
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfrared( bool enable )
{
	mEnabledInfrared = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableNearMode( bool enable )
{
	mEnabledNearMode = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableSeatedMode( bool enable )
{
	mEnabledSeatedMode = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableUserTracking( bool enable )
{
	mEnabledUserTracking = enable;
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

ImageResolution DeviceOptions::getInfraredResolution() const
{
	return mInfraredResolution;
}
	
const Vec2i& DeviceOptions::getInfraredSize() const
{
	return mInfraredSize;
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

bool DeviceOptions::isInfraredEnabled() const
{
	return mEnabledInfrared;
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
	mColorResolution		= resolution;
	switch ( mColorResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		mColorSize			= Vec2i( 1280, 960 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mColorSize			= Vec2i( 640, 480 );
		break;
	default:
		mColorResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mColorSize			= Vec2i::zero();
		mEnabledColor		= false;
		break;
	}
	return *this;
}

DeviceOptions& DeviceOptions::setDepthResolution( const ImageResolution& resolution )
{
	mDepthResolution		= resolution;
	switch ( mDepthResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mDepthSize			= Vec2i( 640, 480 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mDepthSize			= Vec2i( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mDepthSize			= Vec2i( 80, 60 );
		break;
	default:
		mDepthResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mDepthSize			= Vec2i::zero();
		mEnabledDepth		= false;
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

DeviceOptions& DeviceOptions::setInfraredResolution( const ImageResolution& resolution )
{
	mInfraredResolution		= resolution;
	switch ( mInfraredResolution ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mInfraredSize		= Vec2i( 640, 480 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mInfraredSize		= Vec2i( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mInfraredSize		= Vec2i( 80, 60 );
		break;
	default:
		mInfraredResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mInfraredSize		= Vec2i::zero();
		mEnabledInfrared	= false;
		break;
	}
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

Frame::Frame( long long frameId, const std::string& deviceId, const Surface8u& color, 
			 const Channel16u& depth, const Channel16u& infrared, const std::vector<Skeleton>& skeletons ) 
: mColorSurface( color ), mDepthChannel( depth ), mDeviceId( deviceId ), 
mFrameId( frameId ), mInfraredChannel( infrared ), mSkeletons( skeletons )
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

const Channel16u& Frame::getInfraredChannel() const
{
	return mInfraredChannel;
}

const vector<Skeleton>&	Frame::getSkeletons() const
{
	return mSkeletons;
}

//////////////////////////////////////////////////////////////////////////////////////////////

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

INuiCoordinateMapper* Device::getCoordinateMapper() const
{
	return mCoordinateMapper;
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

void Device::init( bool reset )
{
	if ( !reset ) {
		mEventHandler	= nullptr;
		mVerbose		= true;
	}

	mBufferColor		= nullptr;
	mBufferDepth		= nullptr;
	mBufferInfrared		= nullptr;
	mCapture			= false;
	mCoordinateMapper	= 0;
	mFrameId			= 0;
    mKinect				= KCB_INVALID_HANDLE;
	mNuiSensor			= 0;
	mIsSkeletonDevice	= false;
	mTiltRequestTime	= 0.0;

	if ( mChannelDepth ) {
		mChannelDepth.reset();
	}
	if ( mChannelInfrared ) {
		mChannelInfrared.reset();
	}
	if ( mSurfaceColor ) {
		mSurfaceColor.reset();
	}
	for ( vector<Skeleton>::iterator iter = mSkeletons.begin(); iter != mSkeletons.end(); ++iter ) {
		iter->clear();
	}
}

bool Device::isCapturing() const 
{
	return mCapture; 
}

void Device::setTilt( int32_t degrees )
{
	double elapsedSeconds = getElapsedSeconds();
	if ( mCapture && mNuiSensor != 0 && elapsedSeconds - mTiltRequestTime > 1.5 ) {
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
		size_t count = KinectGetPortIDCount();
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

        if ( mKinect == KCB_INVALID_HANDLE ) {
			errorNui( hr );
			mDeviceOptions.setDeviceIndex( -1 );
			mDeviceOptions.setDeviceId( "" );
			if ( index >= 0 ) {
				console() << "Unable to create device instance " + toString( index ) + ": " << endl;
				throw ExcDeviceCreate( hr, deviceId );
			} else if ( deviceId.length() > 0 ) {
				console() << "Unable to create device instance " + deviceId + ":" << endl;
				throw ExcDeviceCreate( hr, deviceId );
			} else {
				console() << "Invalid device name or index." << endl;
				throw ExcDeviceInvalid( hr, deviceId );
			}
		}

		KINECT_SENSOR_STATUS status = KinectGetKinectSensorStatus( mKinect );
		statusKinect( status );
		if ( status > 1 ) {
			throw ExcDeviceInit( status, deviceId );
		}
		
		if ( mDeviceOptions.isColorEnabled() && mDeviceOptions.getColorResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatColor						= format;
			KinectEnableColorStream( mKinect, mDeviceOptions.getColorResolution(), &mFormatColor );
            if ( KinectGetColorStreamStatus( mKinect ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
				mBufferColor					= new uint8_t[ mFormatColor.cbBufferSize ];
			} else {
				mDeviceOptions.enableColor( false );
				console() << "Unable to initialize color stream: ";
				errorNui( hr );
				throw ExcOpenStreamColor( hr, deviceId );
			}
		}

		if ( mDeviceOptions.isDepthEnabled() && mDeviceOptions.getDepthResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatDepth						= format;
			KinectEnableDepthStream( mKinect, mDeviceOptions.isNearModeEnabled(), mDeviceOptions.getDepthResolution(), &mFormatDepth );
            if ( KinectGetDepthStreamStatus( mKinect ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
				mBufferDepth					= new uint8_t[ mFormatDepth.cbBufferSize ];
			} else {
				mDeviceOptions.enableDepth( false );
				console() << "Unable to initialize depth stream: ";
				errorNui( hr );
				throw ExcOpenStreamDepth( hr, deviceId );
			}
		}

		if ( mDeviceOptions.isInfraredEnabled() && mDeviceOptions.getInfraredResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatInfrared						= format;
			KinectEnableIRStream( mKinect, mDeviceOptions.getInfraredResolution(), &mFormatInfrared );
            if ( KinectGetIRStreamStatus( mKinect ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
				mBufferInfrared					= new uint8_t[ mFormatInfrared.cbBufferSize ];
			} else {
				mDeviceOptions.enableInfrared( false );
				console() << "Unable to initialize infrared stream: ";
				errorNui( hr );
				throw ExcOpenStreamInfrared( hr, deviceId );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() ) {
			KinectEnableSkeletonStream( mKinect, mDeviceOptions.isSeatedModeEnabled(), 
				mDeviceOptions.getSkeletonSelectionMode(), &kTransformParams[ mDeviceOptions.getSkeletonTransform() ] );
			if ( KinectGetSkeletonStreamStatus( mKinect ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
				mIsSkeletonDevice = true;
			} else {
				mDeviceOptions.enableUserTracking( false );
				console() << "Unable to initialize user tracking: ";
				errorNui( hr );
				throw ExcUserTrackingEnable( hr, deviceId );
			}
		}

		mSkeletons.clear();
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
			mSkeletons.push_back( Skeleton() );
		}

		hr = KinectStartStreams( mKinect );
		if ( FAILED( hr ) ) {
			console() << "Unable to start the streams: ";
			errorNui( hr );
			throw ExcStreamStart( hr, deviceId );
		}

		hr = mNuiSensor->NuiGetCoordinateMapper( &mCoordinateMapper );
		if ( FAILED( hr ) ) {
			console() << "Unable to initialize coordinate mapper: ";
			errorNui( hr );
			throw ExcGetCoordinateMapper( hr, deviceId );
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
	if ( mBufferInfrared != nullptr ) {
		delete [] mBufferInfrared;
	}
	KinectCloseSensor( mKinect );
	init( true );
}

void Device::update()
{
	if ( !KinectAllFramesReady( mKinect ) ) {
		return;
	}

	if ( mSurfaceColor ) {
		mSurfaceColor.reset();
	}
	if ( mChannelDepth ) {
		mChannelDepth.reset();
	}
	if ( mChannelInfrared ) {
		mChannelInfrared.reset();
	}

	long long timestamp;
	if ( mDeviceOptions.isColorEnabled() && 
		SUCCEEDED( KinectGetColorFrame( mKinect, mFormatColor.cbBufferSize, mBufferColor, &timestamp ) ) ) {
		mSurfaceColor = Surface8u( mBufferColor, 
			(int32_t)mFormatColor.dwWidth, (int32_t)mFormatColor.dwHeight, 
			(int32_t)mFormatColor.dwWidth *(int32_t) mFormatColor.cbBytesPerPixel, 
			SurfaceChannelOrder::BGRX );
	}
	if ( mDeviceOptions.isDepthEnabled() && 
		SUCCEEDED( KinectGetDepthFrame( mKinect, mFormatDepth.cbBufferSize, mBufferDepth, &timestamp ) ) ) {
		mChannelDepth = Channel16u( (int32_t)mFormatDepth.dwWidth, (int32_t)mFormatDepth.dwHeight, 
			(int32_t)mFormatDepth.dwWidth * (int32_t)mFormatDepth.cbBytesPerPixel, 0, 
			(uint16_t*)mBufferDepth );
    }
	if ( mDeviceOptions.isInfraredEnabled() && 
		SUCCEEDED( KinectGetColorFrame( mKinect, mFormatInfrared.cbBufferSize, mBufferInfrared, &timestamp ) ) ) {
		mChannelInfrared = Channel16u( (int32_t)mFormatInfrared.dwWidth, (int32_t)mFormatInfrared.dwHeight, 
			(int32_t)mFormatInfrared.dwWidth * (int32_t)mFormatInfrared.cbBytesPerPixel, 0, 
			(uint16_t*)mBufferInfrared );
    }

	NUI_SKELETON_FRAME skeletonFrame;
	if ( mDeviceOptions.isUserTrackingEnabled() && 
		SUCCEEDED( KinectGetSkeletonFrame( mKinect, &skeletonFrame ) ) ) {
		for ( int32_t i = 0; i < NUI_SKELETON_COUNT; ++i ) {
			mSkeletons.at( i ).clear();
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[ i ].eTrackingState;
			if ( trackingState == NUI_SKELETON_TRACKED ) {
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

	Frame frame( mFrameId, mDeviceOptions.getDeviceId(), mSurfaceColor, mChannelDepth, mChannelInfrared, mSkeletons );
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

Device::ExcGetCoordinateMapper::ExcGetCoordinateMapper( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to get coordinate mapper: %s. Error: %i", id, hr );
}

Device::ExcOpenStreamColor::ExcOpenStreamColor( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to open color stream: %s. Error: %i", hr );
}

Device::ExcOpenStreamDepth::ExcOpenStreamDepth( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to open depth stream. Error: %s: %i", hr );
}

Device::ExcOpenStreamInfrared::ExcOpenStreamInfrared( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to open infrared stream. Error: %s: %i", hr );
}

Device::ExcStreamStart::ExcStreamStart( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to start streams: %s. Error: %i", hr );
}

Device::ExcUserTrackingEnable::ExcUserTrackingEnable( long hr, const string& id ) throw()
{
	sprintf( mMessage, "Unable to enable user tracking: %s. Error: %i", hr );
}
}
 