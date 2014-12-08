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

mat4 toMat4( const Matrix4& m ) 
{
	return mat4( vec4( m.M11, m.M12, m.M13, m.M14 ), 
		vec4( m.M21, m.M22, m.M23, m.M24 ), 
		vec4( m.M31, m.M32, m.M33, m.M34 ), 
		vec4( m.M41, m.M42, m.M43, m.M44 ) );
}

quat toQuat( const Vector4& v ) 
{
	return quat( v.w, v.x, v.y, v.z );
}

vec3 toVec3( const Vector4& v ) 
{
	return vec3( v.x, v.y, v.z );
}

vec4 toVec4( const Vector4& v ) 
{
	return vec4( v.x, v.y, v.z, v.w );
}

//////////////////////////////////////////////////////////////////////////////////////////////

Face::Face()
{
	mPoseMatrix = mat4( 0.0f );
	mUserId = 0;
}

const AnimationUnitMap& Face::getAnimationUnits() const
{
	return mAnimationUnits;
}

const Rectf& Face::getBounds() const
{
	return mBounds;
}

const TriMeshRef& Face::getMesh() const
{
	return mMesh;
}

const TriMeshRef& Face::getMesh2d() const
{
	return mMesh2d;
}

const mat4& Face::getPoseMatrix() const
{
	return mPoseMatrix;
}

size_t Face::getUserId() const
{
	return mUserId;
}

//////////////////////////////////////////////////////////////////////////////////////////////

FaceTrackerRef FaceTracker::create()
{
	return FaceTrackerRef( new FaceTracker() );
}

FaceTracker::FaceTracker()
	: mCalcMesh( true ), mCalcMesh2d( true ), mEventHandler( nullptr ), mFaceTracker( 0 ), 
	mModel( 0 ), mNewFace( false ), mResult( 0 ), mRunning( false ), mUserId( 0 )
{
}

FaceTracker::~FaceTracker()
{
	stop();

	if ( mFaceTracker != 0 ) {
		mFaceTracker->Release();
		mFaceTracker = 0;
	}

	if ( mSensorData.pVideoFrame != 0 ) {
		mSensorData.pVideoFrame->Release();
		mSensorData.pVideoFrame = 0;
	}

	if ( mSensorData.pDepthFrame != 0 ) {
		mSensorData.pDepthFrame->Release();
		mSensorData.pDepthFrame = 0;
	}

	if ( mModel != 0 ) {
		mModel->Release();
		mModel = 0;
	}

	if ( mResult != 0 ) {
		mResult->Release();
		mResult = 0;
	}
}

IFTFaceTracker* FaceTracker::getFaceTracker() const
{
	return mFaceTracker;
}

IFTModel* FaceTracker::getModel() const
{
	return mModel;
}

IFTResult* FaceTracker::getResult() const
{
	return mResult;
}

void FaceTracker::enableCalcMesh( bool enabled )
{
	mCalcMesh = enabled;
}

void FaceTracker::enableCalcMesh2d( bool enabled )
{
	mCalcMesh2d = enabled;
}

bool FaceTracker::isCalcMeshEnabled() const
{
	return mCalcMesh;
}

bool FaceTracker::isCalcMesh2dEnabled() const
{
	return mCalcMesh2d;
}

bool FaceTracker::isTracking() const
{
	return mRunning;
}

void FaceTracker::start( const DeviceOptions& deviceOptions )
{
	stop();

	mConfigColor.Height				= deviceOptions.getColorSize().y;
	mConfigColor.Width				= deviceOptions.getColorSize().x;
	switch ( deviceOptions.getColorResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mConfigColor.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_1280x960:
		mConfigColor.FocalLength	= NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		mConfigColor.FocalLength	= 0.0f;
		break;
	}

	mConfigDepth.Height				= deviceOptions.getDepthSize().y;
	mConfigDepth.Width				= deviceOptions.getDepthSize().x;
	switch ( deviceOptions.getDepthResolution() ) {
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 0.25f;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mConfigDepth.FocalLength	= NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.0f;
		break;
	default:
		mConfigDepth.FocalLength	= 0.0f;
		break;
	}

	long hr = S_OK;
	mFaceTracker = FTCreateFaceTracker();
    if ( !mFaceTracker ) {
		throw ExcFaceTrackerCreate();
	}

	hr = mFaceTracker->Initialize( &mConfigColor, &mConfigDepth, 0, 0 );
	if ( FAILED( hr ) ) {
		throw ExcFaceTrackerInit( hr );
    }

	hr = mFaceTracker->CreateFTResult( &mResult );
	if ( FAILED( hr ) || mResult == 0 ) {
		throw ExcFaceTrackerCreateResult( hr );
	}

	tagPOINT offset;
	offset.x			= 0;
	offset.y			= 0;
	mSensorData.pDepthFrame	= FTCreateImage();
	mSensorData.pVideoFrame	= FTCreateImage();
	mSensorData.ViewOffset	= offset;
	mSensorData.ZoomFactor	= 1.0f;
	
	mRunning	= true;
	mThread		= ThreadRef( new thread( &FaceTracker::run, this ) );
}

void FaceTracker::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}
}

void FaceTracker::update( const Surface8u& color, const Channel16u& depth, const vec3 headPoints[ 2 ], size_t userId )
{
	if ( mNewFace && mEventHandler != nullptr ) {
		mEventHandler( mFace );
		if ( color && depth ) {
			mHeadPoints.clear();
			if ( headPoints != 0 ) {
				mHeadPoints.push_back( headPoints[ 0 ] );
				mHeadPoints.push_back( headPoints[ 1 ] );
			}

			bool attach = !mChannelDepth || !mSurfaceColor;

			mChannelDepth	= depth;
			mSurfaceColor	= color;
			mUserId			= userId;

			if ( attach ) {
				mSensorData.pVideoFrame->Attach( mSurfaceColor.getWidth(), mSurfaceColor.getHeight(), 
					(void*)mSurfaceColor.getData(), FTIMAGEFORMAT_UINT8_B8G8R8X8, mSurfaceColor.getWidth() * 4 );
				mSensorData.pDepthFrame->Attach( mChannelDepth.getWidth(), mChannelDepth.getHeight(), 
					(void*)mChannelDepth.getData(), FTIMAGEFORMAT_UINT16_D13P3,	mChannelDepth.getWidth() * 2 );
			}
		}
		mNewFace = false;
	}
}

void FaceTracker::run()
{
	while ( mRunning ) {
		lock_guard<mutex> lock( mMutex );
		if ( !mNewFace ) {
			long hr = S_OK;

			Face face;
			face.mBounds	= Rectf( 0.0f, 0.0f, 0.0f, 0.0f );
			face.mMesh		= TriMesh::create( TriMesh::Format().positions() );
			face.mMesh2d	= TriMesh::create( TriMesh::Format().positions() );
			face.mUserId	= mUserId;
			face.mPoseMatrix.setToIdentity();

			FT_VECTOR3D* hint = 0;
			if ( mHeadPoints.size() == 2 ) {
				hint = new FT_VECTOR3D[ 2 ];
				for ( size_t i = 0; i < 2; ++i ) {
					hint[ i ] = FT_VECTOR3D( mHeadPoints[ i ].x, mHeadPoints[ i ].y, mHeadPoints[ i ].z );
				}
			}

			if ( mSuccess ) {
				hr = mFaceTracker->ContinueTracking( &mSensorData, hint, mResult );
			} else {
				hr = mFaceTracker->StartTracking( &mSensorData, 0, hint, mResult );
			}

			if ( hint != 0 ) {
				delete [] hint;
			}

			mSuccess = SUCCEEDED( hr ) && SUCCEEDED( mResult->GetStatus() );
		
			if ( mSuccess ) {
				hr = mFaceTracker->GetFaceModel( &mModel );

				if ( SUCCEEDED( hr ) ) {
					float* shapeUnits	= 0;
					UINT numShapeUnits	= 0;
					int32_t haveConverged	= false;
					mFaceTracker->GetShapeUnits( 0, &shapeUnits, &numShapeUnits, &haveConverged );
							
					float* animationUnits	= 0;
					UINT numAnimationUnits	= 0;
					hr = mResult->GetAUCoefficients( &animationUnits, &numAnimationUnits );
					if ( SUCCEEDED( hr ) ) {
						for ( size_t i = 0; i < numAnimationUnits; ++i ) {
							face.mAnimationUnits[ (AnimationUnit)i ] = animationUnits[ i ];
						}
					}

					float scale;
					float rotation[ 3 ];
					float translation[ 3 ];
					hr = mResult->Get3DPose( &scale, rotation, translation );
					if ( SUCCEEDED( hr ) ) {
						vec3 r( rotation[ 0 ], rotation[ 1 ], rotation[ 2 ] );
						vec3 t( translation[ 0 ], translation[ 1 ], translation[ 2 ] );

						face.mPoseMatrix.translate( t );
						face.mPoseMatrix.rotate( r );
						face.mPoseMatrix.translate( -t );
						face.mPoseMatrix.translate( t );
						face.mPoseMatrix.scale( vec3::one() * scale );
					}

					size_t numVertices	= mModel->GetVertexCount();
							
					if ( numAnimationUnits > 0 && numShapeUnits > 0 && numVertices > 0 ) {
						if ( mCalcMesh ) {
							FT_VECTOR3D* pts		= new FT_VECTOR3D[ numVertices ];
							pts[ numVertices - 1 ]	= 0;
							hr = mModel->Get3DShape( shapeUnits, numShapeUnits, animationUnits, numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									vec3 v( pts[ i ].x, pts[ i ].y, pts[ i ].z );
									face.mMesh->appendVertex( v );
								}

								FT_TRIANGLE* triangles	= 0;
								UINT triangleCount		= 0;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										face.mMesh->appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							delete [] pts;
						}

						if ( mCalcMesh2d ) {
							tagPOINT viewOffset	= { 0, 0 };
							FT_VECTOR2D* pts		= new FT_VECTOR2D[ numVertices ];
							pts[ numVertices - 1 ]	= 0;
							hr = mModel->GetProjectedShape( &mConfigColor, mSensorData.ZoomFactor, viewOffset, shapeUnits, numShapeUnits, animationUnits, 
								numAnimationUnits, scale, rotation, translation, pts, numVertices );
							if ( SUCCEEDED( hr ) ) {
								for ( size_t i = 0; i < numVertices; ++i ) {
									vec2 v( pts[ i ].x + 0.5f, pts[ i ].y + 0.5f );
									face.mMesh2d->appendVertex( vec3( v.x, v.y, 0.0f ) );
								}

								FT_TRIANGLE* triangles	= 0;
								UINT triangleCount		= 0;
								hr = mModel->GetTriangles( &triangles, &triangleCount );
								if ( SUCCEEDED( hr ) ) {
									for ( size_t i = 0; i < triangleCount; ++i ) {
										face.mMesh2d->appendTriangle( triangles[ i ].i, triangles[ i ].j, triangles[ i ].k );
									}
								}
							}
							delete [] pts;
						}
					}

					tagRECT rect;
					hr = mResult->GetFaceRect( &rect );
					if ( SUCCEEDED( hr ) ) {
						face.mBounds = Rectf( (float)rect.left, (float)rect.top, (float)rect.right, (float)rect.bottom );
					}
				}
			} else {
				mResult->Reset();
			}

			mFace		= face;
			mNewFace	= true;
		}
	}
}

void FaceTracker::connectEventHander( const function<void( Face )>& eventHandler )
{
	mEventHandler = eventHandler;
}

//////////////////////////////////////////////////////////////////////////////////////////////

const char* FaceTracker::Exception::what() const throw() 
{ 
	return mMessage; 
}

FaceTracker::ExcFaceTrackerCreate::ExcFaceTrackerCreate() throw()
{
	sprintf( mMessage, "Unable to create face tracker." );
}

FaceTracker::ExcFaceTrackerCreateImage::ExcFaceTrackerCreateImage( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker image. Error: %i", hr );
}

FaceTracker::ExcFaceTrackerCreateResult::ExcFaceTrackerCreateResult( long hr ) throw()
{
	sprintf( mMessage, "Unable to create face tracker result. Error: %i", hr );
}

FaceTracker::ExcFaceTrackerInit::ExcFaceTrackerInit( long hr ) throw()
{
	sprintf( mMessage, "Unable to initialize face tracker. Error: %i", hr );
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

Bone::Bone( const Vector4& position, const _NUI_SKELETON_BONE_ORIENTATION& bone, JointTrackingState trackingState )
{
	mAbsRotQuat		= toQuat( bone.absoluteRotation.rotationQuaternion );
	mAbsRotMat		= toMat4( bone.absoluteRotation.rotationMatrix );
	mJointEnd		= bone.endJoint;
	mJointStart		= bone.startJoint;
	mPosition		= toVec3( position );
	mRotQuat		= toQuat( bone.hierarchicalRotation.rotationQuaternion );
	mRotMat			= toMat4( bone.hierarchicalRotation.rotationMatrix );
	mTrackingState	= trackingState;
}

const quat& Bone::getAbsoluteRotation() const 
{ 
	return mAbsRotQuat; 
}

const mat4& Bone::getAbsoluteRotationMatrix() const 
{ 
	return mAbsRotMat; 
}

JointName Bone::getEndJoint() const
{
	return mJointEnd;
}

const vec3& Bone::getPosition() const 
{ 
	return mPosition; 
}

const quat& Bone::getRotation() const 
{ 
	return mRotQuat; 
}

const mat4& Bone::getRotationMatrix() const 
{ 
	return mRotMat; 
}

JointName Bone::getStartJoint() const
{
	return mJointStart;
}

JointTrackingState Bone::getTrackingState() const
{
	return mTrackingState;
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

float calcSkeletonConfidence( const Skeleton& skeleton, bool weighted )
{
	// TODO weighted
	float c = 0.0f;
	if ( weighted ) {
		static map<JointName, float> weights;
		if ( weights.empty() ) {
			weights[ JointName::NUI_SKELETON_POSITION_HIP_CENTER ]		= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_SPINE ]			= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_SHOULDER_CENTER ]	= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_HEAD ]			= 0.045454545f;
			weights[ JointName::NUI_SKELETON_POSITION_SHOULDER_LEFT ]	= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_ELBOW_LEFT ]		= 0.045454545f;
			weights[ JointName::NUI_SKELETON_POSITION_WRIST_LEFT ]		= 0.022727273f;
			weights[ JointName::NUI_SKELETON_POSITION_HAND_LEFT ]		= 0.011363636f;
			weights[ JointName::NUI_SKELETON_POSITION_SHOULDER_RIGHT ]	= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_ELBOW_RIGHT ]		= 0.045454545f;
			weights[ JointName::NUI_SKELETON_POSITION_WRIST_RIGHT ]		= 0.022727273f;
			weights[ JointName::NUI_SKELETON_POSITION_HAND_RIGHT ]		= 0.011363636f;
			weights[ JointName::NUI_SKELETON_POSITION_HIP_LEFT ]		= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_KNEE_LEFT ]		= 0.045454545f;
			weights[ JointName::NUI_SKELETON_POSITION_ANKLE_LEFT ]		= 0.022727273f;
			weights[ JointName::NUI_SKELETON_POSITION_FOOT_LEFT ]		= 0.011363636f;
			weights[ JointName::NUI_SKELETON_POSITION_HIP_RIGHT ]		= 0.090909091f;
			weights[ JointName::NUI_SKELETON_POSITION_KNEE_RIGHT ]		= 0.045454545f;
			weights[ JointName::NUI_SKELETON_POSITION_ANKLE_RIGHT ]		= 0.022727273f;
			weights[ JointName::NUI_SKELETON_POSITION_FOOT_RIGHT ]		= 0.011363636f;
		}
		for ( Skeleton::const_iterator iter = skeleton.begin(); iter != skeleton.end(); ++iter ) {
			if ( iter->second.getTrackingState() == JointTrackingState::NUI_SKELETON_POSITION_TRACKED ) {
				c += weights.at( iter->first );
			}
		}
	} else {
		for ( Skeleton::const_iterator iter = skeleton.begin(); iter != skeleton.end(); ++iter ) {
			if ( iter->second.getTrackingState() == JointTrackingState::NUI_SKELETON_POSITION_TRACKED ) {
				c += 1.0f;
			}
		}
		c /= (float)JointName::NUI_SKELETON_POSITION_COUNT;
	}
	return c;
}

Channel8u channel16To8( const Channel16u& channel )
{
	Channel8u channel8;
	if ( channel ) {
		channel8						= Channel8u( channel.getWidth(), channel.getHeight() );
		Channel16u::ConstIter iter16	= channel.getIter();
		Channel8u::Iter iter8			= channel8.getIter();
		while ( iter8.line() && iter16.line() ) {
			while ( iter8.pixel() && iter16.pixel() ) {
				iter8.v()				= iter16.v() >> 4;
			}
		}
	}
	return channel8;
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

float getDepthAtCoord( const Channel16u& depth, const ivec2& v ) 
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

uint16_t userIdFromDepthCoord( const Channel16u& depth, const ivec2& v )
{
	return NuiDepthPixelToPlayerIndex( depth.getValue( v ) );
}

NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformNone			= { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformDefault		= { 0.5f, 0.5f, 0.5f, 0.05f, 0.04f };
NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformSmooth		= { 0.5f, 0.1f, 0.5f, 0.1f, 0.1f };
NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformVerySmooth	= { 0.7f, 0.3f, 1.0f, 1.0f, 1.0f };
NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformMax			= { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
NUI_TRANSFORM_SMOOTH_PARAMETERS	kTransformParams[ 5 ]	= 
{ kTransformNone, kTransformDefault, kTransformSmooth, kTransformVerySmooth, kTransformMax };

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceOptions::DeviceOptions()
: mDeviceHandle( KCB_INVALID_HANDLE ), mDeviceId( "" ), mDeviceIndex( 0 ), mEnabledColor( true ), 
mEnabledDepth( true ), mEnabledInfrared( false ), mEnabledNearMode( false ), mEnabledSeatedMode( false ),
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
	if ( mEnabledColor ) {
		mEnabledInfrared = false;
	}
	return *this;
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableFaceTracking( bool enable )
{
	mEnabledFaceTracking = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfrared( bool enable )
{
	mEnabledInfrared = enable;
	if ( mEnabledInfrared ) {
		mEnabledColor = false;
	}
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

const ivec2& DeviceOptions::getColorSize() const 
{
	return mColorSize;
}

ImageResolution DeviceOptions::getDepthResolution() const
{
	return mDepthResolution;
}
	
const ivec2& DeviceOptions::getDepthSize() const
{
	return mDepthSize;
}

KCBHANDLE DeviceOptions::getDeviceHandle() const
{
	return mDeviceHandle;
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
	
const ivec2& DeviceOptions::getInfraredSize() const
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

bool DeviceOptions::isFaceTrackingEnabled() const
{
	return mEnabledFaceTracking;
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
		mColorSize			= ivec2( 1280, 960 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_640x480:
		mColorSize			= ivec2( 640, 480 );
		break;
	default:
		mColorResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mColorSize			= ivec2( 0 );
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
		mDepthSize			= ivec2( 640, 480 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mDepthSize			= ivec2( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mDepthSize			= ivec2( 80, 60 );
		break;
	default:
		mDepthResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mDepthSize			= ivec2( 0 );
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
		mInfraredSize		= ivec2( 640, 480 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_320x240:
		mInfraredSize		= ivec2( 320, 240 );
		break;
	case ImageResolution::NUI_IMAGE_RESOLUTION_80x60:
		mInfraredSize		= ivec2( 80, 60 );
		break;
	default:
		mInfraredResolution	= NUI_IMAGE_RESOLUTION_INVALID;
		mInfraredSize		= ivec2( 0 );
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
	: mDeviceId( "" ), mFloorClipPlane( vec4( 0.0f ) ), mFrameId( 0 ), mNormalToGravity( vec3( 0.0f ) )
{
}

Frame::Frame( long long frameId, const std::string& deviceId, const Surface8u& color, 
			 const Channel16u& depth, const Channel16u& infrared, const std::vector<Skeleton>& skeletons, 
			 const Face& face, const vec4& floorClipPlane, const vec3& normalToGravity ) 
: mColorSurface( color ), mDepthChannel( depth ), mDeviceId( deviceId ), mFace( face ), 
mFloorClipPlane( floorClipPlane ), mFrameId( frameId ), mInfraredChannel( infrared ), 
mNormalToGravity( normalToGravity ), mSkeletons( skeletons )
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

const Face&	Frame::getFace() const
{
	return mFace;
}

const vec4& Frame::getFloorClipPlane() const 
{
	return mFloorClipPlane;
}

long long Frame::getFrameId() const
{
	return mFrameId;
}

const Channel16u& Frame::getInfraredChannel() const
{
	return mInfraredChannel;
}

const vec3& Frame::getNormalToGravity() const 
{
	return mNormalToGravity;
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

FaceTrackerRef& Device::getFaceTracker()
{
	return mFaceTracker;
}

const FaceTrackerRef& Device::getFaceTracker() const
{
	return mFaceTracker;
}

quat Device::getOrientation() const
{
	Vector4 v;
	if ( mNuiSensor != 0 ) {
		mNuiSensor->NuiAccelerometerGetCurrentReading( &v );
	}
	return toQuat( v );
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

ivec2 Device::mapColorCoordToDepth( const ivec2& v )
{
	long x;
	long y;
	if ( mChannelDepth ) {
		long hr = NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution( 
			mDeviceOptions.getColorResolution(), 
			mDeviceOptions.getDepthResolution(), 
			0, v.x, v.y, 
			mChannelDepth.getValue( v ), &x, &y );
		if ( FAILED( hr ) ) {
			errorNui( hr );
		}
	}
	return ivec2( (int32_t)x, (int32_t)y );
}

ivec2 Device::mapDepthCoordToColor( const ivec2& v )
{
	NUI_COLOR_IMAGE_POINT mapped;
	if ( mChannelDepth && mCoordinateMapper ) {
		NUI_DEPTH_IMAGE_POINT p;
		p.x		= v.x;
		p.y		= v.y;
		p.depth	= mChannelDepth.getValue( v );
		long hr = mCoordinateMapper->MapDepthPointToColorPoint( 
			mDeviceOptions.getDepthResolution(), &p, 
			NUI_IMAGE_TYPE::NUI_IMAGE_TYPE_COLOR_INFRARED, 
			mDeviceOptions.getColorResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			errorNui( hr );
		}
	}
	return ivec2( mapped.x, mapped.y );
}

ivec2 Device::mapSkeletonCoordToColor( const vec3& v )
{
	NUI_COLOR_IMAGE_POINT mapped;
	if ( mCoordinateMapper ) {
		Vector4 p;
		p.x		= v.x;
		p.y		= v.y;
		p.z		= v.z;
		p.w		= 0.0f;
		long hr	= mCoordinateMapper->MapSkeletonPointToColorPoint( 
			&p, NUI_IMAGE_TYPE::NUI_IMAGE_TYPE_COLOR, 
			mDeviceOptions.getColorResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			errorNui( hr );
		}
	}
	return ivec2( mapped.x, mapped.y );
}

ivec2 Device::mapSkeletonCoordToDepth( const vec3& v )
{
	NUI_DEPTH_IMAGE_POINT mapped;
	if ( mCoordinateMapper ) {
		Vector4 p;
		p.x		= v.x;
		p.y		= v.y;
		p.z		= v.z;
		p.w		= 0.0f;
		long hr	= mCoordinateMapper->MapSkeletonPointToDepthPoint( 
			&p, mDeviceOptions.getDepthResolution(), &mapped 
			);
		if ( FAILED( hr ) ) {
			errorNui( hr );
		}
	}
	return ivec2( mapped.x, mapped.y );
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

		if ( getDeviceCount() == 0 ) {
			console( ) << "No devices available" << endl;
			throw ExcDeviceUnavailable();
		}

		wchar_t portId[ KINECT_MAX_PORTID_LENGTH ];
		size_t count = KinectGetPortIDCount();
		if ( index >= 0 ) {
			if ( KinectGetPortIDByIndex( index, _countof( portId ), portId ) ) {
				mDeviceOptions.mDeviceHandle	= KinectOpenSensor( portId );
				hr								= NuiCreateSensorById( portId, &mNuiSensor );
				mDeviceOptions.setDeviceId( wcharToString( portId ) );
			}
		} else if ( deviceId.length() > 0 ) {
			_bstr_t id = deviceId.c_str();
			hr = NuiCreateSensorById( id, &mNuiSensor );
			for ( size_t i = 0; i < count; ++i ) {
				if ( deviceId == wcharToString( portId ) && KinectGetPortIDByIndex( i, _countof( portId ), portId ) ) {
					mDeviceOptions.mDeviceHandle	= KinectOpenSensor( portId );
					hr		= NuiCreateSensorById( portId, &mNuiSensor );
					mDeviceOptions.setDeviceIndex( i );
					break;
				} 
			}
		}

        if ( mDeviceOptions.getDeviceHandle() == KCB_INVALID_HANDLE || mNuiSensor == 0 ) {
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

		KINECT_SENSOR_STATUS status = KinectGetKinectSensorStatus( mDeviceOptions.getDeviceHandle() );
		statusKinect( status );
		if ( status > 1 ) {
			throw ExcDeviceInit( status, deviceId );
		}
		
		if ( mDeviceOptions.isColorEnabled() && mDeviceOptions.getColorResolution() != ImageResolution::NUI_IMAGE_RESOLUTION_INVALID ) {
			KINECT_IMAGE_FRAME_FORMAT format	= { sizeof( KINECT_IMAGE_FRAME_FORMAT ), 0 };
			mFormatColor						= format;
			KinectEnableColorStream( mDeviceOptions.getDeviceHandle(), mDeviceOptions.getColorResolution(), &mFormatColor );
            if ( KinectGetColorStreamStatus( mDeviceOptions.getDeviceHandle() ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
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
			KinectEnableDepthStream( mDeviceOptions.getDeviceHandle(), mDeviceOptions.isNearModeEnabled(), mDeviceOptions.getDepthResolution(), &mFormatDepth );
            if ( KinectGetDepthStreamStatus( mDeviceOptions.getDeviceHandle() ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
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
			KinectEnableIRStream( mDeviceOptions.getDeviceHandle(), mDeviceOptions.getInfraredResolution(), &mFormatInfrared );
            if ( KinectGetIRStreamStatus( mDeviceOptions.getDeviceHandle() ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
				mBufferInfrared					= new uint8_t[ mFormatInfrared.cbBufferSize ];
			} else {
				mDeviceOptions.enableInfrared( false );
				console() << "Unable to initialize infrared stream: ";
				errorNui( hr );
				throw ExcOpenStreamInfrared( hr, deviceId );
			}
		}

		if ( mDeviceOptions.isUserTrackingEnabled() ) {
			KinectEnableSkeletonStream( mDeviceOptions.getDeviceHandle(), mDeviceOptions.isSeatedModeEnabled(), 
				mDeviceOptions.getSkeletonSelectionMode(), &kTransformParams[ mDeviceOptions.getSkeletonTransform() ] );
			if ( KinectGetSkeletonStreamStatus( mDeviceOptions.getDeviceHandle() ) != KINECT_STREAM_STATUS::KinectStreamStatusError ) {
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

		hr = KinectStartStreams( mDeviceOptions.getDeviceHandle() );
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

		if ( mDeviceOptions.isFaceTrackingEnabled() ) {
			mFaceTracker = FaceTracker::create();
			mFaceTracker->connectEventHander( [ & ]( MsKinect::Face face ) {
				mFace = face;
			} );
			mFaceTracker->start( mDeviceOptions );
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
	if ( mFaceTracker ) {
		mFaceTracker->stop();
		mFaceTracker.reset();
	}
	KinectCloseSensor( mDeviceOptions.getDeviceHandle() );
	init( true );
}

void Device::update()
{
	if ( !KinectAllFramesReady( mDeviceOptions.getDeviceHandle() ) ) {
		return;
	}

	vec4 floorClipPlane		= vec4( 0.0f );
	vec3 normalToGravity	= vec3( 0.0f );

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
		SUCCEEDED( KinectGetColorFrame( mDeviceOptions.getDeviceHandle(), mFormatColor.cbBufferSize, mBufferColor, &timestamp ) ) ) {
		mSurfaceColor = Surface8u( mBufferColor, 
			(int32_t)mFormatColor.dwWidth, (int32_t)mFormatColor.dwHeight, 
			(int32_t)mFormatColor.dwWidth *(int32_t) mFormatColor.cbBytesPerPixel, 
			SurfaceChannelOrder::BGRX );
	}
	if ( mDeviceOptions.isDepthEnabled() && 
		SUCCEEDED( KinectGetDepthFrame( mDeviceOptions.getDeviceHandle(), mFormatDepth.cbBufferSize, mBufferDepth, &timestamp ) ) ) {
		mChannelDepth = Channel16u( (int32_t)mFormatDepth.dwWidth, (int32_t)mFormatDepth.dwHeight, 
			(int32_t)mFormatDepth.dwWidth * (int32_t)mFormatDepth.cbBytesPerPixel, 0, 
			(uint16_t*)mBufferDepth );
    }
	if ( mDeviceOptions.isInfraredEnabled() && 
		SUCCEEDED( KinectGetIRFrame( mDeviceOptions.getDeviceHandle(), mFormatInfrared.cbBufferSize, mBufferInfrared, &timestamp ) ) ) {
		mChannelInfrared = Channel16u( (int32_t)mFormatInfrared.dwWidth, (int32_t)mFormatInfrared.dwHeight, 
			(int32_t)mFormatInfrared.dwWidth * (int32_t)mFormatInfrared.cbBytesPerPixel, 0, 
			(uint16_t*)mBufferInfrared );
    }

	NUI_SKELETON_FRAME skeletonFrame;
	if ( mDeviceOptions.isUserTrackingEnabled() && 
		SUCCEEDED( KinectGetSkeletonFrame( mDeviceOptions.getDeviceHandle(), &skeletonFrame ) ) ) {
		floorClipPlane	= toVec4( skeletonFrame.vFloorClipPlane );
		normalToGravity	= toVec3( skeletonFrame.vNormalToGravity );
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
					JointTrackingState trackingState = skeletonFrame.SkeletonData[ i ].eSkeletonPositionTrackingState[ j ];
					Bone bone( *( ( skeletonFrame.SkeletonData + i )->SkeletonPositions + j ), 
							   *( bones + j ), 
							   trackingState );
					( mSkeletons.begin() + i )->insert( std::pair<JointName, Bone>( (JointName)j, bone ) );
				}
			}
		}
	}

	if ( mDeviceOptions.isFaceTrackingEnabled() && mFaceTracker && mChannelDepth && mSurfaceColor ) {
		mFaceTracker->update( mSurfaceColor, mChannelDepth );
	}

	if ( mEventHandler ) {
		Frame frame( mFrameId, mDeviceOptions.getDeviceId(), mSurfaceColor, mChannelDepth, mChannelInfrared, 
					 mSkeletons, mFace, floorClipPlane, normalToGravity );
		mEventHandler( frame );
	}
	++mFrameId;
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

Device::ExcDeviceUnavailable::ExcDeviceUnavailable() throw()
{
	sprintf( mMessage, "No device available." );
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
 