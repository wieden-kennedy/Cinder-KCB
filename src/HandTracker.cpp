/*
* 
* Copyright (c) 2013, Ban the Rewind, Wieden+Kennedy
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

#include "HandTracker.h"

#include "cinder/Rect.h"

namespace MsKinect
{
using namespace ci;
using namespace std;

HandTracker::Hand::Hand()
	: mPalmPosition( Vec2i::zero() )
{
}

HandTracker::Hand::Hand( const vector<Vec2i>& fingerTipPositions, const Vec2i& palmPosition, 
						JointName jointName, size_t userId )
{
	mFingerTipPositions = fingerTipPositions;
	mJointName			= jointName;
	mPalmPosition		= palmPosition;
	mUserId				= userId;
}

const vector<Vec2i>& HandTracker::Hand::getFingerTipPositions() const
{
	return mFingerTipPositions;
}

JointName HandTracker::Hand::getJointName() const
{
	return mJointName;
}

const Vec2i& HandTracker::Hand::getPalmPosition() const
{
	return mPalmPosition;
}

size_t HandTracker::Hand::getUserId() const
{
	return mUserId;
}

//////////////////////////////////////////////////////////////////////////////////////////////

HandTrackerRef HandTracker::create()
{
	return HandTrackerRef( new HandTracker() );
}

HandTracker::HandTracker()
	: mEventHandler( nullptr ), mImageResolution( ImageResolution::NUI_IMAGE_RESOLUTION_320x240 ), 
	mImageSize( 320, 240 ), mNewHands( false ), mRunning( false )
{
}

HandTracker::~HandTracker()
{
	stop();
}

void HandTracker::start( const DeviceOptions& deviceOptions )
{
	stop();

	mImageResolution	= deviceOptions.getDepthResolution();
	mImageSize			= deviceOptions.getDepthSize();
	mRunning			= true;
	mThread				= ThreadRef( new thread( &HandTracker::run, this ) );
}

void HandTracker::stop()
{
	mRunning = false;
	if ( mThread ) {
		mThread->join();
		mThread.reset();
	}

	if ( mDepthChannel ) {
		mDepthChannel.reset();
	}
	mHands.clear();
	mSkeletons.clear();
	mNewHands = false;
}

void HandTracker::update( const Channel16u& depth, const vector<Skeleton>& skeletons )
{
	if ( mNewHands && mEventHandler != nullptr ) {
		mEventHandler( mHands );
		if ( depth ) {
			mDepthChannel	= Surface16u( depth ).getChannelRed();
		}
		mSkeletons			= skeletons;
		mNewHands			= false;
	}
}

void HandTracker::connectEventHander( const function<void( vector<Hand> )>& eventHandler )
{
	mEventHandler = eventHandler;
}

void HandTracker::run()
{
	while ( mRunning ) {
		if ( !mNewHands && mDepthChannel ) {
			mHands.clear();
			size_t userId = 1;
			for ( vector<Skeleton>::const_iterator skeletonIt = mSkeletons.begin(); skeletonIt != mSkeletons.end(); ++skeletonIt, ++userId ) {
				for ( Skeleton::const_iterator boneIt = skeletonIt->begin(); boneIt != skeletonIt->end(); ++boneIt ) {
					if ( boneIt->first	== JointName::NUI_SKELETON_POSITION_WRIST_LEFT || 
						boneIt->first	== JointName::NUI_SKELETON_POSITION_WRIST_RIGHT ) {
						vector<Vec2i> fingers;
						const Bone& bone		= boneIt->second;
						const Vec3f& handPos	= bone.getPosition();
						Vec2i centroid			= mapSkeletonCoordToDepth( handPos, mImageResolution );
						Vec3f palm( (float)centroid.x, (float)centroid.y, getDepthAtCoord( mDepthChannel, centroid ) );

						float radius			= (float)mImageSize.y * 0.2f;
						float scale				= 1.0f - handPos.z * 0.25f;
						Area bounds( Rectf( 
							(float)centroid.x - radius, (float)centroid.y - radius, 
							(float)centroid.x + radius, (float)centroid.y + radius 
							) * scale );
						bounds.x1 = math<int32_t>::clamp( bounds.x1, 1, mImageSize.x - 1 );
						bounds.y1 = math<int32_t>::clamp( bounds.y1, 1, mImageSize.y - 1 );
						bounds.x2 = math<int32_t>::clamp( bounds.x2, bounds.x1, mImageSize.x - 1 );
						bounds.y2 = math<int32_t>::clamp( bounds.y2, bounds.y1, mImageSize.y - 1 );
				
						Vec2i v0;
						Vec2i v1;
						for ( v0.x = bounds.x1; v0.x <= bounds.x2; ++v0.x ) {
							for ( v0.y = bounds.y1; v0.y <= bounds.y2; ++v0.y ) {
								Vec3f pt0( (float)v0.x, (float)v0.y, getDepthAtCoord( mDepthChannel, v0 ) );
								float d0				= pt0.distance( palm );
								size_t numGreaterThan	= 0;
								for ( v1.x = v0.x - 1; v1.x <= v0.x + 1; ++v1.x ) {
									for ( v1.y = v0.y - 1; v1.y <= v0.y + 1; ++v1.y ) {
										if ( v0 != v1 ) {
											Vec3f pt1( (float)v1.x, (float)v1.y, getDepthAtCoord( mDepthChannel, v1 ) );
											float d1 = pt1.distance( palm );
											if ( d1 > d0 ) {
												++numGreaterThan;
												d0 = d1;
											}
										}
									}
								}
								if ( numGreaterThan >= 8 ) {
									fingers.push_back( v0 );
								}
							}
						}
						if ( !fingers.empty() ) {
							mHands.push_back( Hand( fingers, centroid, boneIt->first, userId ) );
						}
					}
				}
			}
			mNewHands = true;
		}
	}
}

}
