#include "HandFinder.h"

#include "cinder/Rect.h"

namespace MsKinect
{
using namespace ci;
using namespace std;

Hand::Hand()
	: mPalmPosition( Vec2i::zero() )
{
}

Hand::Hand( const vector<Vec2i>& fingerTipPositions, const Vec2i& palmPosition )
{
	mFingerTipPositions = fingerTipPositions;
	mPalmPosition		= palmPosition;
}

const vector<Vec2i>& Hand::getFingerTipPositions() const
{
	return mFingerTipPositions;
}

const Vec2i& Hand::getPalmPosition() const
{
	return mPalmPosition;
}

vector<Hand> findHands( Channel16u depth, const vector<Skeleton>& skeletons, 
					   const DeviceOptions& deviceOptions )
{
	vector<Hand> hands;
	const ci::Vec2i& depthSize = deviceOptions.getDepthSize();
	for ( vector<Skeleton>::const_iterator skeletonIt = skeletons.begin(); skeletonIt != skeletons.end(); ++skeletonIt ) {
		for ( Skeleton::const_iterator boneIt = skeletonIt->begin(); boneIt != skeletonIt->end(); ++boneIt ) {
			if ( boneIt->first	== JointName::NUI_SKELETON_POSITION_WRIST_LEFT || 
				boneIt->first	== JointName::NUI_SKELETON_POSITION_WRIST_RIGHT ) {
				vector<Vec2i> fingers;
				const Bone& bone		= boneIt->second;
				const Vec3f& handPos	= bone.getPosition();
				Vec2i centroid			= mapSkeletonCoordToDepth( handPos, deviceOptions.getDepthResolution() );
				Vec3f palm( (float)centroid.x, (float)centroid.y, (float)depth.getValue( centroid ) );
				float radius			= (float)depthSize.y * 0.2f;
				float scale				= 1.0f - handPos.z * 0.25f;
				Area bounds( Rectf( 
					(float)centroid.x - radius, (float)centroid.y - radius, 
					(float)centroid.x + radius, (float)centroid.y + radius 
					) * scale );
				bounds.x1 = math<int32_t>::clamp( bounds.x1, 1, depthSize.x - 1 );
				bounds.y1 = math<int32_t>::clamp( bounds.y1, 1, depthSize.y - 1 );
				bounds.x2 = math<int32_t>::clamp( bounds.x2, bounds.x1, depthSize.x - 1 );
				bounds.y2 = math<int32_t>::clamp( bounds.y2, bounds.y1, depthSize.y - 1 );
				
				Vec2i v0;
				Vec2i v1;
				for ( v0.x = bounds.x1; v0.x <= bounds.x2; ++v0.x ) {
					for ( v0.y = bounds.y1; v0.y <= bounds.y2; ++v0.y ) {
						size_t wins	= 0;
						Vec3f pt0( (float)v0.x, (float)v0.y, (float)depth.getValue( v0 ) );
						float d0	= pt0.distance( palm );
						for ( v1.x = v0.x - 1; v1.x <= v0.x + 1; ++v1.x ) {
							for ( v1.y = v0.y - 1; v1.y <= v0.y + 1; ++v1.y ) {
								if ( v0 != v1 ) {
									Vec3f pt1( (float)v1.x, (float)v1.y, (float)depth.getValue( v1 ) );
									float d1 = pt1.distance( palm );
									if ( d1 > d0 ) {
										++wins;
										d0 = d1;
									}
								}
							}
						}
						if ( wins >= 8 ) {
							fingers.push_back( v0 );
						}
					}
				}
				if ( !fingers.empty() ) {
					hands.push_back( Hand( fingers, centroid ) );
				}
			}
		}
	}
	return hands;
}
	
}
