#pragma once

#include "Kinect.h"

namespace MsKinect
{
class Hand
{
public:
	Hand();

	const std::vector<ci::Vec2i>&	getFingerTipPositions() const;
	const ci::Vec2i&				getPalmPosition() const;
protected:
	Hand( const std::vector<ci::Vec2i>& fingerTipPositions, const ci::Vec2i& palmPosition );

	std::vector<ci::Vec2i>			mFingerTipPositions;
	ci::Vec2i						mPalmPosition;

	friend std::vector<Hand>		findHands( ci::Channel16u depth, const std::vector<Skeleton>& skeletons, 
											  const DeviceOptions& deviceOptions );
};

std::vector<Hand>					findHands( ci::Channel16u depth, const std::vector<Skeleton>& skeletons, 
											  const DeviceOptions& deviceOptions );
}
 