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

#pragma once

#include "Kinect.h"
#include <memory>
#include <thread>

namespace MsKinect
{
typedef std::shared_ptr<class HandTracker> HandTrackerRef;

class HandTracker
{
public:
	
	//////////////////////////////////////////////////////////////////////////////////////////////

	class Hand
	{
	public:
		Hand();

		const std::vector<ci::Vec2i>&			getFingerTipPositions() const;
		JointName								getJointName() const;
		const ci::Vec2i&						getPalmPosition() const;
		size_t									getUserId() const;

		
	protected:
		Hand( const std::vector<ci::Vec2i>& fingerTipPositions, const ci::Vec2i& palmPosition, 
			JointName jointName, size_t userId );

		std::vector<ci::Vec2i>					mFingerTipPositions;
		JointName								mJointName;
		ci::Vec2i								mPalmPosition;
		size_t									mUserId;

		friend class							HandTracker;
	};

	//////////////////////////////////////////////////////////////////////////////////////////////

	static HandTrackerRef						create();
	~HandTracker();

	virtual void								start( const DeviceOptions& deviceOptions = DeviceOptions() );
	virtual void								stop();
	virtual void								update( const ci::Channel16u& depth, const std::vector<Skeleton>& skeletons );

	template<typename T, typename Y> 
	inline void									connectEventHander( T eventHandler, Y* obj )
	{
		connectEventHander( std::bind( eventHandler, obj, std::placeholders::_1 ) );
	}

	void										connectEventHander( const std::function<void( std::vector<Hand> )>& eventHandler );
protected:
	typedef std::shared_ptr<std::thread>		ThreadRef;

	HandTracker();
	
	ImageResolution								mImageResolution;
	ci::Vec2i									mImageSize;

	ci::Channel16u								mDepthChannel;
	std::vector<Hand>							mHands;
	std::vector<Skeleton>						mSkeletons;
	
	std::function<void( std::vector<Hand> )>	mEventHandler;
	volatile bool								mNewHands;
	volatile bool								mRunning;
	ThreadRef									mThread;
	virtual void								run();
};

}
 