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

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "FaceTracker.h"
#include "Kinect.h"

class TestApp : public ci::app::AppBasic 
{
public:
	void 						draw();
	void 						prepareSettings( ci::app::AppBasic::Settings* settings );
	void 						setup();
	void						update();
private:
	MsKinect::DeviceRef			mDevice;
	MsKinect::Frame				mFrame;
	
	MsKinect::FaceTracker::Face	mFace;
	MsKinect::FaceTrackerRef	mFaceTracker;
	
	float						mFrameRate;
	bool						mFullScreen;
	bool						mFullScreenPrev;
	int32_t						mTilt;
	int32_t						mTiltPrev;
	int32_t						mNumUsers;
	ci::params::InterfaceGlRef	mParams;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void TestApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	gl::enableAlphaBlending();
	
	gl::enable( GL_TEXTURE_2D );
	gl::color( ColorAf::white() );
	
	if ( mFrame.getColorSurface() ) {		
		gl::draw( gl::Texture::create( mFrame.getColorSurface() ) );
	}

	if ( mFrame.getInfraredChannel() ) {
		gl::pushMatrices();
		gl::TextureRef tex = gl::Texture::create( mFrame.getInfraredChannel() );
		gl::translate( (float)( getWindowWidth() - tex->getWidth() / 2 ), 0.0f );
		gl::draw( tex, tex->getBounds(), Rectf( tex->getBounds() ) * 0.5f );
		gl::popMatrices();
	}

	if ( mFrame.getDepthChannel() ) {
		gl::pushMatrices();
		gl::TextureRef texDepth = gl::Texture::create( mFrame.getDepthChannel() );
		gl::translate( (float)( getWindowWidth() - texDepth->getWidth() / 2 ), (float)( texDepth->getHeight() / 2 ) );
		gl::draw( texDepth, texDepth->getBounds(), Rectf( texDepth->getBounds() ) * 0.5f );
		
		Surface16u surface = MsKinect::depthChannelToSurface( mFrame.getDepthChannel(), 
			MsKinect::DepthProcessOptions().enableUserColor().enableRemoveBackground() );
		
		gl::TextureRef texUsers = gl::Texture::create( surface );
		gl::pushMatrices();
		gl::translate( 0.0f, (float)( texUsers->getHeight() / 2 ) );
		gl::draw( texUsers, texUsers->getBounds(), Rectf( texUsers->getBounds() ) * 0.5f );
		gl::popMatrices();
		gl::popMatrices();
	}

	gl::disable( GL_TEXTURE_2D );

	uint32_t id = 1;
	for ( const auto& skeleton : mFrame.getSkeletons() ) {
		gl::color( MsKinect::getUserColor( id ) );
		for ( const auto& joint : skeleton ) {
			const MsKinect::Bone& bone = joint.second;

			Vec2i v0 = MsKinect::mapSkeletonCoordToColor( 
				bone.getPosition(), 
				mFrame.getDepthChannel(), 
				mDevice->getDeviceOptions().getColorResolution(), 
				mDevice->getDeviceOptions().getDepthResolution() 
				);
			Vec2i v1 = MsKinect::mapSkeletonCoordToColor( 
				skeleton.at( bone.getStartJoint() ).getPosition(), 
				mFrame.getDepthChannel(), 
				mDevice->getDeviceOptions().getColorResolution(), 
				mDevice->getDeviceOptions().getDepthResolution() 
				);
			gl::drawLine( v0, v1 );
			gl::drawSolidCircle( v0, 5.0f, 16 );
		}
		++id;
	}

	if ( mFace.getMesh2d().getNumVertices() > 0 ) {
		gl::color( ColorAf::white() );
		gl::enableWireframe();
		gl::draw( mFace.getMesh2d() );
		gl::disableWireframe();
	}

	mParams->draw();
}

void TestApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 640, 480 );
	settings->setFrameRate( 60.0f );
}

void TestApp::setup()
{
	mFrameRate		= 0.0f;
	mFullScreen		= isFullScreen();
	mFullScreenPrev	= mFullScreen;
	mNumUsers		= 0;

	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( [ & ]( MsKinect::Frame frame )
	{
		mFrame = frame;
		if ( mFaceTracker ) {
			mFaceTracker->update( mFrame.getColorSurface(), mFrame.getDepthChannel() );
		}
		mNumUsers = (int32_t)MsKinect::calcNumUsersFromDepth( mFrame.getDepthChannel() );
	} );
	mDevice->start();

	mTilt		= mDevice->getTilt();
	mTiltPrev	= mTilt;

	mFaceTracker = MsKinect::FaceTracker::create();
	mFaceTracker->enableCalcMesh( false );
	mFaceTracker->enableCalcMesh2d();
	mFaceTracker->connectEventHander( [ & ]( MsKinect::FaceTracker::Face face ) {
		mFace = face;
	} );
	mFaceTracker->start();

	mParams = params::InterfaceGl::create( "PARAMS", Vec2i( 200, 140 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,					"", true );
	mParams->addParam( "User count",	&mNumUsers,						"", true );
	mParams->addParam( "Tilt",			&mTilt,							"min=-28 max=28 step=1" );
	mParams->addParam( "Full screen",	&mFullScreen,					"key=f" );
	mParams->addButton( "Quit",			bind( &TestApp::quit, this ),	"key=q" );
}

void TestApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreenPrev != mFullScreen ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}

	if ( mTiltPrev = mTilt ) {
		mDevice->setTilt( mTilt );
		mTiltPrev = mTilt;
	}
}

CINDER_APP_BASIC( TestApp, RendererGl )
