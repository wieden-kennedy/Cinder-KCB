/*
* 
* Copyright (c) 2013, Wieden+Kennedy
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
#include "HandFinder.h"
#include "Kinect.h"

class HandApp : public ci::app::AppBasic 
{
public:
	void 						draw();	
	void						keyDown( ci::app::KeyEvent event );
	void 						setup();
private:
	MsKinect::DeviceRef			mDevice;
	ci::Channel16u				mDepthChannel;
	ci::gl::TextureRef			mDepthTexture;
	std::vector<MsKinect::Hand>	mHands;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void HandApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	
	if ( mDepthChannel ) {
		gl::color( ColorAf::white() );
		gl::enable( GL_TEXTURE_2D );
		gl::draw( mDepthTexture, mDepthTexture->getBounds(), getWindowBounds() );
		gl::disable( GL_TEXTURE_2D );

		gl::pushMatrices();
		gl::scale( Vec2f( getWindowSize() ) / Vec2f( mDepthTexture->getSize() ) );
		for ( auto hand : mHands ) {
			for ( auto finger : hand.getFingerTipPositions() ) {
				gl::drawStrokedCircle( finger, 5.0f, 24 );
			}
		}
		gl::popMatrices();
	}
}

void HandApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	case KeyEvent::KEY_q:
		quit();
		break;
	}
}

void HandApp::setup()
{
	setFrameRate( 60.0f );
	//setFullScreen( true );

	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( [ & ]( MsKinect::Frame frame )
	{
		if ( frame.getDepthChannel() ) {
			mDepthChannel	= frame.getDepthChannel();
			mDepthTexture	= gl::Texture::create( mDepthChannel );
			mHands			= MsKinect::findHands( mDepthChannel, frame.getSkeletons(), mDevice->getDeviceOptions() );
			for ( auto hand : mHands ) {
				console() << hand.getFingerTipPositions().size() << endl;
			}
		}
	} );
	mDevice->start();
}

CINDER_APP_BASIC( HandApp, RendererGl )
