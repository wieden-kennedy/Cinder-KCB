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
#include "cinder/Arcball.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

class TestApp : public ci::app::AppBasic 
{
public:
	void 					draw();
	void 					keyDown( ci::app::KeyEvent event );
	void 					prepareSettings( ci::app::AppBasic::Settings* settings );
	void 					shutdown();
	void 					setup();
private:
	MsKinect::DeviceRef		mDevice;
	MsKinect::Frame			mFrame;
};

using namespace ci;
using namespace ci::app;
using namespace MsKinect;
using namespace std;

const Vec2i	kKinectSize( 640, 480 );

void TestApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );

	if ( mFrame.getColorSurface() ) {
		gl::TextureRef tex = gl::Texture::create( mFrame.getColorSurface() );
		gl::draw( tex, tex->getBounds(), Area( 0, ( getWindowHeight() / 4 ) * 1, getWindowWidth() / 2, ( getWindowHeight() / 4 ) * 3 ) );
	}
	if ( mFrame.getDepthChannel() ) {
		gl::TextureRef tex = gl::Texture::create( mFrame.getDepthChannel() );
		gl::draw( tex, tex->getBounds(), Area( getWindowWidth() / 2, ( getWindowHeight() / 4 ) * 1, getWindowWidth(), ( getWindowHeight() / 4 ) * 3 ) );
	}
}

void TestApp::keyDown( KeyEvent event )
{
	switch ( event.getCode() ) {
	case KeyEvent::KEY_q:
		quit();
		break;
	case KeyEvent::KEY_f:
		setFullScreen( !isFullScreen() );
		break;
	}
}

void TestApp::prepareSettings( Settings* settings )
{
	settings->setWindowSize( 1024, 768 );
	settings->setFrameRate( 60.0f );
}

void TestApp::setup()
{
	gl::enable( GL_TEXTURE_2D );
	gl::color( ColorAf::white() );

	mDevice = Device::create();
	mDevice->start( DeviceOptions().enableDepth( true ).enableUserTracking( false ) );
	mDevice->connectEventHandler( [ & ]( Frame frame )
	{
		mFrame = frame;
	} );
}

void TestApp::shutdown()
{
	mDevice->stop();
}

CINDER_APP_BASIC( TestApp, RendererGl )
