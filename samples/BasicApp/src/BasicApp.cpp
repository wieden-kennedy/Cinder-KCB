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

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "Kinect.h"

class BasicApp : public ci::app::AppBasic 
{
public:
	void 				draw();
	void				keyDown( ci::app::KeyEvent event );
	void 				setup();
private:
	MsKinect::DeviceRef	mDevice;
	ci::gl::TextureRef	mTextureColor;
	ci::gl::TextureRef	mTextureDepth;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void BasicApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	
	gl::translate( 0.0f, getWindowCenter().y * 0.5f );
	Rectf bounds = Rectf( getWindowBounds() ) * 0.5f;
	if ( mTextureColor ) {
		gl::draw( mTextureColor, mTextureColor->getBounds(), bounds );
	}
	if ( mTextureDepth ) {
		gl::translate( getWindowCenter().x, 0.0f );
		gl::draw( mTextureDepth, mTextureDepth->getBounds(), bounds );
	}
}

void BasicApp::keyDown( KeyEvent event )
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

void BasicApp::setup()
{
	gl::color( ColorAf::white() );
	gl::enable( GL_TEXTURE_2D );
	
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( [ & ]( MsKinect::Frame frame )
	{
		if ( frame.getColorSurface() ) {
			mTextureColor = gl::Texture::create( frame.getColorSurface() );
		} else if ( frame.getInfraredChannel() ) {
			mTextureColor = gl::Texture::create( frame.getInfraredChannel() );
		}
		if ( frame.getDepthChannel() ) {
			mTextureDepth = gl::Texture::create( frame.getDepthChannel() );
		}
	} );
	MsKinect::DeviceOptions options;
	// Uncomment to read IR stream. Enabling IR disables color stream.
	//options.enableInfrared();
	mDevice->start( options );
}

CINDER_APP_BASIC( BasicApp, RendererGl )
 