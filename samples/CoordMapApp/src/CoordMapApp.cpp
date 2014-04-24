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

#include <algorithm>
#include "boost/algorithm/string.hpp"
#include "cinder/app/AppBasic.h"
#include "cinder/Camera.h"
#include "cinder/gl/gl.h"
#include "cinder/gl/Texture.h"
#include "cinder/ImageIo.h"
#include "cinder/params/Params.h"
#include "cinder/Vector.h"
#include "cinder/Utilities.h"
#include "Kinect.h"

/*
* This application demonstrates coordinate mapping 
* between color and depth images.
*/
class CoordMapApp : public ci::app::AppBasic 
{
public:
	void								draw();
	void								prepareSettings( ci::app::AppBasic::Settings* settings );
	void								setup();
	void								shutdown();
	void								update();
private:
	MsKinect::DeviceRef					mDevice;
	ci::Surface8u						mSurface;
	void								onFrame( MsKinect::Frame frame );

	float								mFrameRate;
	bool								mFullScreen;
	ci::params::InterfaceGlRef			mParams;
	void								screenShot();
};

using namespace ci;
using namespace ci::app;
using namespace std;

void CoordMapApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::gray( 0.1f ) );
	gl::setMatricesWindow( getWindowSize(), true );

	if ( mDevice->isCapturing() && mSurface ) {
		gl::color( Colorf::white() );
		if ( mSurface ) {
			gl::draw( gl::Texture( mSurface ), mSurface.getBounds(), getWindowBounds() );
		}
	}

	mParams->draw();
}

void CoordMapApp::onFrame( MsKinect::Frame frame )
{
	mSurface					= frame.getColorSurface();
	Channel16u::ConstIter iter	= frame.getDepthChannel().getIter();
	
	while ( iter.line() ) {
		while ( iter.pixel() ) {
			Vec2i pos	= MsKinect::mapDepthCoordToColor( iter.getPos(), frame.getDepthChannel(), mDevice );
			const uint16_t& v = frame.getDepthChannel().getValue( pos );
			mSurface.setPixel( pos, Color8u::black() );
		}
	}
}

void CoordMapApp::prepareSettings( Settings *settings )
{
	settings->setWindowSize( 640, 480 );
	settings->setFrameRate( 60.0f );
}

void CoordMapApp::screenShot()
{
	writeImage( getAppPath() / fs::path( "frame" + toString( getElapsedFrames() ) + ".png" ), copyWindowSurface() );
}

void CoordMapApp::setup()
{
	mFrameRate	= 0.0f;
	mFullScreen	= isFullScreen();
	
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( &CoordMapApp::onFrame, this );
	MsKinect::DeviceOptions options;
	options.enableColor();
	options.enableDepth();
	options.setColorResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	options.setDepthResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	try {
		mDevice->start( options );
	} catch ( MsKinect::Device::ExcDeviceCreate ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcDeviceInit ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcDeviceInvalid ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcGetCoordinateMapper ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcOpenStreamColor ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcOpenStreamDepth ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcStreamStart ex ) {
		console() << ex.what() << endl;
	} catch ( MsKinect::Device::ExcUserTrackingEnable ex ) {
		console() << ex.what() << endl;
	}
	
	mParams = params::InterfaceGl::create( "Parameters", Vec2i( 220, 100 ) );
	mParams->addParam( "App frame rate",	&mFrameRate,							"", true	);
	mParams->addParam( "Full screen",		&mFullScreen,							"key=f"		);
	mParams->addButton( "Screen shot",		bind( &CoordMapApp::screenShot, this ),	"key=s"		);
	mParams->addButton( "Quit",				bind( &CoordMapApp::quit, this ),		"key=q"		);
}

void CoordMapApp::shutdown()
{
	mDevice->stop();
}

void CoordMapApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
	}

	if ( !mDevice->isCapturing() && getElapsedFrames() % 90 == 0 ) {
		mDevice->start();
	}
}

CINDER_APP_BASIC( CoordMapApp, RendererGl )
