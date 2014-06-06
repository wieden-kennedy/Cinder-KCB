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
#include "cinder/params/Params.h"
#include "Kinect.h"

class UserApp : public ci::app::AppBasic 
{
public:
	void 						draw();	
	void						keyDown( ci::app::KeyEvent event );
	void 						setup();
private:
	MsKinect::DeviceRef			mDevice;
	MsKinect::Frame				mFrame;
	
	MsKinect::Face				mFace;
	MsKinect::FaceTrackerRef	mFaceTracker;
};

using namespace ci;
using namespace ci::app;
using namespace std;

void UserApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear();
	gl::setMatricesWindow( getWindowSize() );
	gl::enableAlphaBlending();
	gl::color( ColorAf::white() );
	
	if ( mFrame.getDepthChannel() ) {
		gl::enable( GL_TEXTURE_2D );
		gl::TextureRef tex = gl::Texture::create( mFrame.getDepthChannel() );
		gl::draw( tex, tex->getBounds(), getWindowBounds() );
		gl::disable( GL_TEXTURE_2D );

		gl::pushMatrices();
		gl::scale( Vec2f( getWindowSize() ) / Vec2f( tex->getSize() ) );

		for ( const auto& skeleton : mFrame.getSkeletons() ) {
			for ( const auto& joint : skeleton ) {
				const MsKinect::Bone& bone = joint.second;

				Vec2i v0 = mDevice->mapSkeletonCoordToDepth( bone.getPosition() );
				Vec2i v1 = mDevice->mapSkeletonCoordToDepth( skeleton.at( bone.getStartJoint() ).getPosition() );
				gl::drawLine( v0, v1 );
				gl::drawSolidCircle( v0, 5.0f, 16 );
			}
		}

		if ( mFace.getMesh2d().getNumVertices() > 0 ) {
			gl::pushMatrices();
			gl::scale( 0.5f, 0.5f );
			gl::color( ColorAf::white() );
			gl::enableWireframe();
			gl::draw( mFace.getMesh2d() );
			gl::disableWireframe();
			gl::popMatrices();
		}

		gl::popMatrices();
	}
}

void UserApp::keyDown( KeyEvent event )
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

void UserApp::setup()
{
	setFrameRate( 60.0f );
	glLineWidth( 2.0f );

	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( [ & ]( MsKinect::Frame frame )
	{
		mFrame  = frame;
		mFace	= frame.getFace();
	} );
	try {
		mDevice->start( MsKinect::DeviceOptions().enableFaceTracking() );
		mDevice->getFaceTracker()->enableCalcMesh( false );
		mDevice->getFaceTracker()->enableCalcMesh2d();
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
}

CINDER_APP_BASIC( UserApp, RendererGl )
