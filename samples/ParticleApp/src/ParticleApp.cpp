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
#include "cinder/Camera.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"
#include "cinder/params/Params.h"
#include "Kinect.h"

class ParticleApp : public ci::app::AppBasic 
{
public:
	void 						draw();
	void 						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						resize();			
	void 						setup();
	void						update();
private:
	MsKinect::DeviceRef			mDevice;
	void						onFrame( MsKinect::Frame frame );
	
	ci::CameraPersp				mCamera;
	ci::gl::Fbo					mFbo[ 2 ];
	ci::gl::GlslProgRef			mShaderDraw;
	ci::gl::GlslProgRef			mShaderGpGpu;
	ci::gl::VboMeshRef			mMesh;
	ci::gl::TextureRef			mTextureDepth;

	float						mParticleDampen;
	ci::Vec3f					mParticleCenter;
	float						mParticleSpeed;
	float						mParticleTrails;
	
	float						mFrameRate;
	bool						mFullScreen;
	bool						mFullScreenPrev;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/ImageIo.h"
#include "cinder/Rand.h"
#include "cinder/Utilities.h"
#include "Resources.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void ParticleApp::draw()
{
	if ( mTextureDepth ) {
		gl::enable( GL_TEXTURE_2D );
		
		////////////////////////////////////////////////////////////////
		// GPGPU

		int32_t ping = getElapsedFrames() % 2;
		int32_t pong = ( ping + 1 ) % 2;
		{
			mFbo[ pong ].bindFramebuffer();
			const GLenum buffers[ 2 ] = {
				GL_COLOR_ATTACHMENT0, 
				GL_COLOR_ATTACHMENT1
			};
			glDrawBuffers( 2, buffers );
			gl::setViewport( mFbo[ pong ].getBounds() );
			gl::setMatricesWindow( mFbo[ pong ].getSize(), false );
			for ( int32_t i = 0; i < 2; ++i ) {
				mFbo[ ping ].bindTexture( i, i );
			}
			mTextureDepth->bind( 2 );
			
			mShaderGpGpu->bind();
			mShaderGpGpu->uniform( "center",		mParticleCenter );
			mShaderGpGpu->uniform( "dampen",		mParticleDampen );
			mShaderGpGpu->uniform( "kinect",		2 );
			mShaderGpGpu->uniform( "positions",		0 );
			mShaderGpGpu->uniform( "speed",			mParticleSpeed );
			mShaderGpGpu->uniform( "velocities",	1 );
		
			gl::drawSolidRect( mFbo[ pong ].getBounds(), false );
			
			mShaderGpGpu->unbind();
			mTextureDepth->unbind();
			mFbo[ ping ].unbindTexture();
			mFbo[ pong ].unbindFramebuffer();
		}

		////////////////////////////////////////////////////////////////
		// Draw particles

		gl::setViewport( getWindowBounds() );
		gl::clear();
		{
			gl::setMatrices( mCamera );

			gl::enableAlphaBlending();
			gl::disable( GL_TEXTURE_2D );
			gl::color( ColorAf( Colorf::black(), 1.0f - mParticleTrails ) );
			gl::drawSolidRect( getWindowBounds() );
			gl::enable( GL_TEXTURE_2D );

			gl::enableAdditiveBlending();
			gl::enableDepthRead();
			gl::enableDepthWrite();
			gl::color( ColorAf::white() );

			mFbo[ pong ].bindTexture();

			mShaderDraw->bind();
			mShaderDraw->uniform( "positions", 0 );

			gl::draw( mMesh );

			mShaderDraw->unbind();
			mFbo[ pong ].unbindTexture();
			
			gl::disableDepthRead();
			gl::disableDepthWrite();
			gl::disableAlphaBlending();
		}

		////////////////////////////////////////////////////////////////
		// Draw params, debug info

		{
			gl::setMatricesWindow( getWindowSize() );
			gl::color( ColorAf::white() );
			
			float x = (float)( getWindowWidth() - mTextureDepth->getWidth() / 2 );
			float y = 0.0f;

			gl::pushMatrices();
			gl::translate( x, y );
			gl::draw( mTextureDepth, mTextureDepth->getBounds(), Rectf( mTextureDepth->getBounds() ) * 0.5f );
			gl::popMatrices();
			y += (float)( mTextureDepth->getHeight() / 2 );

			for ( int32_t i = 0; i < 2; ++i ) {
				gl::pushMatrices();
				gl::translate( x, y );
				const gl::Texture& tex = mFbo[ 0 ].getTexture( i );
				gl::draw( tex, tex.getBounds(), Rectf( tex.getBounds() ) * 0.5f );
				gl::popMatrices();
				y += (float)( tex.getHeight() / 2 );
			}
		}
	}
	
	mParams->draw();
}

void ParticleApp::onFrame( MsKinect::Frame frame )
{
	if ( frame.getDepthChannel() ) {
		Surface16u depth	= MsKinect::depthChannelToSurface( frame.getDepthChannel(), MsKinect::DepthProcessOptions().enableRemoveBackground() );
		mTextureDepth		= gl::Texture::create( depth );
	}
}

void ParticleApp::prepareSettings( Settings* settings )
{
	settings->setFrameRate( 60.0f );
	settings->setFullScreen( true );
	settings->setWindowSize( 1280, 720 );
}

void ParticleApp::resize()
{
	gl::enable( GL_POINT_SMOOTH );
	glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );

	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 60.0f, 1.0f, 5000.0f );
	mCamera.lookAt( Vec3f( 300.0f, 300.0f, 1200.0f ), Vec3f::zero() );
	mCamera.setWorldUp( -Vec3f::yAxis() );
}

void ParticleApp::setup()
{
	////////////////////////////////////////////////////////////////
	// Load shaders

	try {
		mShaderDraw = gl::GlslProg::create( loadResource( RES_GLSL_DRAW_VERT ), loadResource( RES_GLSL_DRAW_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "Unable to load draw shader: " << ex.what() << endl;
		quit();
		return;
	}

	try {
		mShaderGpGpu = gl::GlslProg::create( loadResource( RES_GLSL_GPGPU_VERT ), loadResource( RES_GLSL_GPGPU_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "Unable to load GPGPU shader: " << ex.what() << endl;
		quit();
		return;
	}

	////////////////////////////////////////////////////////////////
	// Define properties

	mFrameRate			= 0.0f;
	mFullScreen			= isFullScreen();
	mFullScreenPrev		= mFullScreen;
	mParticleDampen		= 0.97f;
	mParticleSpeed		= 0.033f;
	mParticleTrails		= 0.9f;

	////////////////////////////////////////////////////////////////
	// Set up Kinect

	MsKinect::DeviceOptions deviceOptions;
	deviceOptions.enableColor( false );
	deviceOptions.enableNearMode();
	deviceOptions.enableSeatedMode();
	deviceOptions.enableUserTracking( false );
	deviceOptions.setDepthResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( &ParticleApp::onFrame, this );
	mDevice->start( deviceOptions );

	////////////////////////////////////////////////////////////////
	// Set up particles

	glPointSize( 1.0f );

	gl::Fbo::Format format;
	format.enableColorBuffer( true, 4 );
	format.setColorInternalFormat( GL_RGBA32F );

	for ( size_t i = 0; i < 2; ++i ) {
		mFbo[ i ] = gl::Fbo( deviceOptions.getDepthSize().x, deviceOptions.getDepthSize().y, format );
		mFbo[ i ].bindFramebuffer();
		const GLenum buffers[ 2 ] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
		glDrawBuffers( 2, buffers );
		gl::setViewport( mFbo[ i ].getBounds() );
		gl::setMatricesWindow( mFbo[ i ].getSize() );
		gl::clear();
		mFbo[ i ].unbindFramebuffer();
	}

	vector<uint32_t> indices;
	vector<Vec3f> positions;
	vector<Vec2f> texCoords;
	int32_t h	= mFbo[ 0 ].getHeight();
	int32_t w	= mFbo[ 0 ].getWidth();
	for ( int32_t x = 0; x < w; ++x ) {
		for ( int32_t y = 0; y < h; ++y ) {
			indices.push_back( (uint32_t)( x * h + y ) );
			texCoords.push_back( Vec2f( (float)x / (float)( w - 1 ), (float)y / (float)( h - 1 ) ) );
			positions.push_back( Vec3f(
				( texCoords.rbegin()->x * 2.0f - 1.0f ) * (float)h, 
				( texCoords.rbegin()->y * 2.0f - 1.0f ) * (float)w, 
				0.0f ) );
		}
	}
	gl::VboMesh::Layout layout;
	layout.setStaticIndices();
	layout.setStaticPositions();
	layout.setStaticTexCoords2d();

	mMesh = gl::VboMesh::create( positions.size(), indices.size(), layout, GL_POINTS );
	mMesh->bufferIndices( indices );
	mMesh->bufferPositions( positions );
	mMesh->bufferTexCoords2d( 0, texCoords );
	mMesh->unbindBuffers();

	////////////////////////////////////////////////////////////////
	// Set up parameters

	mParams = params::InterfaceGl::create( "PARAMS", Vec2i( 200, 400 ) );
	mParams->addParam( "Frame rate",		&mFrameRate,						"", true );
	mParams->addParam( "Full screen",		&mFullScreen,						"key=f" );
	mParams->addButton( "Quit",				bind( &ParticleApp::quit, this ),	"key=q" );
	mParams->addSeparator( "" );
	mParams->addParam( "Particle dampen",	&mParticleDampen,					"min=0.0 max=1.0 step=0.0001" );
	mParams->addParam( "Particle origin",	&mParticleCenter );
	mParams->addParam( "Particle speed",	&mParticleSpeed,					"min=-10.0 max=10.0 step=0.001" );
	mParams->addParam( "Particle trails",	&mParticleTrails,					"min=0.0 max=1.0 step=0.000001" );

	resize();
}

void ParticleApp::update()
{
	mFrameRate = getAverageFps();

	if ( mFullScreenPrev != mFullScreen ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}
}

CINDER_APP_BASIC( ParticleApp, RendererGl )
