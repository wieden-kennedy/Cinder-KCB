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
	ci::gl::Fbo					mFboGpGpu;
	ci::gl::GlslProgRef			mShaderDraw;
	ci::gl::GlslProgRef			mShaderGpGpu;
	ci::gl::VboMeshRef			mMesh;
	ci::gl::TextureRef			mTextureInfrared;

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
	if ( mTextureInfrared ) {
		
		////////////////////////////////////////////////////////////////
		// GPGPU

		int32_t ping	= getElapsedFrames() % 2;
		int32_t pong	= ( ping + 1 ) % 2;
		ping			*= 2;
		pong			*= 2;
		gl::enable( GL_TEXTURE_2D );
		{
			mFboGpGpu.bindFramebuffer();
			const GLenum buffers[ 2 ] = {
				GL_COLOR_ATTACHMENT0 + pong, 
				GL_COLOR_ATTACHMENT1 + pong
			};
			glDrawBuffers( 2, buffers );
			gl::setViewport( mFboGpGpu.getBounds() );
			gl::setMatricesWindow( mFboGpGpu.getSize(), false );
			for ( int32_t i = 0; i < 2; ++i ) {
				mFboGpGpu.bindTexture( i, i + ping );
			}
			mTextureInfrared->bind( 2 );
			
			mShaderGpGpu->bind();
			mShaderGpGpu->uniform( "center",		mParticleCenter );
			mShaderGpGpu->uniform( "dampen",		mParticleDampen );
			mShaderGpGpu->uniform( "kinect",		2 );
			mShaderGpGpu->uniform( "positions",		0 );
			mShaderGpGpu->uniform( "speed",			mParticleSpeed );
			mShaderGpGpu->uniform( "velocities",	1 );
		
			gl::drawSolidRect( mFboGpGpu.getBounds(), true );
			
			mShaderGpGpu->unbind();
			mTextureInfrared->unbind();
			mFboGpGpu.unbindTexture();
			mFboGpGpu.unbindFramebuffer();
		}
		gl::disable( GL_TEXTURE_2D );

		////////////////////////////////////////////////////////////////
		// Draw particles

		gl::setViewport( getWindowBounds() );
		gl::clear();
		{
			gl::setMatrices( mCamera );

			gl::enableAlphaBlending();
			gl::color( ColorAf( Colorf::black(), 1.0f - mParticleTrails ) );
			gl::drawSolidRect( getWindowBounds() );

			gl::enableAdditiveBlending();
			gl::enableDepthRead();
			gl::enableDepthWrite();
			gl::color( ColorAf::white() );

			mFboGpGpu.bindTexture();

			mShaderDraw->bind();
			mShaderDraw->uniform( "positions", 0 );

			gl::draw( mMesh );

			mShaderDraw->unbind();
			mFboGpGpu.unbindTexture();
		}

		////////////////////////////////////////////////////////////////
		// Draw params, debug info

		{
			gl::setMatricesWindow( getWindowSize() );
			gl::disableAlphaBlending();
			gl::enable( GL_TEXTURE_2D );
			gl::color( ColorAf::white() );
			
			float x = (float)( getWindowWidth() - mTextureInfrared->getWidth() / 4 );
			float y = 0.0f;

			gl::pushMatrices();
			gl::translate( x, y );
			gl::draw( mTextureInfrared, mTextureInfrared->getBounds(), Rectf( mTextureInfrared->getBounds() ) * 0.25f );
			gl::popMatrices();
			y += (float)( mTextureInfrared->getHeight() / 4 );

			for ( int32_t i = 0; i < 4; ++i ) {
				gl::pushMatrices();
				gl::translate( x, y );
				gl::draw( mFboGpGpu.getTexture( i ), mFboGpGpu.getTexture( i ).getBounds(), Rectf( mFboGpGpu.getTexture( i ).getBounds() ) * 0.25f );
				gl::popMatrices();
				y += (float)( mFboGpGpu.getTexture( i ).getHeight() / 4 );
			}
			gl::disable( GL_TEXTURE_2D );
		}
	}
	
	mParams->draw();
}

void ParticleApp::onFrame( MsKinect::Frame frame )
{
	if ( frame.getInfraredChannel() ) {
		Channel32f ir( frame.getInfraredChannel() );
		if ( mTextureInfrared ) {
			mTextureInfrared->update( ir );
		} else {
			mTextureInfrared = gl::Texture::create( ir );
		}
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
	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 60.0f, 1.0f, 100.0f );
	mCamera.lookAt( Vec3f( 0.0f, 0.0f, 5.0f ), Vec3f::zero() );
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
	mParticleDampen		= 1.0f;
	mParticleCenter		= Vec3f::zero();
	mParticleSpeed		= 0.021f;
	mParticleTrails		= 0.0f;

	////////////////////////////////////////////////////////////////
	// Set up Kinect

	MsKinect::DeviceOptions deviceOptions;
	deviceOptions.enableInfrared( true );
	deviceOptions.enableUserTracking( false );
	deviceOptions.setInfraredResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( &ParticleApp::onFrame, this );
	mDevice->start( deviceOptions );

	////////////////////////////////////////////////////////////////
	// Set up particles

	glPointSize( 1.0f );

	gl::Fbo::Format format;
	format.enableColorBuffer( true, 4 );
	format.setColorInternalFormat( GL_RGBA32F );

	mFboGpGpu = gl::Fbo( deviceOptions.getInfraredSize().x, deviceOptions.getInfraredSize().y, format );
	mFboGpGpu.bindFramebuffer();
	gl::setViewport( mFboGpGpu.getBounds() );
	gl::setMatricesWindow( mFboGpGpu.getSize() );

	const GLenum positionBuffers[ 2 ] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT0 + 2 };
	glDrawBuffers( 2, positionBuffers );
	gl::clear( Colorf( ColorModel::CM_RGB, mParticleCenter ) );

	const GLenum velocityBuffers[ 2 ] = { GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT1 + 2 };
	glDrawBuffers( 2, velocityBuffers );
	gl::clear();

	mFboGpGpu.unbindFramebuffer();

	vector<uint32_t> indices;
	vector<Vec3f> positions;
	vector<Vec2f> texCoords;
	uint32_t i = 0;
	for ( int32_t x = 0; x < mFboGpGpu.getWidth(); ++x ) {
		for ( int32_t y = 0; y < mFboGpGpu.getHeight(); ++y, ++i ) {
			float u = (float)x / (float)mFboGpGpu.getWidth();
			float v = (float)y / (float)mFboGpGpu.getHeight();
			indices.push_back( i );
			positions.push_back( Vec3f::zero() );
			texCoords.push_back( Vec2f( u, v ) );
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
