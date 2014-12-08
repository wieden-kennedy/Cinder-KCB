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
#include "cinder/app/RendererGl.h"
#include "cinder/Camera.h"
#include "cinder/gl/Context.h"
#include "cinder/gl/Fbo.h"
#include "cinder/gl/GlslProg.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/VboMesh.h"
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

	ci::gl::FboRef				mFbo[ 2 ];
	ci::gl::GlslProgRef			mGlslProgDraw;
	ci::gl::GlslProgRef			mGlslProgGpGpu;
	ci::gl::VboMeshRef			mMesh;
	ci::gl::TextureRef			mTextureDepth;
	ci::gl::TextureRef			mTexturePosition[ 2 ];
	ci::gl::TextureRef			mTextureVelocity[ 2 ];
	
	ci::CameraPersp				mCamera;
	ci::Vec3f					mEyePoint;
	ci::Vec3f					mLookAt;
	float						mPointCloudDepth;

	float						mParticleDampen;
	ci::Vec3f					mParticleCenter;
	float						mParticleSpeed;
	float						mParticleTrails;
	
	bool						mDrawParams;
	bool						mDrawTextures;
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
			gl::context()->pushFramebuffer( mFbo[ pong ] );
			const GLenum buffers[ 2 ] = {
				GL_COLOR_ATTACHMENT0, 
				GL_COLOR_ATTACHMENT1
			};
			glDrawBuffers( 2, buffers );
			gl::viewport( mFbo[ pong ]->getSize() );
			gl::setMatricesWindow( mFbo[ pong ]->getSize(), false );
			mTextureDepth->bind( 0 );
			mTexturePosition[ ping ]->bind( 1 );
			mTextureVelocity[ ping ]->bind( 2 );
			
			gl::context()->pushGlslProg( mGlslProgGpGpu );
			gl::context()->setDefaultShaderVars();
			mGlslProgGpGpu->uniform( "uCenter",				mParticleCenter );
			mGlslProgGpGpu->uniform( "uDampen",				mParticleDampen );
			mGlslProgGpGpu->uniform( "uSpeed",				mParticleSpeed );
			mGlslProgGpGpu->uniform( "uTextureKinect",		0 );
			mGlslProgGpGpu->uniform( "uTexturePosition",	1 );
			mGlslProgGpGpu->uniform( "uTextureVelocity",	2 );
		
			gl::drawSolidRect( mFbo[ pong ]->getBounds() );
			
			gl::context()->popGlslProg();
			mTextureVelocity[ ping ]->unbind();
			mTexturePosition[ ping ]->unbind();
			mTextureDepth->unbind();
			gl::context()->popFramebuffer();
		}

		////////////////////////////////////////////////////////////////
		// Draw particles

		gl::viewport( getWindowSize() );
		{
			gl::setMatricesWindow( getWindowSize() );
			gl::enableAlphaBlending();
			gl::disable( GL_TEXTURE_2D );
			gl::color( ColorAf( Colorf::black(), 1.0f - mParticleTrails ) );
			gl::drawSolidRect( getWindowBounds() );
			gl::disableAlphaBlending();
			
			gl::setMatrices( mCamera );
			gl::enableAdditiveBlending();
			gl::enable( GL_TEXTURE_2D );
			gl::color( ColorAf::white() );

			mTexturePosition[ pong ]->bind();

			gl::context()->pushGlslProg( mGlslProgDraw );
			gl::context()->setDefaultShaderVars();
			mGlslProgDraw->uniform( "uDepth",			mPointCloudDepth );
			mGlslProgDraw->uniform( "uTexturePosition",	0 );
			mGlslProgDraw->uniform( "uTime",			(float)getElapsedSeconds() );

			gl::draw( mMesh );

			gl::context()->popGlslProg();
			mTexturePosition[ pong ]->unbind();
			gl::disableAlphaBlending();
		}

		////////////////////////////////////////////////////////////////
		// Draw params, debug info

		if ( mDrawTextures ) {
			gl::setMatricesWindow( getWindowSize() );
			gl::color( ColorAf::white() );
			
			float x = (float)( getWindowWidth() - mTextureDepth->getWidth() / 2 );
			float y = 0.0f;

			gl::pushMatrices();
			gl::translate( x, y );
			gl::draw( mTextureDepth, mTextureDepth->getBounds(), Rectf( mTextureDepth->getBounds() ) * 0.5f );
			gl::popMatrices();
			y += (float)( mTextureDepth->getHeight() / 2 );

			gl::pushMatrices();
			gl::translate( x, y );
			gl::draw( mTexturePosition[ 0 ], mTexturePosition[ 0 ]->getBounds(), Rectf( mTexturePosition[ 0 ]->getBounds() ) * 0.5f );
			gl::popMatrices();
			y += (float)( mTexturePosition[ 0 ]->getHeight() / 2 );

			gl::pushMatrices();
			gl::translate( x, y );
			gl::draw( mTextureVelocity[ 0 ], mTextureVelocity[ 0 ]->getBounds(), Rectf( mTextureVelocity[ 0 ]->getBounds() ) * 0.5f );
			gl::popMatrices();
		}
	}
	
	if ( mDrawParams ) {
		mParams->draw();
	}
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
	settings->setWindowSize( 1280, 720 );
}

void ParticleApp::resize()
{
	glPointSize( 0.25f );
	mCamera = CameraPersp( getWindowWidth(), getWindowHeight(), 60.0f, 1.0f, 50000.0f );
	mCamera.setWorldUp( -Vec3f::yAxis() );
}

void ParticleApp::setup()
{
	////////////////////////////////////////////////////////////////
	// Load shaders

	try {
		mGlslProgDraw = gl::GlslProg::create( loadResource( RES_GLSL_DRAW_VERT ), loadResource( RES_GLSL_DRAW_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "Unable to load draw shader: " << ex.what() << endl;
		quit();
		return;
	}

	try {
		mGlslProgGpGpu = gl::GlslProg::create( loadResource( RES_GLSL_GPGPU_VERT ), loadResource( RES_GLSL_GPGPU_FRAG ) );
	} catch ( gl::GlslProgCompileExc ex ) {
		console() << "Unable to load GPGPU shader: " << ex.what() << endl;
		quit();
		return;
	}

	////////////////////////////////////////////////////////////////
	// Define properties

	mDrawParams			= true;
	mDrawTextures		= false;
	mEyePoint			= Vec3f( 0.0f, 0.0f, 3000.0f );
	mFrameRate			= 0.0f;
	mFullScreen			= isFullScreen();
	mFullScreenPrev		= mFullScreen;
	mLookAt				= Vec3f( 0.0f, -500.0f, 0.0f );
	mParticleDampen		= 0.96f;
	mParticleSpeed		= 0.05f;
	mParticleTrails		= 0.92f;
	mPointCloudDepth	= 3000.0f;

	////////////////////////////////////////////////////////////////
	// Kinect

	MsKinect::DeviceOptions deviceOptions;
	deviceOptions.enableColor( false );
	deviceOptions.enableNearMode();
	deviceOptions.setDepthResolution( MsKinect::ImageResolution::NUI_IMAGE_RESOLUTION_640x480 );
	mDevice = MsKinect::Device::create();
	mDevice->connectEventHandler( &ParticleApp::onFrame, this );
	try {
		mDevice->start( deviceOptions );
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

	////////////////////////////////////////////////////////////////
	// Point cloud vertex buffer

	int32_t h = deviceOptions.getDepthSize().y;
	int32_t w = deviceOptions.getDepthSize().x;

	glPointSize( 1.0f );
	
	struct Vertex
	{
		Vec3f position;
		Vec2f texCoord;
	};
	vector<Vertex> vertices;
	vector<uint32_t> indices;
	for ( int32_t x = 0; x < w; ++x ) {
		for ( int32_t y = 0; y < h; ++y ) {
			indices.push_back( (uint32_t)( x * h + y ) );
			Vertex vertex;
			vertex.texCoord = Vec2f( (float)x / (float)( w - 1 ), (float)y / (float)( h - 1 ) );
			vertex.position = Vec3f(
				( vertex.texCoord.x * 2.0f - 1.0f ) * (float)h, 
				( vertex.texCoord.y * 2.0f - 1.0f ) * (float)w, 
				0.0f );
			vertices.push_back( vertex );
		}
	}

	gl::VboRef vboIndices	= gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, sizeof( uint32_t )	* indices.size(),	&indices[ 0 ] );
	gl::VboRef vboVertices	= gl::Vbo::create( GL_ARRAY_BUFFER,			sizeof( Vertex )	* vertices.size(),	&vertices[ 0 ] );
	geom::BufferLayout layout;
	layout.append( geom::Attrib::POSITION,		3, sizeof( Vertex ), 0 );
	layout.append( geom::Attrib::TEX_COORD_0,	2, sizeof( Vertex ), sizeof( Vec3f ) );
	vector<pair<geom::BufferLayout, gl::VboRef> > buffer;
	buffer.push_back( make_pair( layout, vboVertices ) );
	mMesh = gl::VboMesh::create( vertices.size(), GL_POINTS, buffer, indices.size(), GL_UNSIGNED_BYTE, vboIndices );

	////////////////////////////////////////////////////////////////
	// Position and velocity buffer

	gl::Texture2d::Format textureFormat;
	textureFormat.setInternalFormat( GL_RGBA32F );
	textureFormat.setMagFilter( GL_NEAREST );
	textureFormat.setMinFilter( GL_NEAREST );
	textureFormat.setWrap( GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE );
	textureFormat.setPixelDataFormat( GL_RGBA );
	textureFormat.setPixelDataType( GL_FLOAT );

	for ( size_t i = 0; i < 2; ++i ) {
		mTexturePosition[ i ] = gl::Texture2d::create( w, h, textureFormat );
		mTextureVelocity[ i ] = gl::Texture2d::create( w, h, textureFormat );

		gl::RenderbufferRef positionBuffer = gl::Renderbuffer::create( w, h, 0, 0 );
		gl::RenderbufferRef velocityBuffer = gl::Renderbuffer::create( w, h, 0, 0 );

		gl::Fbo::Format fboFormat;
		fboFormat.attachment( GL_COLOR_ATTACHMENT0, mTexturePosition[ i ], positionBuffer );
		fboFormat.attachment( GL_COLOR_ATTACHMENT1, mTextureVelocity[ i ], velocityBuffer );
		mFbo[ i ] = gl::Fbo::create( w, h, fboFormat );

		gl::ScopedFramebuffer fboScope( mFbo[ i ] );
		gl::viewport( mFbo[ i ]->getSize() );
		gl::clear();
	}

	////////////////////////////////////////////////////////////////
	// Set up parameters

	mParams = params::InterfaceGl::create( "PARAMS", Vec2i( 200, 400 ) );
	mParams->addParam( "Frame rate",		&mFrameRate,			"", true );
	mParams->addParam( "Full screen",		&mFullScreen,			"key=f" );
	mParams->addButton( "Quit",				[ & ]() { quit(); },	"key=q" );
	mParams->addSeparator( "" );
	mParams->addParam( "Eye point",			&mEyePoint );
	mParams->addParam( "Look at",			&mLookAt );
	mParams->addParam( "Point cloud depth",	&mPointCloudDepth,		"min=-10000.0 max=10000.0 step=1.0" );
	mParams->addSeparator( "" );
	mParams->addParam( "Particle dampen",	&mParticleDampen,		"min=0.0 max=1.0 step=0.0001" );
	mParams->addParam( "Particle origin",	&mParticleCenter );
	mParams->addParam( "Particle speed",	&mParticleSpeed,		"min=-10.0 max=10.0 step=0.001" );
	mParams->addParam( "Particle trails",	&mParticleTrails,		"min=0.0 max=1.0 step=0.000001" );
	mParams->addSeparator( "" );
	mParams->addParam( "Draw params",		&mDrawParams,			"key=p" );
	mParams->addParam( "Draw textures",		&mDrawTextures,			"key=t" );
	
	resize();
}

void ParticleApp::update()
{
	mFrameRate = getAverageFps();
	mCamera.lookAt( mEyePoint, mLookAt );

	if ( mFullScreenPrev != mFullScreen ) {
		setFullScreen( mFullScreen );
		mFullScreenPrev = mFullScreen;
	}
}

CINDER_APP_BASIC( ParticleApp, RendererGl( RendererGl::Options().antiAliasing( RendererGl::AA_NONE ).coreProfile().version( 3, 3 ) ) )
 