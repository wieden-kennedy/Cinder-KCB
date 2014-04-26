#version 330 core

uniform sampler2D	uTextureKinect;
uniform sampler2D	uTexturePosition;
uniform sampler2D	uTextureVelocity;

uniform float		uDampen;
uniform vec3		uCenter;
uniform float		uSpeed;

in vec2				vUv;

out vec4			oPosition;
out vec4			oVelocity;

void main ( void )
{
	vec3 position		= texture( uTexturePosition, vUv ).rgb;
	vec3 velocity		= texture( uTextureVelocity, vUv ).rgb;

	vec3 destination	= vec3( vUv.s, vUv.t, 1.0 - texture( uTextureKinect, vUv ).b );
	float depth			= destination.z;

	velocity			+= normalize( destination - position ) * uSpeed * depth;
	
	if ( destination.z != position.z ) {
		position		+= velocity;
	}
	position			= ( position - uCenter ) * uDampen * 0.5;
	velocity			*= uDampen;

	oPosition			= vec4( position, 1.0 );
	oVelocity			= vec4( velocity, 1.0 );
}
 