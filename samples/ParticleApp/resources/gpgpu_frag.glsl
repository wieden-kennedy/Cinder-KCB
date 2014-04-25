uniform sampler2D	uTextureKinect;
uniform sampler2D	uTexturePosition;
uniform sampler2D	uTextureVelocity;

uniform float		uDampen;
uniform vec3		uCenter;
uniform float		uSpeed;

varying vec2		vUv;

void main ( void )
{
	vec3 position		= texture2D( uTexturePosition,	vUv ).rgb;
	vec3 velocity		= texture2D( uTextureVelocity,	vUv ).rgb;

	vec3 destination	= vec3( vUv.s, vUv.t, 1.0 - texture2D( uTextureKinect, vUv ).b );
	float depth			= destination.z;

	velocity			+= normalize( destination - position ) * uSpeed * depth;
	
	if ( destination.z != position.z ) {
		position		+= velocity;
	}
	position			= ( position - uCenter ) * uDampen * 0.5;
	velocity			*= uDampen;

	gl_FragData[ 0 ]	= vec4( position, 1.0 );
	gl_FragData[ 1 ]	= vec4( velocity, 1.0 );
}
 