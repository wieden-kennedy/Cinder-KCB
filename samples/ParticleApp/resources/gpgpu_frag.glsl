uniform sampler2D	kinect;
uniform sampler2D	positions;
uniform sampler2D	velocities;

uniform float		dampen;
uniform vec3		center;
uniform float		speed;

varying vec2		uv;

void main ( void )
{
	vec3 position		= texture2D( positions,		uv ).rgb;
	vec3 velocity		= texture2D( velocities,	uv ).rgb;

	vec3 destination	= vec3( uv.s, uv.t, 1.0 - texture2D( kinect, uv ).b );
	float depth			= destination.z;

	velocity			+= normalize( destination - position ) * speed * depth;
	
	position			+= velocity;
	position			= ( position - center ) * dampen * 0.5;
	velocity			*= dampen;

	gl_FragData[ 0 ]	= vec4( position, 1.0 );
	gl_FragData[ 1 ]	= vec4( velocity, 1.0 );
}
 