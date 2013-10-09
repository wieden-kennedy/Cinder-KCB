uniform sampler2D	colors;
uniform float		dampen;
uniform float		depthThreshold;
uniform sampler2D	destinations;	// Kinect image
uniform float		fade;
uniform float		minDistance;
uniform vec3		origin;
uniform sampler2D	positions;
uniform float		speed;
uniform sampler2D	velocities;

varying vec2		uv;

float rand( vec2 co )
{
	return fract( sin( dot( co.xy, vec2( 12.9898, 78.233 ) ) ) * 43758.5453 );
}

void main ( void )
{
	// Look up position in destination in Kinect image
	float depth					= texture2D( destinations,	uv ).r;
	vec3 destination			= vec3( uv, depth );
	if ( destination.z >= depthThreshold ) {
		destination				= origin;
	}

	// Get current color, position, and velocity
	vec4 color					= texture2D( colors,		uv );
	vec3 position				= texture2D( positions,		uv ).rgb;
	vec3 velocity				= texture2D( velocities,	uv ).rgb;
	
	// Update velocity and position
	/*vec3 direction				= destination - position;
	float distanceToDestination	= length( direction );
	if ( distanceToDestination <= minDistance ) {
		vec4 newColor;
		newColor.r				= rand( color.rg );
		newColor.g				= rand( color.gb );
		newColor.b				= rand( color.rb );
		newColor.a				= 1.0;
		color					= newColor;
		position				= origin;
		velocity				= vec3( 0.0 );
	}
	velocity					+= direction * speed;
	position					+= velocity;
	velocity					*= dampen;

	// Set alpha using distance from origin
	float distanceToOrigin		= length( origin - position );
	color.a						= min( distanceToOrigin * fade, 1.0 );*/

	// Update data
	gl_FragData[ 0 ]			= vec4( position, 1.0 );
	gl_FragData[ 1 ]			= vec4( velocity, 1.0 );
	gl_FragData[ 2 ]			= color;
}
 