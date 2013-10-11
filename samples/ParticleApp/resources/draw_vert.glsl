uniform sampler2D positions;

varying vec4 color;

void main( void ) 
{
	vec2 uv			= gl_MultiTexCoord0.st;

	vec4 position 	= gl_Vertex;
	position.z		= texture2D( positions, uv ).b;

	color			= vec4( uv.s * ( 1.0 - position.z ) + 0.5, 0.6 - position.z * 0.5, uv.t * ( position.z * 0.5 ) + 0.3, position.z );
	color			= clamp( color * 0.5 + 0.5 * color * color * 8.0, 0.0, 1.0 );

	position.x		*= -2.0;
	position.z		*= 400.0;

	gl_Position		= gl_ModelViewProjectionMatrix * position;
}
 