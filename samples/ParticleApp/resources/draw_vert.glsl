uniform sampler2D colors;
uniform sampler2D positions;

varying vec4 color;

void main( void ) 
{
	vec2 uv			= gl_MultiTexCoord0.st;
	color			= texture2D( colors, uv );
	vec3 position 	= texture2D( positions, uv ).rgb * 2.0 - vec3( 1.0 );
	gl_Position		= gl_ModelViewProjectionMatrix * vec4( position, 1.0 );
}
 