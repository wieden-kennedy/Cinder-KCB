uniform sampler2D positions;

varying vec4 color;

void main( void ) 
{
	vec3 position 	= texture2D( positions, gl_MultiTexCoord0.st ) * 2.0 - vec3( 1.0 );
	gl_Position		= gl_ModelViewProjectionMatrix * vec4( position, 0.0 );
}
 