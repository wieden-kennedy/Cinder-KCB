uniform float		uDepth;
uniform float		uTime;
uniform sampler2D	uTexturePosition;

varying vec4		vColor;

void main( void ) 
{
	vec2 uv			= gl_MultiTexCoord0.st;

	vec4 position 	= gl_Vertex;
	position.z		= texture2D( uTexturePosition, uv ).b;

	vColor			= vec4( uv.s * ( 1.0 - position.z ) + 0.5, 0.5 - position.z * 0.5, uv.t * ( position.z * 0.5 ) + 0.1, position.z );
	vColor.rgb		+= sin( uTime * 0.5 ) * 0.2;
	vColor			= clamp( vColor * 0.5 + 0.5 * vColor * vColor * 8.0, 0.0, 1.0 );

	position.x		*= -2.0;
	position.z		*= uDepth + sin( uTime + uv.s * 10.0 ) * ( uDepth * 0.033 );

	gl_Position		= gl_ModelViewProjectionMatrix * position;
}
 