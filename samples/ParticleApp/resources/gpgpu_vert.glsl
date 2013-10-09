varying vec2 uv;

void main ( void )
{
	uv			= gl_MultiTexCoord0.st;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
 