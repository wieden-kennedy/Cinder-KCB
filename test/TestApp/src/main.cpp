// main.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include <tchar.h>

// add the Kinect for Windows SDK Include folder for HR values and NUI structs
// $(KINECTSDK10_DIR)inc;$(KINECT_TOOLKIT_DIR)inc;$(PROJECTDIR)Library
#include "Library\KinectWrapperLib.h"

void SimpleSetup()
{
	// Get the instance of the sensor you are going to use
	HKINECT hKinect = KinectOpenDefaultSensor();

	// create a frame buffer, get the details of the frame from the api
	KINECT_IMAGE_FRAME_FORMAT format = { sizeof(KINECT_IMAGE_FRAME_FORMAT), 0 };
    KinectGetColorFrameFormat( hKinect, &format );
    BYTE* pColorBuffer = new BYTE[format.cbBufferSize];

	// simple loop, demo grabs 10 frames over 10 seconds
	bool quit = false; 
	int count = 10;
    while (!quit)
    {
        if( KinectGetColorFrame( hKinect, format.cbBufferSize, pColorBuffer, nullptr ) )
        {
            // ProcessColorFrameData(&format, pColorBuffer);
        }
        // Other loop processing here
		
		// check for exit criteria
		if( count-- < 0 )
			quit = true;
		{
			Sleep(1000);
		}
    }

    delete [] pColorBuffer;
    KinectCloseSensor(hKinect);
}

void EnumerationSetup()
{
	HRESULT hr = S_OK;

	HKINECT hKinect = nullptr;
	WCHAR portID[KINECT_MAX_PORTID_LENGTH];

	int count = KinectGetSensorCount();
	for( int i = 0; i < count; ++i )
	{
		if( KinectGetPortIDByIndex( i, _countof(portID), portID ) )
		{
			hKinect = KinectOpenSensor( portID );
			if( nullptr != hKinect )
			{
				// if it is valid, then we can use it
				break;
			}
		}
	}
	// as long as there are sensors attached
	// if you  know the PortID for the sensor, then
	//if( nullptr == hKinect )
	//{
	//	hKinect = KinectOpenSensor( L"USB\\VID_045E&PID_02C2\\6&2B73FD2A&0&3" ); // port 2
	//	//hKinect = KinectOpenSensor( L"USB\\VID_0409&PID_005A\\6&648DF71&0&2" );
	//	//hKinect = KinectOpenSensor( L"USB\\VID_045E&PID_02C2\\6&4E6C5B4&0&3" ); // port 1
	//}

	// color - enable and set the resolution
	// enable stream and set the format at the same time, but frame is options
	KINECT_IMAGE_FRAME_FORMAT colorFormat = { sizeof(KINECT_IMAGE_FRAME_FORMAT), 0 };
	hr = KinectEnableColorStream( hKinect, NUI_IMAGE_RESOLUTION_640x480, &colorFormat );

	BYTE* pColorBuffer = nullptr;
	if( SUCCEEDED(hr) )
	{
		pColorBuffer = new BYTE[colorFormat.cbBufferSize];
	}

	/* FOR IR FRAME */
	//// can only have one color stream, so last one wins
	//KINECT_IMAGE_FRAME_FORMAT irFormat = { sizeof(KINECT_IMAGE_FRAME_FORMAT), 0 };
	//hr = KinectEnableIRStream( hKinect, NUI_IMAGE_RESOLUTION_640x480, &irFormat );

	//BYTE* pIRBuffer = nullptr;
	//if( SUCCEEDED(hr) )
	//{
	//	pIRBuffer = new BYTE[irFormat.cbBufferSize];
	//}

	// depth - enable nearMode and set 320x240 depth resolution (Xbox 360 senosr?)
	hr = KinectEnableDepthStream( hKinect, true, NUI_IMAGE_RESOLUTION_320x240, nullptr ); // should this be allowed?
	KINECT_IMAGE_FRAME_FORMAT depthFormat = { sizeof(KINECT_IMAGE_FRAME_FORMAT), 0 };
	BYTE* pDepthBuffer = nullptr;
	if( SUCCEEDED( KinectGetDepthFrameFormat(hKinect, &depthFormat) ) )
	{
		pDepthBuffer = new BYTE[depthFormat.cbBufferSize];
	}

	// skeletal - near mode ST with seated skeletal(upper body only)
	// last param, how we track the users is set to default
	KinectEnableSkeletalStream( hKinect, true, SkeletonSelectionModeDefault, nullptr );
	NUI_SKELETON_FRAME SkeletonFrame = {0};

	// loop to grab frames
	bool quit = false; 
	count = 0x0000ffff;	// a big number for # of loops
	LONGLONG timestamp;
    while( !quit )
    {
		// get a depth frame - not aligning the data to color yet
		if( KinectIsDepthFrameReady(hKinect) && SUCCEEDED( KinectGetDepthFrame( hKinect, depthFormat.cbBufferSize, pDepthBuffer, &timestamp ) ) )
        {
            // ProcessColorFrameData(&format, pColorBuffer);
			//printf( "Depth frame acquired: %d\r\n", timestamp );
        }
		else
		{
		//	// a way to check for why it failed
		//	HRESULT hrSensor = KinectGetSensorStatus( hKinect );
		//	if( hr == E_NUI_STREAM_NOT_ENABLED && hrSensor == E_NUI_DEVICE_NOT_CONNECTED )
		//		// take action on why the device and streams are not enabled - probably device not connected
		//	else if( hr == E_NUI_FRAME_NO_DATA )
		//		// wait for the next frame
		//	else
		//		// some other error that was bubbled up - device not genuine, Xbox 360 in release mode, etc...
		}

		// get a color frame
		if( KinectIsColorFrameReady(hKinect) && SUCCEEDED( KinectGetColorFrame(hKinect, colorFormat.cbBufferSize, pColorBuffer, &timestamp) ) )
        {
            // ProcessColorFrameData(&format, pColorBuffer);
			printf( "Color frame acquired: %d\r\n", timestamp );
        }

		//// get a ir frame
		//if( KinectIsColorFrameReady(hKinect) && SUCCEEDED( KinectGetColorFrame(hKinect, irFormat.cbBufferSize, pIRBuffer, &timestamp) ) )
  //      {
  //          // ProcessColorFrameData(&format, pColorBuffer);
		//	printf( "IR frame acquired: %d\r\n", timestamp );
		//	for (DWORD i = 0; i < irFormat.dwWidth * irFormat.dwHeight; ++i)
		//	{
		//		BYTE intensity = reinterpret_cast<USHORT*>(pIRBuffer)[i] >> 8;

		//		// first position for the pixel in the color buffer
		//		BYTE *pPixel = &pColorBuffer[i];
		//		*(pPixel + 0) = intensity;	// blue
		//		*(pPixel + 1) = intensity;	// green
		//		*(pPixel + 2) = intensity;	// red
		//		*(pPixel + 3) = 255;		// alpha
		//	}

  //      }

		// get a skeletal frame 
		if( KinectIsSkeletonFrameReady(hKinect) && SUCCEEDED( KinectGetSkeletonData( hKinect, &SkeletonFrame ) ) )
        {
			const NUI_SKELETON_DATA* pSkeletonData = SkeletonFrame.SkeletonData;
			// process skeleton
			for( int skelNum = 0 ; skelNum < NUI_SKELETON_COUNT; ++skelNum )
			{
				NUI_SKELETON_TRACKING_STATE trackingState = pSkeletonData[skelNum].eTrackingState;
				if( NUI_SKELETON_TRACKED == trackingState )
				{
					// We're tracking the skeleton, draw it
					// DrawSkeleton(SkeletonData[i]);
					for( int jointNum = 0; jointNum < NUI_SKELETON_POSITION_COUNT; ++jointNum )
					{
						Vector4 skeletonPoint = pSkeletonData[skelNum].SkeletonPositions[jointNum];
						LONG x, y;
						USHORT depth;
						NuiTransformSkeletonToDepthImage( skeletonPoint, &x, &y, &depth);

						UINT windowWidth = 800, windowHeight = 600;
						float screenPointX = static_cast<float>(x * windowWidth) / depthFormat.dwWidth;
						float screenPointY = static_cast<float>(y * windowHeight) / depthFormat.dwHeight;
						printf( "Joint %d:\r\nPos: (%d, %d) screen: (%f, %f) depth: %d\r\n", jointNum, x, y, screenPointX, screenPointY, depth );
						printf("\r\n");
					}
				}
				else if( NUI_SKELETON_POSITION_ONLY == trackingState )
				{
					// we've only received the center point of the skeleton, draw that
					// SkeletonToScreen( pSkeletonData[skelNum].Position, 15, 15 )

				}
			}
        }

        // Other loop processing here
		
		// check for exit criteria
		if( count-- < 0 )
		{
			quit = true;
		}
		Sleep(1);
    }

    delete [] pColorBuffer;
    delete [] pDepthBuffer;

	KinectCloseSensor(hKinect);
}

void DeferSensorSetup()
{

}

int _tmain(int argc, _TCHAR* argv[])
{
	EnumerationSetup();

	return 0;
}

