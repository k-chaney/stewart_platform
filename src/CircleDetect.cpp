
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <dc1394/dc1394.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;



void printHelp( void )
{
printf("Usage: ProcessFiles \n"
	   "GrabFFFrames Usage: \n"
	   "     [-r <rate>]        # manually specify frame rate (frames/second), default is 30 \n"
	   "     [-b <brightness>]  # manually specify brightness (1-255), default is auto \n"
	   "     [-e <exposure>]    # manually specify exposure (7-62), default is auto \n"
	   "     [-g <gamma>]       # manually specify gamma (0-1), default is off \n"
	   "     [-s <shutter>]     # manually specify shutter speed (1-531), default is auto \n"
	   "     [-gn <gain>]       # manually specify gain (16-64), default is auto \n"
	   "     [-bf <frames>]     # manually specify number of images in the camera's ring buffer, default is 1 \n"
	   "     [-small]           # make output image is 1/2 size \n"
	   "     [-f <maxframes>]   # manually specify number of images to be captured before exiting \n"
	   "     [-t]               # turn triggering on \n"
	   "     [-save <prefix>]   # save the processed files to disk using prefix in the file name \n"
	   "     [-o]               # display the camera's options \n"
	   "\n" );
}


IplImage *dc1394_frame_get_iplimage(dc1394video_frame_t *frame)
{
    IplImage *img;
    unsigned char *imdata;
    dc1394video_mode_t video_mode = frame->video_mode;
    CvSize size = cvSize(frame->size[0], frame->size[1]);

    if (video_mode == DC1394_VIDEO_MODE_640x480_MONO8) {

       // g_return_val_if_fail(
       //     (size.width * size.height * 1 * sizeof(unsigned char)) == frame->image_bytes,
       //     NULL);

        IplImage *tmp = cvCreateImageHeader(size, IPL_DEPTH_8U, 1);
        cvSetData(tmp, frame->image, size.width);

        img = cvCreateImage(size, IPL_DEPTH_8U, tmp->nChannels);
        cvCopy(tmp, img, 0);

        cvReleaseImageHeader(&tmp);

    } else if (video_mode == DC1394_VIDEO_MODE_640x480_MONO16) {

       // g_return_val_if_fail(
       //     (size.width * size.height * 2 * sizeof(unsigned char)) == frame->image_bytes,
       //     NULL);

        IplImage *tmp = cvCreateImageHeader(size, IPL_DEPTH_16U, 1);
        cvSetData(tmp, frame->image, size.width*2);

        img = cvCreateImage(size, IPL_DEPTH_16U, tmp->nChannels);
        cvCopy(tmp, img, 0);

        cvReleaseImageHeader(&tmp);

    } else if ((video_mode == DC1394_VIDEO_MODE_FORMAT7_0) ||
               (video_mode == DC1394_VIDEO_MODE_FORMAT7_1)) {


            dc1394error_t err;
            dc1394video_frame_t dest;
            IplImage *tmp;

            img = cvCreateImageHeader(size, IPL_DEPTH_8U, 3);

            /* debayer frame into RGB8 */
            imdata = (unsigned char *)malloc(frame->size[0]*frame->size[1]*3*sizeof(unsigned char));

            dest.image = imdata;

            dest.color_coding = DC1394_COLOR_CODING_RGB8;

            err=dc1394_debayer_frames(frame, &dest, DC1394_BAYER_METHOD_NEAREST); 
            if (err != DC1394_SUCCESS)
                dc1394_log_error("Could not convert/debayer frames");

            /* convert from RGB to BGR */
            tmp = cvCreateImageHeader(cvSize(frame->size[0], frame->size[1]), IPL_DEPTH_8U, 3);

            cvSetData(tmp, imdata, frame->size[0]*3);

				img = cvCloneImage(tmp);

            cvCvtColor(tmp, img, CV_RGB2BGR);

            free(imdata);
            cvReleaseImageHeader(&tmp);
				
    } else {

    }

    return img;
}


void print_format( uint32_t format )
{
#define print_case(A) case A: printf(#A "\n"); break;
	
    switch( format ) {
			print_case(DC1394_VIDEO_MODE_160x120_YUV444);
			print_case(DC1394_VIDEO_MODE_320x240_YUV422);
			print_case(DC1394_VIDEO_MODE_640x480_YUV411);
			print_case(DC1394_VIDEO_MODE_640x480_YUV422);
			print_case(DC1394_VIDEO_MODE_640x480_RGB8);
			print_case(DC1394_VIDEO_MODE_640x480_MONO8);
			print_case(DC1394_VIDEO_MODE_640x480_MONO16);
			print_case(DC1394_VIDEO_MODE_800x600_YUV422);
			print_case(DC1394_VIDEO_MODE_800x600_RGB8);
			print_case(DC1394_VIDEO_MODE_800x600_MONO8);
			print_case(DC1394_VIDEO_MODE_1024x768_YUV422);
			print_case(DC1394_VIDEO_MODE_1024x768_RGB8);
			print_case(DC1394_VIDEO_MODE_1024x768_MONO8);
			print_case(DC1394_VIDEO_MODE_800x600_MONO16);
			print_case(DC1394_VIDEO_MODE_1024x768_MONO16);
			print_case(DC1394_VIDEO_MODE_1280x960_YUV422);
			print_case(DC1394_VIDEO_MODE_1280x960_RGB8);
			print_case(DC1394_VIDEO_MODE_1280x960_MONO8);
			print_case(DC1394_VIDEO_MODE_1600x1200_YUV422);
			print_case(DC1394_VIDEO_MODE_1600x1200_RGB8);
			print_case(DC1394_VIDEO_MODE_1600x1200_MONO8);
			print_case(DC1394_VIDEO_MODE_1280x960_MONO16);
			print_case(DC1394_VIDEO_MODE_1600x1200_MONO16);
			print_case(DC1394_VIDEO_MODE_EXIF);
			print_case(DC1394_VIDEO_MODE_FORMAT7_0);
			print_case(DC1394_VIDEO_MODE_FORMAT7_1);
			print_case(DC1394_VIDEO_MODE_FORMAT7_2);
			print_case(DC1394_VIDEO_MODE_FORMAT7_3);
			print_case(DC1394_VIDEO_MODE_FORMAT7_4);
			print_case(DC1394_VIDEO_MODE_FORMAT7_5);
			print_case(DC1394_VIDEO_MODE_FORMAT7_6);
			print_case(DC1394_VIDEO_MODE_FORMAT7_7);
			
		default:
			dc1394_log_error("Unknown format\n");
			//exit(1);
    }
}


int main(int argc, char *argv[])
{
	bool brightnessFlag = 0;
	int brightness = 0;
	bool exposureFlag = 0;
	int exposure = 0;
	bool gammaFlag = 0;
	int gamma = 0;
	bool shutterFlag = 0;
	bool triggerFlag = 0;
	int shutter = 0;
	bool gainFlag = 0;
	bool optionFlag = 0;
	int gain = 0;
	int maxFrames = 0;
	int buffer = 1;
	int rate = 30;
	bool saveFlag = 0;
	bool smallFlag = 0;
	const char* prefix;
	char filename[256];
	
	if (argc == 1)
	{
		printHelp();
	}
	
	for( int i = 1; i < argc; i++ )
    	{
        const char* s = argv[i];
	if( strcmp( s, "-r" ) == 0 )
        {
		rate = 1000 / atoi( argv[++i] );
        }
	if( strcmp( s, "-b" ) == 0 )
        {
		brightnessFlag = 1;
		brightness = atoi( argv[++i] );
        }
        else if( strcmp( s, "-e" ) == 0 )
        {
		exposureFlag = 1;
		exposure = atoi( argv[++i] );
        }
	else if( strcmp( s, "-g" ) == 0 )
        {
		gammaFlag = 1;
		gamma = atoi( argv[++i] );       
	}
	else if( strcmp( s, "-s" ) == 0 )
        {
		shutterFlag = 1;
		shutter = atoi( argv[++i] );
        }
	else if( strcmp( s, "-gn" ) == 0 )
        {
		gainFlag = 1;
		gain = atoi( argv[++i] );
        }
	else if( strcmp( s, "-f" ) == 0 )
        {
		maxFrames = atoi( argv[++i] );
        }
	else if( strcmp( s, "-bf" ) == 0 )
        {
		buffer = atoi( argv[++i] );
        }
	else if( strcmp( s, "-save" ) == 0 )
        {
		saveFlag = 1;
		prefix = argv[++i];
        }
	else if( strcmp( s, "-t" ) == 0 )
        {
		triggerFlag = 1;
		rate = 2; 
        }
	else if( strcmp( s, "-o" ) == 0 )
        {
		optionFlag = 1;
        }
	else if( strcmp( s, "-small" ) == 0 )
        {
		smallFlag = 1;
        }
	else if( ( strcmp( s, "-help" ) == 0 ) || ( strcmp( s, "-h" ) == 0 ) )
        {
		printHelp();
		return 0;
        }
		
    }	
	

	dc1394camera_t * camera;
	dc1394error_t err;
	dc1394video_frame_t * frame;
	dc1394video_frame_t * color_frame;
	dc1394_t * d;
	dc1394camera_list_t * list;
	dc1394featureset_t features;
	dc1394video_mode_t video_mode;
	dc1394video_modes_t modes;
	dc1394color_coding_t	color_coding;
	dc1394bool_t is_color;
	unsigned int width, height;

	// initialize 
	d = dc1394_new ();                                                    
	if (!d)
	return 1;

	// find cameras
	err=dc1394_camera_enumerate (d, &list);                              
	DC1394_ERR_RTN(err,"Failed to enumerate cameras");

	// check that we have at least one camera
	if (list->num == 0) {                                                  
		dc1394_log_error("No cameras found");
		return 1;
	}

	// select the first camera
	camera = dc1394_camera_new (d, list->ids[0].guid);                    
	if (!camera) {
		dc1394_log_error("Failed to initialize camera with guid %llx", list->ids[0].guid);
		return 1;
	}

	// free all of the other cameras
	dc1394_camera_free_list (list);

	if (triggerFlag != 0)
	{
		// Set the camera to capture based on the trigger input
        err = dc1394_external_trigger_set_power(camera, DC1394_ON);
		
        // Set the source of the trigger firing
        err = dc1394_external_trigger_set_source(camera, DC1394_TRIGGER_SOURCE_0);
		
        // Set the trigger mode to 0 (start capture on trigger fire, stop after pre-set time)
        err = dc1394_external_trigger_set_mode(camera, DC1394_TRIGGER_MODE_0);
    }
    else
    {
        // Set the camera to capture based on the trigger input
        err = dc1394_external_trigger_set_power(camera, DC1394_OFF);
    }
	
	
	// setup for capture with only specified number of frames in the ring buffer
	err=dc1394_capture_setup(camera, buffer, DC1394_CAPTURE_FLAGS_DEFAULT);    

	// set frame rate (not necessarily related to capture frame rate)...
	err=dc1394_video_set_framerate(camera, DC1394_FRAMERATE_60);
	
	
	if (brightnessFlag != 0)
	{
		// set brightness
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
		err=dc1394_feature_set_value(camera, DC1394_FEATURE_BRIGHTNESS, brightness);
	}
	else
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);
	
	if (exposureFlag != 0)
	{
		// set exposure
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
		err=dc1394_feature_set_value(camera, DC1394_FEATURE_EXPOSURE, exposure);
	}
	else
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);

	if (gammaFlag != 0)
	{
		// set gamma
		err=dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_ON);
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_MANUAL);
		err=dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA, gamma);
	}
	else
		err=dc1394_feature_set_power(camera, DC1394_FEATURE_GAMMA, DC1394_OFF);
	
	if (shutterFlag != 0)
	{
		// set shutter 
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
		err=dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER, shutter);
	}
	else
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);

	if (gainFlag != 0)
	{
		// set gain 
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
		err=dc1394_feature_set_value(camera, DC1394_FEATURE_GAIN, gain);
	}
	else
		err=dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
	
	// list available capture modes
	err=dc1394_video_get_supported_modes(camera, &modes);
    DC1394_ERR_RTN(err,"Could not get list of modes");

	for (unsigned int video_mode_index = 0; video_mode_index < modes.num; video_mode_index++) 
	{
		cout << "video mode option: " << video_mode_index << ", " << modes.modes[video_mode_index] << endl;
		print_format( modes.modes[video_mode_index] );
		cout << endl;
	}
	video_mode = modes.modes[2];

	err=dc1394_video_set_mode(camera, video_mode);
	
	dc1394_get_image_size_from_video_mode(camera, video_mode, &width, &height);
 
	dc1394_format7_get_color_coding(camera, video_mode, &color_coding);
	dc1394_is_color(color_coding, &is_color);

	dc1394color_filter_t filter_id;
	dc1394_format7_get_color_filter(camera, video_mode, &filter_id);

	// start transmission
	err=dc1394_video_set_transmission(camera, DC1394_ON);                 

	if (optionFlag != 0)
	{
		// show the camera's features
		err=dc1394_feature_get_all(camera,&features);
		if (err!=DC1394_SUCCESS) 
		{
			dc1394_log_warning("Could not get feature set");
		}
		else 
		{
			dc1394_feature_print_all(&features, stdout);
		}
	}
		


	// setup the IplImage
	IplImage * CapturedImage = cvCreateImage(cvSize (width,   height  ), IPL_DEPTH_8U, 3);
	IplImage * SmallImage    = cvCreateImage(cvSize (width/2, height/2), IPL_DEPTH_8U, 3); 	

	Mat CircleImage;
	int frameNum = 1;

	namedWindow( "Captured Image", CV_WINDOW_AUTOSIZE );

	for (;;)
	{
			cout << frameNum << endl;
	   err=dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);/* Capture */
		DC1394_ERR_RTN(err,"Problem getting an image");

   	CapturedImage = dc1394_frame_get_iplimage( frame );

		// release buffer
		err=dc1394_capture_enqueue(camera, frame);                           

		Mat CapturedImageMat;
		//cout << "1 " << endl;
		if (smallFlag != 0)
		{
			cvResize( CapturedImage, SmallImage, CV_INTER_LINEAR);
			CapturedImageMat = cvarrToMat(SmallImage);
		}
		else
		{
			CapturedImageMat = cvarrToMat(CapturedImage);
		}
		
		//cout << "2 " << endl;

		Mat CircleImage;// = CapturedImageMat;

		cvtColor( CapturedImageMat, CircleImage, CV_BGR2GRAY );

		//cout << "3 " << endl;

  		GaussianBlur( CircleImage, CircleImage, Size(9, 9), 2, 2 );
  		vector<Vec3f> circles;
	  	HoughCircles( CircleImage, circles, CV_HOUGH_GRADIENT, 1, CircleImage.rows/8, 200, 100, 0, 0 );

		cout << "circles" << circles.size() << endl;

		if ( saveFlag != 0 )
		{
			if (frameNum < 10) 
				sprintf( filename, "%s_0000%d.jpg", prefix, frameNum ); 
			else if (frameNum < 100)
				sprintf( filename, "%s_000%d.jpg", prefix, frameNum ); 
			else if (frameNum < 1000)
				sprintf( filename, "%s_00%d.jpg", prefix, frameNum ); 
			else if (frameNum < 10000)
				sprintf( filename, "%s_0%d.jpg", prefix, frameNum ); 
			else 
				sprintf( filename, "%s_%d.jpg", prefix, frameNum ); 
				cvSaveImage( filename, CapturedImage );

		}
  		/// Draw the circles detected
  		for( size_t i = 0; i < circles.size(); i++ )
  		{
      	Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      	int radius = cvRound(circles[i][2]);
      	// circle center
      	circle( CapturedImageMat, center, 3, Scalar(0,255,0), -1, 8, 0 );
      	// circle outline
      	circle( CapturedImageMat, center, radius, Scalar(0,0,255), 3, 8, 0 );
   	}
	
		imshow( "Captured Image", CapturedImageMat );
	
		
		if( ( maxFrames > 0 ) && ( frameNum == maxFrames ))
		{
			break;
		}
		
		if( cvWaitKey( rate ) >= 0 )
			break;
		
		frameNum++;
	
	}


	// stop transmission
   err=dc1394_video_set_transmission(camera, DC1394_OFF);                
    // stop capture
	err=dc1394_capture_stop(camera);                                       
    // free camera
	dc1394_camera_free(camera);    
	// uninitialize
	dc1394_free(d);
	
    return 0;
}
