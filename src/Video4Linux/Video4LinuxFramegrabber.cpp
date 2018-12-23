/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads camera images using Video4Linux Interface.
 *
 * The Component is based on the example from
 * http://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html
 *
 * @author Christian Waechter <christian.waechter@in.tum.de>
 */


// std
#include <list>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include <iostream>
#include <strstream>

// v4l
#include <libv4l2.h>

#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>


// boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>


// functions for yuv to rgb24 conversion
#include "image_conversion.h"

// Ubitrack
#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>

#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <opencv/cv.h>
#include <utVision/OpenCLManager.h>

// get a logger
#include <log4cpp/Category.hh>
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.Video4LinuxFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;

// static void errno_exit(const char *s)
// {
        // fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        // exit(EXIT_FAILURE);
// }

static int xioctl( int fh, int request, void *arg )
{
	int r;

	do
	{
		r = ioctl( fh, request, arg );
	}
	while( -1 == r && EINTR == errno );

	return r;
}

bool set_camera_parameter( int fd, unsigned int ctrl_type, int ctrl_value, std::string name ){
	// set controls
	struct v4l2_queryctrl query;
	struct v4l2_control control;

    memset(&query,0,sizeof(v4l2_queryctrl));        //zero out the structures
    memset(&control,0,sizeof(control));

    query.id = ctrl_type;                       //Query exposure
	if(ioctl(fd, VIDIOC_QUERYCTRL, &query) == -1)
	{
		// error querying property
		LOG4CPP_ERROR( logger, "Error setting V4L camera control: "  << name << " parameter not found.");
	}
	else if(query.flags & V4L2_CTRL_FLAG_DISABLED)
	{
		// disabled
		LOG4CPP_WARN( logger, "Error setting V4L camera control: "  << name << " parameter disabled.");
	}
	else
	{
		if (ctrl_value != query.default_value) {
			if ((ctrl_value <= query.maximum) && (ctrl_value >= query.minimum)) {
				control.id = ctrl_type;
				control.value = ctrl_value;
				if(ioctl(fd, VIDIOC_S_CTRL, &control) == -1) {
					// error setting
					LOG4CPP_WARN( logger, "Set V4L camera control: "  << name << " to value: " << ctrl_value << " FAILED!");
				} else {
					LOG4CPP_INFO( logger, "Set V4L camera control: "  << name << " to value: " << ctrl_value);
					return true;
				}
			} else {
				// not within bounds
				LOG4CPP_WARN( logger, "Error setting V4L camera control: "  << name << " not within bounds: " << ctrl_value << " (" << query.minimum << " - " << query.maximum << ")");
			}
		} else {
			return true;
		}
	}
	return false;
}


template< typename T >
inline void CLEAR( T value )
{
	memset( &( value ), 0, sizeof( value ) );
};

struct buffer
{
	void*		start;
	std::size_t	length;
};

enum io_method
{
	IO_METHOD_MMAP,
	IO_METHOD_READ,
	IO_METHOD_USERPTR,
};


//right now only two color space formats are supported :(
static const uint32_t pix_formats [] = {
	V4L2_PIX_FMT_BGR24,
	V4L2_PIX_FMT_YUV420,	// 'YU12', e.g. Quickcam 4000
	V4L2_PIX_FMT_YUYV /*, // 'YUV422', e.g. Quickcam 5500
	V4L2_PIX_FMT_YVU420,	// 'YV12'


	V4L2_PIX_FMT_YUV411P, 

	V4L2_PIX_FMT_UYVY,
	V4L2_PIX_FMT_MJPEG,
	V4L2_PIX_FMT_JPEG,
	V4L2_PIX_FMT_SN9C10X,
	V4L2_PIX_FMT_SBGGR8,
	V4L2_PIX_FMT_SGBRG */
};

/** Function to transform pixel format value into four character string */
static inline std::string fourcc( const uint32_t pixform )
{
	const char cc[] = { pixform & 0xFF, (pixform >> 8) & 0xFF, (pixform >> 16) & 0xFF, (pixform >> 24) & 0xFF, '\0' };
	return std::string( cc );
};

/** Function to transform version number into 5 character string */
static inline std::string version( const uint32_t value )
{
	const char verchar[] = { 48 + ((value >> 16) & 0xFF), '.',  48 + ((value >> 8) & 0xFF), '.', (value & 0xFF), '\0' };
	return std::string( verchar );
};


namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Measurement::ImageMeasurement.
 *
 */
template< io_method MEM_TYPE >
class Video4LinuxFramegrabber
	: public Dataflow::Component
{
protected:
	/** video device name on linux, e.g. /dev/video0 */
	const char* m_deviceName;
	
	/** identifier - file descriptor */
	const int m_fd;

	/** signs the capture device's pixelformat, e.g. RBG24 */
	uint32_t m_pixelFormat;

	/** several image buffers */
	struct buffer *m_buffers;

	/** amount of image buffers */
	std::size_t m_nBuffers;

	/** signs when to stop the thread*/
	bool m_bStop;

	/** shift timestamps (ms) */
	int m_timeOffset;

	/** only send every nth image */
	std::size_t m_divisor;

	/** desired width */
	std::size_t m_desiredWidth;

	/** desired image height */
	std::size_t m_desiredHeight;

	/** desired frame rate */
	std::size_t m_desiredFrameRate;

	/** output image width */
	std::size_t m_width;

	/** output image height */
	std::size_t m_height;

	/** number of frames received */
	std::size_t m_counter;

	/** timestamp of last frame */
	Measurement::Timestamp m_lastTime;

	/** automatic upload of images to the GPU*/
	bool m_autoGPUUpload;

	// the thread
	boost::scoped_ptr< boost::thread > m_Thread;

	/** timestamp synchronizer */
	//Measurement::TimestampSync m_syncer;

	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	/** color image output port*/
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

	/** grayscale image output port*/
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;

	/** intrinsics matrix output port **/
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;

	/** cameraintrinsics output port **/
    Dataflow::PullSupplier< Measurement::CameraIntrinsics > m_cameraModelPort;

	/** camera control parameters **/
	int m_shutter;
	int m_gain;
	int m_brightness;

public:

	/** constructor */
	Video4LinuxFramegrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~Video4LinuxFramegrabber();

	/** starts the camera */
	void start();

	/** stops the camera */
	void stop();



	/** handler method for incoming pull requests */

	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for V4L2FrameGrabber" );
		}
	}

    Measurement::CameraIntrinsics getCameraModel( Measurement::Timestamp t )
    {
        if (m_undistorter) {
            return Measurement::CameraIntrinsics( t, m_undistorter->getIntrinsics() );
        } else {
            UBITRACK_THROW( "No undistortion configured for V4L2FrameGrabber" );
        }
    }


protected:

	/** accesses the camera device */
	int openDevice( const char* dev_name );
	
	/** checks if desired memory type works. */
	int checkMemoryType( const int fd, const char* dev_name );

	/** initializes the device. */
	int initDevice( const int fd, const char* dev_name, const std::size_t width, const std::size_t height );
	
	/** initializes the type of memory handling, depends on template parameter. */
	void initMemory( const int fd, const std::size_t );

	/** starts the capturing process. */
	void startCapture( const int fd );

	/** main thread that fetches the images. */
	void startLoop( const int fd );

	/** reads from capture device into memory, depends on template parameter. */
	int readFrame( const int fd );

	/** generates image structure. */
	void processImage( const Measurement::Timestamp, void* ptdImage, const std::size_t lengthImage );

	/** prepares the image and sends it to the dataflow network. */
	void handleFrame( const Measurement::Timestamp t, boost::shared_ptr<Vision::Image>& pColorImage );

	/** stops the capturing process. */
	void stopCapture( const int fd );

	/** uninitializes the memory, depends on template parameter. */
	void uninitMemory();

	/** closes the camera device. */
	int closeDevice( const int fd ) const;
};

template< io_method MEM_TYPE >
Video4LinuxFramegrabber< MEM_TYPE >::Video4LinuxFramegrabber( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_deviceName( subgraph->m_DataflowAttributes.getAttributeString( "cameraName" ).c_str() )
	, m_fd( openDevice( m_deviceName ) )
	, m_buffers( NULL )
	, m_nBuffers( 4 ) //amount of requested buffers, what is an adequate value?
	, m_bStop( true )
	, m_timeOffset( 0 )
	, m_divisor( 1 )
	, m_desiredWidth( 640 )
	, m_desiredHeight( 480 )
	, m_desiredFrameRate( 30 )
	, m_width( 0 )
	, m_height( 0 )
	, m_counter( 0 )
	, m_lastTime( 0 )
	, m_shutter( -1 )
	, m_gain( 0 )
	, m_brightness( 1 )
	, m_autoGPUUpload(false)
	//, m_syncer( 1.0 )
	//, m_undistorter( *subgraph )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &Video4LinuxFramegrabber::getIntrinsic, this, _1 ) )
    , m_cameraModelPort( "CameraModel", *this, boost::bind( &Video4LinuxFramegrabber::getCameraModel, this, _1 ) )
{
	// subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );
	// subgraph->m_DataflowAttributes.getAttributeData( "divisor", m_divisor );
	subgraph->m_DataflowAttributes.getAttributeData( "width", m_desiredWidth );
	subgraph->m_DataflowAttributes.getAttributeData( "height", m_desiredHeight );

	subgraph->m_DataflowAttributes.getAttributeData( "frameRate", m_desiredFrameRate );

	subgraph->m_DataflowAttributes.getAttributeData( "shutter", m_shutter );
	subgraph->m_DataflowAttributes.getAttributeData( "brightness", m_brightness );
	subgraph->m_DataflowAttributes.getAttributeData( "gain", m_gain );


	if (subgraph->m_DataflowAttributes.hasAttribute("cameraModelFile")){
		std::string cameraModelFile = subgraph->m_DataflowAttributes.getAttributeString("cameraModelFile");
		m_undistorter.reset(new Vision::Undistortion(cameraModelFile));
	}
	else {
		std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString("intrinsicMatrixFile");
		std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString("distortionFile");


		m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
	}

	Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
	if (oclManager.isEnabled()) {
		if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
			m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
			LOG4CPP_INFO(logger, "Upload to GPU enabled? " << m_autoGPUUpload);
		}
		if (m_autoGPUUpload){
			oclManager.activate();
			LOG4CPP_INFO(logger, "Require OpenCLManager");
		}
	}


	if( -1 != m_fd )
	{
		if( 0 == checkMemoryType( m_fd, m_deviceName) )	
		{
			int size = initDevice( m_fd, m_deviceName, m_desiredWidth, m_desiredHeight );
			initMemory( m_fd, size );
		}
	}
}

template< io_method MEM_TYPE >
Video4LinuxFramegrabber< MEM_TYPE >::~Video4LinuxFramegrabber()
{
	uninitMemory();
	closeDevice( m_fd );
}


template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::start()
{
	startCapture( m_fd );

	if ( !m_running )
	{
		m_running = true;
		m_bStop = false;
		m_Thread.reset( new boost::thread( boost::bind ( &Video4LinuxFramegrabber< MEM_TYPE >::startLoop, this, m_fd ) ) );
	}
	Component::start();
}

template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::stop()
{
	if ( m_running )
	{
		m_running = false;
		m_bStop = true;
		LOG4CPP_DEBUG( logger, "Trying to stop video4linux framegrabber");
		if ( m_Thread )
		{
			m_Thread->join();
			LOG4CPP_DEBUG( logger, "Trying to stop video4linux framegrabber, thread joined");
		}
	}
	
	stopCapture( m_fd );
	
	Component::stop();
}



/* returns fd */
template< io_method MEM_TYPE >
int Video4LinuxFramegrabber< MEM_TYPE >::openDevice( const char* dev_name )
{
	struct stat st;
	if (-1 == stat( dev_name, &st ) )
	{
		LOG4CPP_ERROR( logger, "Cannot identify device " << dev_name << ", error: \"" << strerror( errno ) << "\".");
		return -1;
	}

	if (!S_ISCHR( st.st_mode ) )
	{
		LOG4CPP_ERROR( logger, dev_name << " is no device, error : \"" << strerror( errno ) << "\"." );
		return -1;
	}

	int fd = open( dev_name, O_RDWR /* required */ | O_NONBLOCK, 0 );
	if (-1 == fd )
	{
		LOG4CPP_ERROR( logger, "Cannot open " << dev_name << ", error: \"" << strerror( errno ) << "\"." );
		return -1;
	}
	return fd;
}

template< io_method MEM_TYPE >
int Video4LinuxFramegrabber< MEM_TYPE >::checkMemoryType( const int fd, const char* dev_name )
{
	struct v4l2_capability cap;
	if( -1 == xioctl( fd, VIDIOC_QUERYCAP, &cap  ))
	{
		if( EINVAL == errno )
		{
			LOG4CPP_ERROR( logger, "Cannot open " << dev_name << ", " << dev_name << " is no V4L2 device, error: \"" << strerror( errno ) << "\"." );
			return -1;
			// fprintf(stderr, "%s is no V4L2 device\n", dev_name);
			// exit(EXIT_FAILURE);
		}
		else
		{
			UBITRACK_THROW("VIDIOC_QUERYCAP");
			// errno_exit();
		}
	}
	//std::cout << cap.driver << std::endl;
	LOG4CPP_INFO( logger, "Device "<< dev_name << " : \"" << cap.card << "\", bus \"" << cap.bus_info << "\", driver \"" << cap.driver << "\", version " << version( cap.version ).c_str() << "." );

	if( !( cap.capabilities & V4L2_CAP_VIDEO_CAPTURE ) )
	{

		LOG4CPP_ERROR( logger, "Cannot use " << dev_name << ", " << dev_name << " is no video capture device, error: \"" << strerror( errno ) << "\"." );
		return -1;
			// fprintf(stderr, "%s is no video capture device\n", dev_name);
			// exit(EXIT_FAILURE);
	}

	if( !( cap.capabilities & V4L2_CAP_STREAMING ) )
	{

		LOG4CPP_ERROR( logger, "Cannot use " << dev_name << ", " << dev_name << " does not support streaming, error: \"" << strerror( errno ) << "\"." );
		return -1;
	}



	return 0;
}
template< >
int Video4LinuxFramegrabber< IO_METHOD_READ >::checkMemoryType( const int fd, const char* dev_name )
{
	struct v4l2_capability cap;
	if( -1 == xioctl( fd, VIDIOC_QUERYCAP, &cap  ))
	{
		if( EINVAL == errno )
		{
			LOG4CPP_ERROR( logger, "Cannot open " << dev_name << ", " << dev_name << " is no V4L2 device." );
			return -1;
			// fprintf(stderr, "%s is no V4L2 device\n", dev_name);
			// exit(EXIT_FAILURE);
		}
		else
		{
			UBITRACK_THROW("VIDIOC_QUERYCAP");
			// errno_exit();
		}
	}

	if( !( cap.capabilities & V4L2_CAP_VIDEO_CAPTURE ) )
	{

		LOG4CPP_ERROR( logger, "Cannot use " << dev_name << ", " << dev_name << " is no video capture device." );
		return -1;
			// fprintf(stderr, "%s is no video capture device\n", dev_name);
			// exit(EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_READWRITE))
	{
		LOG4CPP_ERROR( logger, "Cannot use " << dev_name << ", it does not support read i/o." );
		return -1;
	}
	return 0;
}

template< io_method MEM_TYPE >
int Video4LinuxFramegrabber< MEM_TYPE >::initDevice( const int fd, const char* dev_name, const std::size_t width, const std::size_t height )
{
        /* Select video input, video standard and tune here. */
	struct v4l2_cropcap cropcap;
	CLEAR( cropcap );

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if ( 0 != xioctl(fd, VIDIOC_CROPCAP, &cropcap) )
		LOG4CPP_DEBUG( logger, "Device does not support cropcapping, no need to reset." )
	else
	{
		LOG4CPP_DEBUG( logger, "Trying to reset cropping area to camera default value." )

		struct v4l2_crop crop;
		CLEAR( crop );
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
			switch (errno) {
			case EINVAL:
					/* Cropping not supported. */
					break;
			default:
					/* Errors ignored. */
					break;
			}
	}
	struct v4l2_format fmt;
	CLEAR( fmt );
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
		UBITRACK_THROW("VIDIOC_G_FMT");
	LOG4CPP_DEBUG( logger, "Device default pixelformat "  << fourcc( fmt.fmt.pix.pixelformat ) << "." );
	LOG4CPP_DEBUG( logger, "Device default resolution  " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << " pixels." );

	{
		LOG4CPP_DEBUG( logger, "Discovering supported video formats:" )
		struct v4l2_fmtdesc vid_fmtdesc;    /* Enumerated video formats supported by the device \*/
		CLEAR( vid_fmtdesc );

		vid_fmtdesc.index = 0;
		vid_fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
     		char flags[][13] = {"uncompressed", "compressed"};  
		//fprintf(stdout, "\nDiscovering supported video formats:\n");
		

		 /* Send the VIDIOC_ENUM_FM ioctl and print the results */
		 while( ioctl( fd, VIDIOC_ENUM_FMT, &vid_fmtdesc ) == 0 )
		 {

			LOG4CPP_DEBUG( logger, "Format " << vid_fmtdesc.index << ": " << vid_fmtdesc.description << " (" << flags[vid_fmtdesc.flags] << ").")
			if( vid_fmtdesc.flags == 0 )
				fmt.fmt.pix.pixelformat = vid_fmtdesc.pixelformat;
				
		    	vid_fmtdesc.index++;/* Increment the index */
		 }
	}/* End of get_supported_video_formats() */


	
	LOG4CPP_DEBUG( logger, "Attempt to set device capture properties." )
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;
	if ( -1 != xioctl( fd, VIDIOC_S_FMT, &fmt ) )
		LOG4CPP_DEBUG( logger, "Device capture properties set succesfully." )
	else
	{
		for( std::size_t i( 0 ); i < 3; ++i )
		{

			CLEAR( fmt );
			fmt.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE;
			fmt.fmt.pix.width       = width;
			fmt.fmt.pix.height      = height;
			fmt.fmt.pix.field       = V4L2_FIELD_ANY;
			fmt.fmt.pix.pixelformat = pix_formats[ i ];

			//fmt.fmt.win.chromakey = 0;
			//fmt.fmt.win.field = V4L2_FIELD_ANY;
			//fmt.fmt.win.clips = 0;
			//fmt.fmt.win.clipcount = 0;

			if (-1 == xioctl( fd, VIDIOC_S_FMT, &fmt ) )
			{
				LOG4CPP_DEBUG( logger, "Device does not support pixelformat "  << fourcc( pix_formats[ i ] ) << ", error: \"" << strerror( errno ) << "\"." );
				//UBITRACK_THROW( "VIDIOC_S_FMT" );
			}
			else
				break;

		}
	}
	
	/* Note VIDIOC_S_FMT may change width and height. */
	m_width = fmt.fmt.pix.width;
	m_height = fmt.fmt.pix.height;
	m_pixelFormat = fmt.fmt.pix.pixelformat;
	LOG4CPP_INFO( logger, "Device resolution  set to " << fmt.fmt.pix.width << "x" << fmt.fmt.pix.height << " pixels." );
	LOG4CPP_INFO( logger, "Device pixelformat set to "  << fourcc( fmt.fmt.pix.pixelformat ) << " color format." );
	

	struct v4l2_streamparm setfps;
	CLEAR( setfps );
	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps.parm.capture.timeperframe.numerator = 1;
	setfps.parm.capture.timeperframe.denominator = m_desiredFrameRate;
	if( -1 == xioctl ( fd , VIDIOC_S_PARM, &setfps) )
	{
		LOG4CPP_ERROR( logger, "Could not set the framerate to " << setfps.parm.capture.timeperframe.numerator << "/" << setfps.parm.capture.timeperframe.denominator << ", error: \"" << strerror( errno ) << "\"." )


		setfps.parm.capture.timeperframe.numerator = 0;
		setfps.parm.capture.timeperframe.denominator = 0;
		xioctl ( fd , VIDIOC_S_PARM, &setfps);
		LOG4CPP_INFO( logger, "Setting device framerate to default " << setfps.parm.capture.timeperframe.numerator << "/" << setfps.parm.capture.timeperframe.denominator << "." )
	}
	else
		LOG4CPP_INFO( logger, "Device framerate is " << setfps.parm.capture.timeperframe.numerator << "/" << setfps.parm.capture.timeperframe.denominator << "." )

	/* Buggy driver paranoia. */
	std::size_t min = fmt.fmt.pix.width * 2;
	if ( fmt.fmt.pix.bytesperline < min )
		fmt.fmt.pix.bytesperline = min;
		
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if( fmt.fmt.pix.sizeimage < min )
		fmt.fmt.pix.sizeimage = min;

	// camera parameters
	bool ctrl_ok = false;
	if (m_shutter > 0) {
		ctrl_ok = set_camera_parameter(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "ExposureAuto" );
		if (!ctrl_ok)
			LOG4CPP_WARN( logger, "Error setting exposure to manual")
		ctrl_ok = set_camera_parameter(fd, V4L2_CID_EXPOSURE_ABSOLUTE, m_shutter, "Exposure" );
		if (!ctrl_ok)
			LOG4CPP_WARN( logger, "Error setting exposure to: " << m_shutter)
	}

	if (m_brightness != 0) {
		ctrl_ok = set_camera_parameter(fd, V4L2_CID_BRIGHTNESS, m_brightness, "Brightness" );
		if (!ctrl_ok)
			LOG4CPP_WARN( logger, "Error setting brightness to: " << m_brightness)
	}

	if (m_gain >= 0) {
//		ctrl_ok = set_camera_parameter(fd, V4L2_CID_AUTOGAIN, false, "GainAuto" );
//		if (!ctrl_ok)
//			LOG4CPP_WARN( logger, "Error setting gain to manual")
		ctrl_ok = set_camera_parameter(fd, V4L2_CID_GAIN, m_gain, "Gain" );
		if (!ctrl_ok)
			LOG4CPP_WARN( logger, "Error setting gain to: " << m_gain)
	}

	// more ctrls like:
	// V4L2_CID_CONTRAST
	// V4L2_CID_SATURATION
	// V4L2_CID_HUE
	// V4L2_CID_AUTO_WHITE_BALANCE
	// V4L2_CID_GAMMA
	// V4L2_CID_WHITE_BALANCE_TEMPERATURE
	//


	return fmt.fmt.pix.sizeimage;
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_MMAP >::initMemory( const int fd, const std::size_t /* ignored in this method */ )
{
	struct v4l2_requestbuffers req;

	CLEAR( req );

	req.count = m_nBuffers;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if( -1 == xioctl( fd, VIDIOC_REQBUFS, &req ) )
	{
		if (EINVAL == errno)
		{
			UBITRACK_THROW( "Memory mapping is not supported by device.");
			//UBITRACK_THROW( "Camera " << m_deviceName << " does not support memory mapping.");
			//fprintf(stderr, "%s does not support memory mapping\n", m_deviceName);
			// exit(EXIT_FAILURE);
		}
		else
		{
			UBITRACK_THROW("VIDIOC_REQBUFS");
			//errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2)
	{
		UBITRACK_THROW("Insufficient buffer memory on device." );
		// UBITRACK_THROW("Insufficient buffer memory on" << m_deviceName );
		// fprintf(stderr, "Insufficient buffer memory on %s\n", m_deviceName );
		// exit(EXIT_FAILURE);
	}

	m_nBuffers = req.count;
	m_buffers = reinterpret_cast< buffer* > ( calloc( req.count, sizeof( *m_buffers ) ) );

	if( !m_buffers )
		UBITRACK_THROW("Out of memory.");
	/*{

		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}*/

	for( std::size_t i( 0 ); i < req.count; ++i )
	{
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		if( -1 == xioctl( fd, VIDIOC_QUERYBUF, &buf) )
			UBITRACK_THROW("VIDIOC_QUERYBUF");
			//errno_exit("VIDIOC_QUERYBUF");

		m_buffers[ i ].length = buf.length;
		m_buffers[ i ].start =
		mmap( NULL /* start anywhere */,
			  buf.length,
			  PROT_READ | PROT_WRITE /* required */,
			  MAP_SHARED /* recommended */,
			  fd, buf.m.offset );

		if( MAP_FAILED == m_buffers[ i ].start )
			UBITRACK_THROW("mmap failed.");
			//errno_exit("mmap");
	}
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_READ >::initMemory( const int /* ignored in this method */, const std::size_t bufferSize )
{
        m_buffers = reinterpret_cast< buffer* >( calloc( 1, sizeof(*m_buffers) ) );

        if ( !m_buffers )
	{
		UBITRACK_THROW("Out of memory.");
        }

        m_buffers[ 0 ].length = bufferSize;
        m_buffers[ 0 ].start = malloc( bufferSize );

        if( !m_buffers[0].start )
	{
		UBITRACK_THROW("Out of memory.");
        }
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_USERPTR >::initMemory( const int fd, const std::size_t bufferSize )
{
	struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = m_nBuffers;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl( fd, VIDIOC_REQBUFS, &req ))
	{
                if ( EINVAL == errno )
		{
			LOG4CPP_ERROR( logger, "Capture device doe not support user pointer i/o." );
                        //fprintf(stderr, "%s does not support user pointer i/o\n", dev_name);
                        //exit(EXIT_FAILURE);
                }
		else
		{
			UBITRACK_THROW("VIDIOC_REQBUFS");
                        //errno_exit("VIDIOC_REQBUFS");
                }
        }

        m_buffers = reinterpret_cast< buffer* > ( calloc( req.count, sizeof( *m_buffers ) ) );

        if (!m_buffers)
	{
		UBITRACK_THROW( "Out of memory" );
                // fprintf(stderr, "Out of memory\n");
                // exit(EXIT_FAILURE);
        }

        for ( std::size_t i( 0 ); i < req.count; ++i)
	{
                m_buffers[ i ].length = bufferSize;
                m_buffers[ i ].start = malloc( bufferSize );
 
                if (!m_buffers[ i ].start )
		{
			UBITRACK_THROW( "Out of memory" );
                        //fprintf(stderr, "Out of memory\n");
                        //exit(EXIT_FAILURE);
                }
        }
}


template< >
void Video4LinuxFramegrabber< IO_METHOD_MMAP >::uninitMemory( )
{
	for (std::size_t i( 0 ); i < m_nBuffers; ++i )
		if (-1 == munmap( m_buffers[ i ].start, m_buffers[ i ].length))
			LOG4CPP_ERROR( logger, "Cannot unmap buffer " << i << ", trying next one." );
        free( m_buffers );

}

template< >
void Video4LinuxFramegrabber< IO_METHOD_READ >::uninitMemory( )
{
	free( m_buffers[ 0 ].start );
	free( m_buffers );
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_USERPTR >::uninitMemory( )
{
	for (std::size_t i( 0 ); i < m_nBuffers; ++i )
		free( m_buffers[ i ].start );
        free( m_buffers );
}


template< io_method MEM_TYPE >
int Video4LinuxFramegrabber< MEM_TYPE >::closeDevice( const int fd ) const
{
	if (-1 == close( fd ) )
		LOG4CPP_ERROR( logger, "Problem at closing the capture device." );
                
	return -1;
}



template< >
void Video4LinuxFramegrabber< IO_METHOD_READ >::startCapture( const int fd )
{
	/* Nothing to do. */
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_MMAP >::startCapture( const int fd )
{
	for (std::size_t i( 0 ); i < m_nBuffers; ++i )
	{
		struct v4l2_buffer buf;

		CLEAR( buf );
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		
		if (-1 == xioctl( fd, VIDIOC_QBUF, &buf) )
			UBITRACK_THROW("VIDIOC_QBUF" );
			//errno_exit("VIDIOC_QBUF");
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl( fd, VIDIOC_STREAMON, &type) )
		UBITRACK_THROW( "VIDIOC_STREAMON" );
		//errno_exit("VIDIOC_STREAMON");

}

template< >
void Video4LinuxFramegrabber< IO_METHOD_USERPTR >::startCapture( const int fd )
{
	for (std::size_t i( 0 ); i < m_nBuffers; ++i )
	{
		struct v4l2_buffer buf;
		
                CLEAR( buf );
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                buf.index = i;
                buf.m.userptr = (unsigned long)m_buffers[ i ].start;
                buf.length = m_buffers[ i ].length;
		
		if (-1 == xioctl( fd, VIDIOC_QBUF, &buf) )
			UBITRACK_THROW("VIDIOC_QBUF" );
			//errno_exit("VIDIOC_QBUF");
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl( fd, VIDIOC_STREAMON, &type) )
		UBITRACK_THROW( "VIDIOC_STREAMON" );
		//errno_exit("VIDIOC_STREAMON");

}

template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::startLoop( const int fd )
{
	std::size_t timeouts ( 0 );
	while( !m_bStop )
	{
                fd_set fds;
                FD_ZERO( &fds );
                FD_SET( fd, &fds );

                /* Timeout. */
		struct timeval tv;
                tv.tv_sec = 2;
                tv.tv_usec = 0;

                int r = select( fd + 1, &fds, NULL, NULL, &tv);

                if( -1 == r )
		{
			LOG4CPP_ERROR( logger, "selcet error: \"" << strerror( errno ) << "\", trying to fetch new frame." );
                        if (EINTR == errno)
                                continue;
			
                        //errno_exit("select");
                }

                if( 0 == r )
		{
			LOG4CPP_WARN( logger, "select timeout greater than " << 2 << " sec." );
			if( ++timeouts < 10 )
				continue;
			else
				break;
                        //fprintf(stderr, "select timeout\n");
                        //exit(EXIT_FAILURE);
                }

		
		readFrame( fd );
                /* EAGAIN - continue select loop. */
        }
}

template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::stopCapture( const int fd )
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if( -1 == xioctl( fd, VIDIOC_STREAMOFF, &type ) )
		UBITRACK_THROW( "VIDIOC_STREAMOFF" );
		//errno_exit("VIDIOC_STREAMOFF");
}

template< >
void Video4LinuxFramegrabber< IO_METHOD_READ >::stopCapture( const int fd )
{
	/* Nothing to do. */
}

template< >
int Video4LinuxFramegrabber< IO_METHOD_READ >::readFrame( const int fd )
{
	if (-1 == read(fd, m_buffers[ 0 ].start, m_buffers[ 0 ].length))
	{
		if( errno == EAGAIN )
			return 0;
		else
			UBITRACK_THROW("read");
	}

	processImage( Measurement::now(), m_buffers[ 0 ].start, m_buffers[ 0 ].length );
	return 1;
}

template< >
int Video4LinuxFramegrabber< IO_METHOD_MMAP >::readFrame( const int fd )
{
        
	struct v4l2_buffer buf;
	
	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if( -1 == xioctl(fd, VIDIOC_DQBUF, &buf ) )
	{
		if( errno == EAGAIN )
			return 0;
		else
			UBITRACK_THROW("VIDIOC_DQBUF");
                        //errno_exit("VIDIOC_DQBUF");
	}

	assert( buf.index < m_nBuffers );
	Measurement::Timestamp t_capture = buf.timestamp.tv_sec  * 1e09 + buf.timestamp.tv_usec * 1e03;
        processImage( t_capture, m_buffers[ buf.index ].start, buf.bytesused );

        if (-1 == xioctl( fd, VIDIOC_QBUF, &buf ) )
		UBITRACK_THROW("VIDIOC_QBUF");
		//errno_exit("VIDIOC_QBUF");
	return 1;
}

template< >
int Video4LinuxFramegrabber< IO_METHOD_USERPTR >::readFrame( const int fd )
{
	struct v4l2_buffer buf;
	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf) )
	{
		if( errno == EAGAIN )
			return 0;
		else
			UBITRACK_THROW("VIDIOC_DQBUF");
	                //errno_exit("VIDIOC_DQBUF");
        }
	std::size_t i( 0 );
        for (; i < m_nBuffers; ++i )
		if (buf.m.userptr == (unsigned long) m_buffers[i].start	&& buf.length == m_buffers[ i ].length )
                        break;

        assert(i < m_nBuffers);

        processImage( Measurement::now(), (void *)buf.m.userptr, buf.bytesused );

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		UBITRACK_THROW("VIDIOC_QBUF");
                //errno_exit("VIDIOC_QBUF");

	return 1;
}

template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::processImage( const Measurement::Timestamp t, void* ptdImage, const std::size_t lengthImage )
{	

	Vision::Image::ImageFormatProperties fmt;
			fmt.imageFormat = Vision::Image::BGR;
			fmt.channels = 3;
			fmt.depth = CV_8U;
			fmt.bitsPerPixel = 24;
			fmt.origin = 0;

	boost::shared_ptr< Vision::Image > pImage ( new Vision::Image( m_width, m_height, fmt ) );

	/// @todo change image copying/converiosn depending o n pixel format


	switch( m_pixelFormat ) 
	{
		case V4L2_PIX_FMT_BGR24:
			convert< V4L2_PIX_FMT_BGR24, V4L2_PIX_FMT_BGR24 >( m_width, m_height, reinterpret_cast< uint8_t* > ( ptdImage ), reinterpret_cast< uint8_t* > ( pImage->Mat().data ) );
			break;
		case V4L2_PIX_FMT_YUV420 :
			convert< V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_BGR24 >( m_width, m_height, reinterpret_cast< uint8_t* > ( ptdImage ), reinterpret_cast< uint8_t* > ( pImage->Mat().data ) );
			break;
		case V4L2_PIX_FMT_YUYV :
			convert< V4L2_PIX_FMT_YUYV , V4L2_PIX_FMT_BGR24 >( m_width, m_height, reinterpret_cast< uint8_t* > ( ptdImage ), reinterpret_cast< uint8_t* > ( pImage->Mat().data ) );
			break;
		default:
			break;
			/* todo: implement other image converison functions*/
	}

	/// @todo change image sending
	//m_outPort.send( Measurement::ImageMeasurement( t, pImage ) );
	//m_colorOutPort.send( Measurement::ImageMeasurement( t, pImage ) );
	handleFrame(t, pImage);
}

template< io_method MEM_TYPE >
void Video4LinuxFramegrabber< MEM_TYPE >::handleFrame( const Measurement::Timestamp t, boost::shared_ptr<Vision::Image>& pColorImage )
{

	// simplified solution - ignores the possibility of capturing bw images directly ..
	// needs to be extended, when v4l driver supports greyscale capturing.
	pColorImage = m_undistorter->undistort( pColorImage );


	if (m_autoGPUUpload){
		Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
		if (oclManager.isInitialized()) {
			//force upload to the GPU
			pColorImage->uMat();
		}
	}

	if ( m_colorOutPort.isConnected() )
	{
		// memcpy( pColorImage->channelSeq, "BGR", 4 );
		m_colorOutPort.send( Measurement::ImageMeasurement( t, pColorImage ) );
	}

	if ( m_outPort.isConnected() )
	{
		boost::shared_ptr< Vision::Image > pGreyImage;
		pGreyImage = pColorImage->CvtColor( CV_BGR2GRAY, 1 );
		m_outPort.send( Measurement::ImageMeasurement( t, pGreyImage ) );
	}
}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
{
	cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_MMAP > > ( "Video4LinuxFramegrabber" );
	cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_MMAP > > ( "DefaultFramegrabber" );
	//cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_READ > > ( "Video4LinuxFramegrabber" );
	//cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_USERPTR > > ( "Video4LinuxFramegrabber" );

	//cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_MMAP > > ( "Video4LinuxMmap" );
	//cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_READ > > ( "Video4LinuxRead" );
	//cf->registerComponent< Ubitrack::Drivers::Video4LinuxFramegrabber< IO_METHOD_USERPTR > > ( "Video4LinuxUserptr" );
}

