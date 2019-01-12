
#include "Dispatcher.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sstream>
// #include"opencv2/core/core.hpp"
// #include"opencv2/highgui/highgui.hpp"
// #include"opencv2/imgproc/imgproc.hpp"
#define EXIT_IF_NOT_OK(val,msg) \
        {if (val!=Argus::STATUS_OK) {printf("%s\n",msg); return EXIT_FAILURE;}}
#define EXIT_IF_NULL(val,msg)   \
        {if (!val) {printf("%s\n",msg); return EXIT_FAILURE;}}

#define debug false

class ScopedTimer
{
public:
    inline ScopedTimer(const std::string& name) :
        name(name)
    {
        gettimeofday(&begin, NULL);
    }


    inline ~ScopedTimer()
    {
        gettimeofday(&end, NULL);
        double elapsedTime;
        elapsedTime = (end.tv_sec - begin.tv_sec);
        elapsedTime += (end.tv_usec - begin.tv_usec) / 1000000.0; // us to s
#if debug
        fprintf(stderr, "%s: %.2f ms\n", name.c_str(), elapsedTime*1000.0);
#endif
    }


private:
    timeval begin;
    timeval end;
    std::string name;
};

void Dispatcher::printtimestamp(unsigned long long timestamp, int devIndex)
{
    timestamp /= 1000;
    std::cout << "device " << devIndex << ": " << timestamp << std::endl;
}

bool Dispatcher::initialize()
{
    if (m_initialized)
        return true;

    m_cameraProvider = Argus::UniqueObj<Argus::CameraProvider>(Argus::CameraProvider::create());
    m_iCameraProvider = Argus::interface_cast<Argus::ICameraProvider>(m_cameraProvider);
    if (!m_iCameraProvider)
        ORIGINATE_ERROR("Failed to create CameraProvider");

    // Get the camera devices
    m_iCameraProvider->getCameraDevices(&m_cameraDevices);
    if (m_cameraDevices.size() == 0)
    {
        PROPAGATE_ERROR(shutdown());
        ORIGINATE_ERROR("No cameras available");
    }

    framedataInit();

    m_initialized = true;

    return true;
}


Dispatcher &Dispatcher::getInstance()
{
    static InitOnce initOnce;
    static Dispatcher instance;

    if (initOnce.begin())
    {
        if (instance.initialize())
        {
            initOnce.complete();
        }
        else
        {
            initOnce.failed();
            REPORT_ERROR("Initalization failed");
        }
    }

    return instance;
}

bool Dispatcher::createSession(int devIndex)
{
#if debug
    std::cout<<"createSession   ..."<<devIndex<<std::endl;
#endif
    if (devIndex > m_cameraDevices.size())
        ORIGINATE_ERROR("Invalid device index");

    // create the new capture session
    m_session[devIndex] = Argus::UniqueObj<Argus::CaptureSession>(
        m_iCameraProvider->createCaptureSession(m_cameraDevices[devIndex]));
    if (!m_session[devIndex])
        ORIGINATE_ERROR("Failed to create CaptureSession");
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_session[devIndex]);
    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");

    // Create the OutputStream.
    PRODUCER_PRINT("Creating output stream\n");
    UniqueObj<OutputStreamSettings> streamSettings(iCaptureSession->createOutputStreamSettings());
    IOutputStreamSettings *iStreamSettings = interface_cast<IOutputStreamSettings>(streamSettings);
    if (!iStreamSettings)
        ORIGINATE_ERROR("Failed to get IOutputStreamSettings interface");

    iStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
  //  iStreamSettings->setEGLDisplay(renderer->getEGLDisplay());
    iStreamSettings->setResolution(Argus::Size(752, 480));
    iStreamSettings->setMetadataEnable(true);

    UniqueObj<OutputStream> captureStream(iCaptureSession->createOutputStream(streamSettings.get()));
    if (!captureStream)
        ORIGINATE_ERROR("Failed to create OutputStream");

    m_outputStream[devIndex].reset(captureStream.release());

    // Create the FrameConsumer.
    m_consumer[devIndex] = UniqueObj<FrameConsumer>(FrameConsumer::create(m_outputStream[devIndex].get()));
    if (!m_consumer[devIndex])
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    // Create capture request and enable output stream.
    UniqueObj<Request> request(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(request);
    if (!iRequest)
        ORIGINATE_ERROR("Failed to create Request");
    iRequest->enableOutputStream(m_outputStream[devIndex].get());

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    if (!iSourceSettings)
        ORIGINATE_ERROR("Failed to get ISourceSettings interface");
    iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9/60));

    // Submit capture requests.
    PRODUCER_PRINT("Starting repeat capture requests.\n");
    if (iCaptureSession->repeat(request.get()) != STATUS_OK)
        ORIGINATE_ERROR("Failed to start repeat capture request");

    IStream *iStream = interface_cast<IStream>(m_outputStream[devIndex].get());
    // Wait until the producer has connected to the stream.
    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (iStream->waitUntilConnected() != STATUS_OK)
        ORIGINATE_ERROR("Stream failed to connect.");
    CONSUMER_PRINT("Producer has connected; continuing.\n");

#if debug
    std::cout<<"createSession   end"<<devIndex<<std::endl;
#endif
    return true;
}

void Dispatcher::framedataInit()
{
    for(int i = 0; i < MAX_CHANNEL; i++)
    {
        m_framedata[i].dma_fd = -1;
        m_framedata[i].data_y = NULL;
        m_framedata[i].data_u = NULL;
        m_framedata[i].data_v = NULL;

        memset(&m_framedata[i].para, 0, sizeof(m_framedata[i].para));
    }
}





void Dispatcher::framedataMap(int devIndex)
{
#if debug
    printf("framedataMap...\n");
#endif

    int ret;
    int &fd = m_framedata[devIndex].dma_fd;
    NvBufferParams &para = m_framedata[devIndex].para;

    ret = NvBufferGetParams(fd, &para);
    //NvBufferGetParams(fd, &para) function will destroy m_framedata[devIndex+1]
    //so reset it
    for(int ii=devIndex+1;ii<MAX_CHANNEL;ii++)
    {
    	m_framedata[ii].dma_fd=-1;
    }

    if(ret < 0)
        {
	    printf("Failed to get a native buffer parameters\n");
    	    exit(0);
	}
#if debug
    printf("\n\n\npara.pitch************%d %d %d \n",para.pitch[0],para.pitch[1],para.pitch[2]);
    printf("para.height************%d %d %d \n",para.height[0],para.height[1],para.height[2]);
    printf("para.offset************%d %d %d \n\n\n\n",para.offset[0],para.offset[1],para.offset[2]);
#endif
    //Y
    size_t fsize = para.pitch[2] * para.height[2];
#if debug
    std::cout<<"fsize: "<<fsize<<"="<<para.pitch[2]<<"*"<<para.height[2]<<std::endl;
#endif
    void *temp= mmap(0, fsize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, para.offset[2]);
    if(temp== MAP_FAILED)	//(void *)-1 == MAP_FAILED
	{
		printf("y mmap failed\n");
		exit(0);
	}
    m_framedata[devIndex].data_y=(uint8_t *)temp;

/*    //U
    fsize = para.pitch[0] * para.height[0];
#if debug
    std::cout<<"fsize: "<<fsize<<"="<<para.pitch[0]<<"*"<<para.height[0]<<std::endl;
#endif
    temp= mmap(0, fsize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, para.offset[0]);
    if(temp== MAP_FAILED)	//(void *)-1 == MAP_FAILED
	{
		printf("u mmap failed\n");
		exit(0);
	}
    m_framedata[devIndex].data_u=(uint8_t *)temp;

    //V
    fsize = para.pitch[1] * para.height[1];
#if debug
    std::cout<<"fsize: "<<fsize<<"="<<para.pitch[1]<<"*"<<para.height[1]<<std::endl;
#endif
    temp= mmap(0, fsize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, para.offset[1]);
    if(temp== MAP_FAILED)	//(void *)-1 == MAP_FAILED
	{
		printf("v mmap failed\n");
		exit(0);
	}
    m_framedata[devIndex].data_v=(uint8_t *)temp;
*/
#if debug
    printf("framedataMap end\n");
#endif
}

void Dispatcher::framedataUmap(int devIndex)
{
    NvBufferParams &para = m_framedata[devIndex].para;
    
    size_t fsize = para.pitch[2] * para.height[2];
    munmap(m_framedata[devIndex].data_y, fsize);
//     fsize = para.pitch[1] * para.height[1];
//     munmap(m_framedata[devIndex].data_u, fsize);
//     fsize = para.pitch[2] * para.height[2];
//     munmap(m_framedata[devIndex].data_v, fsize);

    if (m_framedata[devIndex].dma_fd != -1)
        NvBufferDestroy(m_framedata[devIndex].dma_fd);
}

bool Dispatcher::destroySession(int devIndex)
{
#if debug
    std::cout<<"destroySession ..."<<devIndex<<std::endl;
#endif
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_session[devIndex]);
    // Stop the repeating request and wait for idle.
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

    // Destroy the output stream
    m_outputStream[devIndex].reset();

    framedataUmap(devIndex);
#if debug
    std::cout<<"destroySession end"<<devIndex<<std::endl;
#endif
}
#include <sys/time.h>
int Dispatcher::capture_frame(int devIndex)
{
#if debug
    std::cout << "Dispatcher::capture_frame "<<devIndex << std::endl;
#endif
    std::stringstream timerName;
    timerName << "capture_frame " << devIndex;
    ScopedTimer timer(timerName.str());

    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer[devIndex]);

    // Acquire a frame.
#if debug
    std::cout << "Acquire a frame. " << std::endl;
#endif
   // Argus::Status status;
   // const uint64_t FIVE_SECONDS_IN_NANOSECONDS = 5000000000;
    Argus::UniqueObj<Frame> frame(
        iFrameConsumer->acquireFrame());
    if(!frame)
	{
		printf("Failed to get IFrame interface.");
		exit(0);
	}

 
    EGLStream::IFrame *iFrame = Argus::interface_cast<IFrame>(frame);
    //EXIT_IF_NULL(iFrame, "Failed to get IFrame interface");
    if(!iFrame)
	{
		printf("Failed to get IFrame interface.");
		exit(0);
	}

    //printtimestamp(iFrame->getTime(), devIndex);
    
/*****************************/
    //write image to disk
/*    static int i=0;
    i++;
    EGLStream::Image *image = iFrame->getImage();
    EXIT_IF_NULL(image, "Failed to get Image from iFrame->getImage()");


    EGLStream::IImageJPEG *iImageJPEG = Argus::interface_cast<EGLStream::IImageJPEG>(image);
    EXIT_IF_NULL(iImageJPEG, "Failed to get ImageJPEG Interface");
    
    char file_name[30];
    sprintf(file_name,"./data/%d-%d.jpg",i,devIndex);
    Argus::Status status = iImageJPEG->writeJPEG(file_name);
    EXIT_IF_NOT_OK(status, "Failed to write JPEG");*/
/*************************/


    // Print out some capture metadata from the frame.
#if debug
    std::cout << "get IArgusCaptureMetadata interface.  " << std::endl;
#endif
    IArgusCaptureMetadata *iArgusCaptureMetadata = interface_cast<IArgusCaptureMetadata>(frame);
    if (!iArgusCaptureMetadata)
        {
		printf("Failed to get IArgusCaptureMetadata interface...\n");	
		exit(0);
	}


    CaptureMetadata *metadata = iArgusCaptureMetadata->getMetadata();
    if(!metadata)
        {
		printf("Failed to getMetadata..\n");	
		exit(0);
	}
    ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(metadata);
    if (!iMetadata)
        {
		printf("Failed to get ICaptureMetadata interface..\n");	
		exit(0);
	}	

//     printf("device %d: Sensor Timestamp: %llu\n", devIndex,
//                        static_cast<unsigned long long>(iMetadata->getSensorTimestamp() / 1000));

    
/*  //NvBuffer format ,not pitch linear !!! comment it
    // Print out image details, and map the buffers to read out some data.
    Image *image = iFrame->getImage();
    IImage *iImage = interface_cast<IImage>(image);
    IImage2D *iImage2D = interface_cast<IImage2D>(image);
    
    cv::Mat img_yuv420(480*3/2,752,CV_8UC1);
    
    int offset=0;
    for (uint32_t i = 0; i < iImage->getBufferCount(); i++)
    {
	const uint8_t *d = static_cast<const uint8_t*>(iImage->mapBuffer(i));
	if (!d)
	    ORIGINATE_ERROR("\tFailed to map buffer\n");
	
	Size size = iImage2D->getSize(i);
	std::cout<<"Stride: "<<iImage2D->getStride(i)<<std::endl;
	std::cout<<"\t"<<i<<":"<<size.width<<","<<size.height<<std::endl;
	int total=size.height*size.width;
	std::cout<<"sizeof d : "<<sizeof(d)<<std::endl;
	std::cout<<"d[0] d[8]: "<<(int)d[0]<<" "<<(int)d[8]<<std::endl;
	//memcpy(img_yuv420.data+offset,d,total);
	//offset+=total;
	 
// 	for(int r=0;r<size.height;r++)
// 	  for(int c=0;c<size.width;c++)
// 	  {
// 	    img_yuv420.at<unsigned char>(r,c)=d[c+r*size.width+1];
// 	  }
	
//	Size size = iImage2D->getSize(i);
// 	CONSUMER_PRINT("\tIImage(2D): "
// 			"buffer %u (%ux%u, %u stride), "
// 			"%d %d %d %d %d %d %d %d %d %d %d %d\n",
// 			i, size.width, size.height, iImage2D->getStride(i),
// 			d[52360], d[1], d[2], d[3], d[4], d[5],
// 			d[6], d[7], d[8], d[9], d[10], d[480]);
    }
    
//     cv::Mat img_rgb;
//     cv::cvtColor(img_yuv420,img_rgb,CV_YUV2BGR_I420);
    
//     cv::imshow("img_rgb",img_yuv420);
//     cv::waitKey(0);
*/




#if 1
    // Get the IImageNativeBuffer extension interface.
#if debug
    std::cout << "Get the IImageNativeBuffer extension interface. " << std::endl;
#endif
    EGLStream::Image *image = iFrame->getImage();
    if (!image)
    {
        printf("Failed to get Image from iFrame->getImage()");     
        exit(0);
    }

 /*   EGLStream::IImageJPEG *iImageJPEG = Argus::interface_cast<EGLStream::IImageJPEG>(image);
    EXIT_IF_NULL(iImageJPEG, "Failed to get ImageJPEG Interface");
    Argus::Status status = iImageJPEG->writeJPEG("/home/nvidia/Desktop/1.jpeg");
    EXIT_IF_NOT_OK(status, "Failed to write JPEG");*/

    NV::IImageNativeBuffer *iNativeBuffer =
        interface_cast<NV::IImageNativeBuffer>(image);
    if (!iNativeBuffer)
    {
        printf("IImageNativeBuffer not supported by Image..\n");     
        exit(0);
    }


    // If we don't already have a buffer, create one from this image.
    // Otherwise, just blit to our buffer.
#if debug
    for(int ii=0;ii<3;ii++)
    {
        printf("m_framedata[%d].dma_fd=%d\n",ii,m_framedata[ii].dma_fd);
    }
#endif
    
    if (m_framedata[devIndex].dma_fd == -1)
    {
        m_framedata[devIndex].dma_fd = iNativeBuffer->createNvBuffer(Argus::Size(752, 480),
                                                 NvBufferColorFormat_YUV420,
                                                 NvBufferLayout_Pitch);

        if (m_framedata[devIndex].dma_fd == -1)
	    {
		printf("\tFailed to create NvBuffer-%d\n",devIndex);
		
		exit(0);
	    }
#if debug
	printf(" create NvBuffer-%d\n",devIndex);
#endif

        framedataMap(devIndex);

    }
    else if (iNativeBuffer->copyToNvBuffer(m_framedata[devIndex].dma_fd) != STATUS_OK)
    {
        printf("Failed to copy frame to NvBuffer..%d\n",devIndex);
        
        exit(0);
    }

#endif

	return 1.0;
   // return iMetadata->getSensorTimestamp() / 1000000;
}

bool Dispatcher::shutdown()
{
    //destroySession();
#if debug
    printf("shutdown ...\n");
#endif
    m_cameraProvider.reset();
    m_cameraDevices.clear();
#if debug
    printf("shutdown end\n");
#endif
    return true;
}
