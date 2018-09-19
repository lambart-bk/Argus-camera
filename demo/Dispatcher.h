
#ifndef __CAMERA__
#define __CAMERA__

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>
#include "InitOnce.h"
#include "Error.h"


// Debug print macros.
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

using namespace Argus;
using namespace ArgusSamples;
using namespace EGLStream;

struct FrameData {
    int dma_fd;
    uint8_t *data_y;
    uint8_t *data_u;
    uint8_t *data_v;
    NvBufferParams para;
};

class Dispatcher 
{
public:
    #define MAX_CHANNEL   (6)

    void printtimestamp(unsigned long long timestamp, int devIndex);

    bool createSession(int devIndex);
    bool destroySession(int devIndex);
    bool shutdown();
    static Dispatcher &getInstance();
    int capture_frame(int devIndex);

    void framedataInit();
    void framedataMap(int devIndex);
    void framedataUmap(int devIndex);


    Argus::UniqueObj<Argus::CameraProvider> m_cameraProvider; ///< camera provider
    Argus::ICameraProvider *m_iCameraProvider;                ///< camera provider interface

    Argus::UniqueObj<Argus::CaptureSession> m_session[MAX_CHANNEL];        ///< Argus session

    Argus::UniqueObj<Argus::Request> m_request[MAX_CHANNEL];                ///< Argus request
    Argus::UniqueObj<Argus::OutputStream> m_outputStream[MAX_CHANNEL];     ///< Argus output stream
    Argus::UniqueObj<EGLStream::FrameConsumer> m_consumer[MAX_CHANNEL];

    struct FrameData m_framedata[MAX_CHANNEL];

private:
    bool m_initialized;
    std::vector<Argus::CameraDevice*> m_cameraDevices;
    bool initialize();
}; 


#endif
