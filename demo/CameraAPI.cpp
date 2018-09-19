
#include "CameraAPI.h"

void *capThread(void *arg)
{
    CameraAPI *pobj = (CameraAPI *)arg;
    static int index = 0;
    int camIndex = index;

    index++;

    while(pobj->capThreadState == CAP_IDLE)
        usleep(1);

    while(pobj->capThreadState == CAP_RUNNING)
    {
        pobj->dispatcher.capture_frame(pobj->camera_list[camIndex]);
    }

    return NULL;
}

CameraAPI::CameraAPI()
{
    int i = 0;
    capThreadState = CAP_IDLE;

    for(i = 0; i < CAM_NUM; i++)
    {
        dispatcher.createSession(camera_list[i]);

       /* if(0 != pthread_create(&capPid[i], NULL, capThread, this))
        {
            cout << "create cap thread error" << endl;
        }*/

        sleep(1);
    }

    //capThreadState = CAP_RUNNING;
}

CameraAPI::~CameraAPI()
{
    int i = 0;

    //capThreadState = CAP_DESTROY;

    for(i = 0; i < CAM_NUM; i++)
    {
	//pthread_join(capPid[i], NULL);
        dispatcher.destroySession(camera_list[i]);
    }

    dispatcher.shutdown();
}

int CameraAPI::CaptureFrame(struct FrameData *fd)
{
    int i;

    for(i = 0; i < CAM_NUM; i++)
    {
        frame_timestamp[i] = dispatcher.capture_frame(camera_list[i]); 
    }  
	
    usleep(16000);
    

    return 0;
}







