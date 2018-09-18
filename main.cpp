
#include "CameraAPI.h"

#define FRAMECOUNT      (5)
using namespace std;
int main(int argc, char * argv[])
{
    int i = FRAMECOUNT;
    
    int temp[6]={0,0,0,0,0,0};
    CameraAPI camapi;
    //IMUThread imut(20); // run 20s

    while(i--)
    {
		camapi.CaptureFrame();
		camapi.publishImage(0);

		if(i<(FRAMECOUNT-1))
		{
			for(int j=0;j<camapi.camNum;j++)
		 	{
				int diff=camapi.frame_timestamp[j]-temp[j];
				cout<<"\t\t\t\tdevice: "<<j<<" : "<<diff<<"ms\n";
			}	
			//cout<<"\n";
		}
        
		for(int j=0;j<camapi.camNum;j++)
	 	{
			temp[j]=camapi.frame_timestamp[j];
	    }	
	
	
    }

    return 0;
}
