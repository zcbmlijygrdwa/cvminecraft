#include <iostream>
#include "PointCloudVisualizer.hpp"
#include <thread>
#include <chrono>

using namespace std;


void changeColor(PointCloudVisualizer* pcv)
{

    for(int it = 1 ; it < 1000; it++)
    {
        cout<<"it = "<<it<<endl;

        vector<cv::Point3f> tPoints;
        vector<cv::Point3f> tColors;

        for(int i = 0 ; i < 10;i++)
        {
            for(int j = 0 ; j < 10 ; j++)
            {
                tPoints.push_back(cv::Point3f(i,j,it));
                tColors.push_back(cv::Point3f((i*i*i)%255,3*j*5,(i*j)%it));
            }
        }

        //pcv->setPoint(tPoints); 
        pcv->setColorPoint(tPoints,tColors); 

        pcv->commitPoints();

        cout<<"data update in thread"<<endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    cout<<"changeColor end"<<endl;
}

int main(int argn, char **argv)
{
    if (argn < 2)
    {
        cout << "Usage: " << endl << "./test [ G | C | GT | CT]" << endl;
        return 1;
    }

    bool camera_pov = (argv[1][0] == 'C');
    bool ifTestPoints = (argv[1][1] == 'T');

    PointCloudVisualizer pcv;
    if(ifTestPoints)
    {
        for(int i = 0 ; i < 10;i++)
        {
            for(int j = 0 ; j < 10 ; j++)
            {
                //pcv.addPoint(cv::Point3f(i,j,i+j));
                pcv.addColorPoint(cv::Point3f(i,j,i+j),i*i%255,3*j*5,i*j);
            }
        }

    }
    else
    {
        pcv.loadFromSource("../../../data/ply/bunny.ply");
    }
    pcv.commitPoints();
    pcv.setCamPov(camera_pov);
    //pcv.randomColor();
    pcv.setPointSize(5);

    if(false)
    {
        pcv.show();
    }
    else
    {
        std::thread my_thread(changeColor,&pcv);
        my_thread.detach();
        //my_thread.join();

        for(int i = 0 ; i < 1000 ; i++)
        {
            pcv.showOnce();  //display function needs to be called in main thread
            cout<<"refresh in main"<<endl;
        }    
    }
    return 0;
}
