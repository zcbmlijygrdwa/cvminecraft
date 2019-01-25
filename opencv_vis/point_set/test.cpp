#include <iostream>
#include "PointCloudVisualizer.hpp"

using namespace std;

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
                pcv.addPoint(cv::Point3f(i,j,i+j));
            }
        }
    
        pcv.commitPoints();
    }
    else
    {
        pcv.setSource("../../../data/ply/bunny.ply");
    }
    pcv.setCamPov(camera_pov);
    pcv.randomColor();
    pcv.setPointSize(5);
    pcv.show();

    return 0;
}
