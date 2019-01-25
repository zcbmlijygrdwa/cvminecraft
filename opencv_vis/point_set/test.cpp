#include <iostream>
#include "PointCloudVisualizer.hpp"

using namespace std;

int main(int argn, char **argv)
{
    if (argn < 2)
    {
        cout << "Usage: " << endl << "./test [ G | C ]" << endl;
        return 1;
    }

    bool camera_pov = (argv[1][0] == 'C');

    PointCloudVisualizer pcv;
    pcv.setSource("../../../data/ply/bunny.ply");
    pcv.setCamPov(camera_pov);
    pcv.show();

    return 0;
}
