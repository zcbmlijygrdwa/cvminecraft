//#include <cstdlib>
//#include <ctime>
#include <sys/time.h>
//#include <iostream>
//#include <cstdio>


#include <stdio.h>

int main(int argc, char** argv)
{

    int* p1 = (int*)calloc(3,sizeof(int));
    int* p2 = (int*)calloc(3,sizeof(int));
    int* p3 = (int*)calloc(3,sizeof(int));

    struct timeval start, end;
    gettimeofday(&start, NULL);


    for(int i = 0 ; i<640*480;i++)
    {
        p1[0] = 0; p1[1] = 0; p1[2] = 0;
        p2[0] = 1; p2[1] = 0; p2[2] = 0;
        p3[0] = 0; p3[1] = 1; p3[2] = 0;


        p2[0] -= p1[0]; p2[1] -= p1[1]; p2[2] -= p1[2];
        p3[0] -= p1[0]; p3[1] -= p1[1]; p3[2] -= p1[2];

        //  i       j       k
        //p2[0]  p2[1]   p2[2]
        //p3[0]  p3[1]   p3[2]

        p1[0] = p2[1]*p3[2] - p2[2]*p3[1];
        p1[1] = p2[2]*p3[0] - p2[0]*p3[2];
        p1[2] = p2[0]*p3[1] - p2[1]*p3[0];
    }
    gettimeofday(&end, NULL);
    long long microseconds = (end.tv_sec - start.tv_sec)*1.0e6+ (end.tv_usec - start.tv_usec);
    printf("%d %d %d\n",p1[0],p1[1],p1[2]);
    printf( "time = %d microseconds\n", microseconds);

}
