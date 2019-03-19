#include <iostream>
#include "LRU.hpp"

using namespace std;

int main(int argc, char** argv)
{

    //LRU cache = LRU( 2 /* capacity */ );
    //cache.put(1, 1);
    //cache.put(2, 2);
    //cout<<cache.get(1)<<endl;       // returns 1
    //cache.put(3, 3);    // evicts key 2
    //cout<<cache.get(2)<<endl;       // returns -1
    //cache.put(4, 4);    // evicts key 1
    //cout<<cache.get(1)<<endl;       // returns -1
    //cout<<cache.get(3)<<endl;       // returns -1
    //cout<<cache.get(4)<<endl;       // returns -1



    //object is int
    LRU<int> cache = LRU<int>( 3 /* capacity */ );
    cache.put(1, 1);
    cache.put(2, 2);
    cache.put(3, 3);
    cache.put(4, 4);
    cache.get(4);
    cache.get(3);
    cache.get(2);
    cache.get(1);
    cache.put(5, 5);
    cout<<cache.get(1)<<endl;
    cout<<cache.get(2)<<endl;
    cout<<cache.get(3)<<endl;
    cout<<cache.get(4)<<endl;
    cout<<cache.get(5)<<endl;
    


    //object is double
    LRU<double> cache_d = LRU<double>( 3 /* capacity */ );
    cache_d.put(1, 1.2);
    cache_d.put(2, 2.5);
    cache_d.put(3, 3.34);
    cache_d.put(4, 4.2);
    cache_d.get(4);
    cache_d.get(3);
    cache_d.get(2);
    cache_d.get(1);
    cache_d.put(5, 5.0);
    cout<<cache_d.get(1)<<endl;
    cout<<cache_d.get(2)<<endl;
    cout<<cache_d.get(3)<<endl;
    cout<<cache_d.get(4)<<endl;
    cout<<cache_d.get(5)<<endl;
    return 0;
}
