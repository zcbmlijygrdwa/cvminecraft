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

    LRU cache = LRU( 3 /* capacity */ );
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
    return 0;
}
