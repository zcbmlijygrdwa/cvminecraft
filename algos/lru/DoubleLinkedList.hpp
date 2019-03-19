#ifndef DoubleLinkedList_HPP
#define DoubleLinkedList_HPP

template<class T>
class DoubleLinkedList
{
    public:
        int key;
        T val;
        DoubleLinkedList *prev, *next;

        DoubleLinkedList(int k, T v)
        {
            key = k;
            val = v;
            prev = NULL;
            next = NULL;
        }
};

#endif
