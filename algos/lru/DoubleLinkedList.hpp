#ifndef DoubleLinkedList_HPP
#define DoubleLinkedList_HPP

class DoubleLinkedList
{
    public:
        int key;
        int val;
        DoubleLinkedList *prev, *next;

        DoubleLinkedList(int k, int v)
        {
            key = k;
            val = v;
            prev = NULL;
            next = NULL;
        }
};

#endif
