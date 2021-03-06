#ifndef LRU_HPP
#define LRU_HPP

#include <iostream>
#include <map>
#include "DoubleLinkedList.hpp"

using namespace std;

template<class T>
class LRU
{
    public:
        DoubleLinkedList<T> *head, *end, *myList;
        map<int,DoubleLinkedList<T>*> myMap;
        uint capacity;
        LRU(uint c)
        {
            capacity = c;
            myList = NULL;
            head = NULL;
            end = NULL;
        }

        void reset()
        {
            myList = NULL;
            head = NULL;
            end = NULL;
        }

        void touch(int key)
        {

            if(key==head->key)
                return;
            else if(key==end->key)
            {
                DoubleLinkedList<T>* temp = end;
                end = end->prev;

                head->prev = temp;
                temp->next = head;
                head = temp;
            }
            else
            {
                DoubleLinkedList<T>* temp = myMap[key];

                if(temp->prev)
                    temp->prev->next = temp->next;
                if(temp->next)
                    temp->next->prev = temp->prev;

                temp->prev = NULL;
                temp->next = head;
                head->prev = temp;
                head = temp;
            } 
        }

        int get(int key)
        {
            if(myMap.find(key)!=myMap.end())
            {
                touch(key);
                return myMap[key]->val;
            }
            else
            {
                return -1;
            }
        }


        void put(int key, T value)
        {
            //check if empty
            if(myMap.size()==0)
            {
                cout<<"init pushing: "<<value<<endl;
                DoubleLinkedList<T>* temp = new DoubleLinkedList<T>(key, value);

                head = temp;
                end = temp;
                myMap[key] = temp;

            }
            else
            {
                if(myMap.find(key)!=myMap.end())
                {
                    myMap[key]->val = value;
                    touch(key);
                }
                else
                {
                    cout<<"pushing: "<<value<<endl;
                    DoubleLinkedList<T>* temp = new DoubleLinkedList<T>(key, value);
                    myMap[key] = temp;

                    temp->next = head;
                    head->prev = temp;

                    head = temp;
                }

                //check if full
                if(myMap.size()>capacity)
                {
                    myMap.erase(end->key);
                    end = end->prev;

                    cout<<"evicts key: "<<end->next->key<<endl;
                    delete(end->next);
                    end->next = NULL;

                    if(myMap.size()==0)
                    {
                        reset();
                    }
                }
            }
        }
};

#endif
