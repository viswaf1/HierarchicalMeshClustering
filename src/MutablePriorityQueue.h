#ifndef MHEAP_H
#define MHEAP_H

#include <vector>
#include <unordered_map>

template <class T, class COMP>
class MutableHeap
{
public:
    MutableHeap():m_heapSize(0){};
    
    ~MutableHeap()
    {
        for(auto ptrIter = m_ptrIndexMap.begin();
            ptrIter != m_ptrIndexMap.end(); ptrIter++)
        {
            delete ptrIter->first;
        }
    }

    T* Insert(T val)
    {
        T* newValPtr = new T;
        *newValPtr = val;
        heapInsert(newValPtr);
        return newValPtr;
    }

    bool PopTop(T& val)
    {
        if(m_heap.size() < 1)
        {return false;}

        T* valPtr;
        if(!heapExtract(0, valPtr))
        {return false;}

        val = *valPtr;
        delete valPtr;

        return true;
    }

    bool PeekTop(T& val)
    {
        if(m_heap.size() < 1 )
        {
            return false;
        }
        val = *(m_heap[0]);
        return true;
    }

    bool UpdateVal(T* valPtr)
    {   
        auto mapIter = m_ptrIndexMap.find(valPtr);
        if(mapIter == m_ptrIndexMap.end())
        {
            return false;
        }
        int valInd = mapIter->second;
        // heapify(valInd);
        T* tempVal;
        heapExtract(valInd, tempVal);
        heapInsert(tempVal);
        return true;
    }

    uint size()
    {
        return m_heap.size();
    }

private:
    int m_heapSize;
    std::vector<T*> m_heap;
    std::unordered_map<T*,int, std::hash<T*>> m_ptrIndexMap;

    COMP m_comp;
    
    int parentIndex(int k)
    { return (k-1)/2;}
    int leftIndex(int k)
    {return 2*k + 1;}
    int rightIndex(int k)
    {return 2*k + 2;}

    void heapInsert(T* valPtr)
    {
        m_heap.push_back(nullptr);
        int k = m_heap.size() - 1;
        while( k > 0 && parentIndex(k) >=0 &&
             m_comp(*valPtr, *(m_heap[parentIndex(k)])) )
        {
            m_heap[k] = m_heap[parentIndex(k)];
            m_ptrIndexMap[m_heap[parentIndex(k)]] = k;
            k = parentIndex(k);
        } 
        m_heap[k] = valPtr;
        m_ptrIndexMap[valPtr] = k;
    }

    void heapify(int k)
    {
        int left,right;
        left = leftIndex(k);
        right = rightIndex(k);
        int largest;
        if(left < m_heap.size() &&  m_comp(*(m_heap[left]), *(m_heap[k]) ) )
        {
            largest = left;
        }
        else
        {
            largest = k;
        }

        if(right < m_heap.size() && m_comp( *(m_heap[right]), *(m_heap[largest]) ) )
        {
            largest = right;
        }

        if(largest != k)
        {
            T* temp = m_heap[k];
            m_heap[k] = m_heap[largest];
            m_ptrIndexMap[m_heap[k]] = k;
            m_heap[largest] = temp;
            m_ptrIndexMap[temp] = largest;
            heapify(largest);
        }
    }

    bool heapExtract(int ind, T* &outValPtr)
    {   
        int heapSize = m_heap.size();
        if(heapSize < 1)
        {
            return false;
        }
        outValPtr = m_heap[ind];
        // T* outValPtr = m_heap[ind];
        m_heap[ind] = m_heap[heapSize-1];
        m_heap.pop_back();

        m_ptrIndexMap.erase(outValPtr);
        //delete valPtr;

        heapify(ind);
        return true;
    }


};

#endif