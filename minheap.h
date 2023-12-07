#ifndef MINHEAP_H
#define MINHEAP_H

class MinHeap {
public:
    MinHeap();
    void init(int size);  // Initialize the min-heap with a given size
    void push(double key, int value);
    int pop();
    bool empty() const;
    MinHeap(int capacity);

private:
    double* keys; // Array to store weights of edges
    int* values;  // Array to store vertice Ids
    int capacity; // Maximum size of the heap
    int size;     // Current number of elements in the heap

    void heapifyUp(int index);
    void heapifyDown(int index);
    void swap(int i, int j);
};

#endif
