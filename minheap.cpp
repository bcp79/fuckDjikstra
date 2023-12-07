#include <cstdlib>
#include <cstdio>
#include "minheap.h"

MinHeap::MinHeap() : capacity(0), size(0), keys(nullptr), values(nullptr) {}

void MinHeap::init(int newSize) {
    // Deallocate existing memory
    free(keys);
    free(values);

    // Allocate memory for keys and values
    keys = (double*)malloc(newSize * sizeof(double));
    values = (int*)malloc(newSize * sizeof(int));

    // Update capacity and size
    capacity = newSize;
    size = 0;
}

void MinHeap::push(double key, int value) {
    if (size >= capacity)
        return;

    size++;
    int index = size - 1;
    keys[index] = key;
    values[index] = value;
    heapifyUp(index);
}

int MinHeap::pop() {
    if (empty())
        return -1;

    int root = values[0];
    values[0] = values[size - 1];
    keys[0] = keys[size - 1];
    size--;
    heapifyDown(0);
    return root;
}

bool MinHeap::empty() const {
    return size == 0;
}

void MinHeap::heapifyUp(int index) {
    while (index > 0 && keys[index] < keys[(index - 1) / 2]) {
        swap(index, (index - 1) / 2);
        index = (index - 1) / 2;
    }
}

void MinHeap::heapifyDown(int index) {
    int smallest = index;
    int leftChild = 2 * index + 1;
    int rightChild = 2 * index + 2;

    if (leftChild < size && keys[leftChild] < keys[smallest])
        smallest = leftChild;

    if (rightChild < size && keys[rightChild] < keys[smallest])
        smallest = rightChild;

    if (smallest != index) {
        swap(index, smallest);
        heapifyDown(smallest);
    }
}

void MinHeap::swap(int i, int j) {
    // Swap values without using std::swap
    double tempKey = keys[i];
    int tempValue = values[i];
    keys[i] = keys[j];
    values[i] = values[j];
    keys[j] = tempKey;
    values[j] = tempValue;
}

MinHeap::MinHeap(int newCapacity) : capacity(newCapacity), size(0), keys(nullptr), values(nullptr) {
    // Allocate memory for keys and values
    keys = (double*)malloc(capacity * sizeof(double));
    values = (int*)malloc(capacity * sizeof(int));
}
