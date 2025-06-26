//20210822
#ifndef CIRCULARQUEUE_H_
#define CIRCULARQUEUE_H_
/*
 * Ring Buffer or Circular Queue:
 * - linear data structure FIFO
 * - last buffer is connected back to the first buffer to make a circle
 * - if queue is full, the overflowed input will not be served
 * - time complexity O(1) for enQueue, deQuene, empty
 *
 * https://www.geeksforgeeks.org/circular-queue-set-1-introduction-array-implementation/
 */


/*
 * Initialization
 */
//define buffer size
#define QUEUE_SIZE 50
//define buffer type
#define T char//set T as Uint8
//typedef Uint8 T;
//template <typename T>;//todo: struct template



typedef enum { CQERROR_NONE = 0, CQERROR_OVERFLOW} CQError;//todo: use a better structure for error code
//typedef struct circularQueue circularQueue;//give type "struct circularQueue" an alias of "circularQueue"

//forward declaration for the instantiation and construction
struct circularQueue;
void _enQueue(T value, struct circularQueue *self);
T _deQueue(struct circularQueue *self);
int _empty(struct circularQueue *self);
int _occupied(struct circularQueue *self);//return number of occupied buffer
struct circularQueue construct_circularQueue();


struct circularQueue{
    int size;//todo: compile time deducible self.size-1
    int first;
    int last;
    int trafficMax;//todo:find max occupied number of buffer
    CQError error;
    void (*enQueue)(T value, struct circularQueue *self);
    T (*deQueue)(struct circularQueue *self);
    int (*empty)(struct circularQueue *self);
    int (*occupied)(struct circularQueue *self);//todo:find max occupied
    T queue[QUEUE_SIZE];
};
//} cq = {//instantiation and construction named "circularQueue", this is allowed because "circularQueue" in the struct is defined in struct namespace, while the "circularQueue" here will be defined in the global namespace
//        .size = QUEUE_SIZE,
//        .first = -1,
//        .last = -1,
//        .trafficMax = 0,
//        .error = CQERROR_NONE,
//        ._enQueue = _enQueue,
//        ._deQueue = _deQueue,
//        ._empty = _empty,
//        ._occupied = _occupied
//};




////implement function argument default value in C, "&circularQueue" is the default value for the argument
//#define enQueue(value,...) enQueue(value, &cq)//_enQueue(value, (pcq, ##__VA_ARGS__))
//#define empty(...) empty(&cq)
//#define deQueue(...) deQueue(&cq)


#endif /* CIRCULARQUEUE_H_ */
