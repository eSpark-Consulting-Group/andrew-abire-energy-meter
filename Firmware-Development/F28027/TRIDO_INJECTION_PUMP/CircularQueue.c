//20210822
#include "CircularQueue.h"

void _enQueue(T value, struct circularQueue *self){
    if(((self->last == QUEUE_SIZE-1 && self->first == 0) || (self->last == self->first-1))){//Full
        self->error = CQERROR_OVERFLOW;
    }else if(self->first == -1){//empty
        self->first = self->last = 0;
        self->queue[self->last] = value;
    }else if(self->last == QUEUE_SIZE-1 && self->first != 0){//jump to beginning
        self->last = 0;
        self->queue[self->last] = value;
    }else{//normal
        ++(self->last);
        self->queue[self->last] = value;
    }

    //traffic monitor
    if(self->occupied(self) > self->trafficMax){
        self->trafficMax = self->occupied(self);
    }
}





T _deQueue(struct circularQueue *self){
    //check if it is empty
    if(self->first == -1){
        return 0;
    }

    //get data
    T data = self->queue[self->first];

    //update index
    if(self->first == self->last){//empty
        self->first = -1;
    }else if(self->first == QUEUE_SIZE-1){//"first" index located in last element, jump to beginning
        self->first = 0;
    }else{
        ++(self->first);
    }

    return data;
}





int _empty(struct circularQueue *self){
    return self->first == -1;
}





int _occupied(struct circularQueue *self){//return number of occupied buffer
    if(self->first == -1)
        return 0;
    else
        return ((self->last - self->first + self->size) % self->size) + 1;
}





//constructor
//struct circularQueue CQ = {.size = QUEUE_SIZE, .first = -1, .last = -1, .error = 0, .enQueue = enQueue, .deQueue = deQueue, .empty = empty};
struct circularQueue construct_circularQueue(){
    //object instantiation
    struct circularQueue cQueue;
    cQueue.size = QUEUE_SIZE;
    cQueue.first = cQueue.last = -1;
    cQueue.trafficMax = 0;
    cQueue.error = CQERROR_NONE;
    cQueue.enQueue = *_enQueue;
    cQueue.deQueue = *_deQueue;
    cQueue.empty = *_empty;
    cQueue.occupied = *_occupied;
    return cQueue;//do member copy outside
};

