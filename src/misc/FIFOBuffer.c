#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "FIFOBuffer.h"

#ifdef __cplusplus
 extern "C" {
#endif

 bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

 bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

 void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;
  
  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

 unsigned char fifo_pop(FIFOBuffer *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

 void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}


 void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

// todo, fix this so it actually displays the amount of data in the fifo
// buffer, not just the size allocated for the buffer
 size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

 bool fifo16_isempty(const FIFOBuffer16 *f) {
  return f->head == f->tail;
}

 bool fifo16_isfull(const FIFOBuffer16 *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

 void fifo16_push(FIFOBuffer16 *f, uint16_t c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

 uint16_t fifo16_pop(FIFOBuffer16 *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

 void fifo16_flush(FIFOBuffer16 *f) {
  f->head = f->tail;
}

 void fifo16_init(FIFOBuffer16 *f, uint16_t *buffer, uint16_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

 uint16_t fifo16_len(FIFOBuffer16 *f) {
  return (f->end - f->begin);
}

#ifdef __cplusplus
}
#endif
