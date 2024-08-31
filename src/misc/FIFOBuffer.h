#ifndef FIFOBUFFER_H

#define FIFOBUFFER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* An 8 bit FIFO buffer implementation */
typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
} FIFOBuffer;

bool fifo_isempty(const FIFOBuffer *f);

bool fifo_isfull(const FIFOBuffer *f);

void fifo_push(FIFOBuffer *f, unsigned char c);

unsigned char fifo_pop(FIFOBuffer *f);

void fifo_flush(FIFOBuffer *f);

void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size);

size_t fifo_len(FIFOBuffer *f);

/* A 16-bit implementation of the same FIFO buffer. */
typedef struct FIFOBuffer16
{
  uint16_t *begin;
  uint16_t *end;
  uint16_t * volatile head;
  uint16_t * volatile tail;
} FIFOBuffer16;

bool fifo16_isempty(const FIFOBuffer16 *f);

bool fifo16_isfull(const FIFOBuffer16 *f);

void fifo16_push(FIFOBuffer16 *f, uint16_t c);

uint16_t fifo16_pop(FIFOBuffer16 *f);

void fifo16_flush(FIFOBuffer16 *f);

void fifo16_init(FIFOBuffer16 *f, uint16_t *buffer, uint16_t size);

uint16_t fifo16_len(FIFOBuffer16 *f);

#ifdef __cplusplus
}
#endif

#endif
