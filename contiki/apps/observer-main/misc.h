#ifndef MISC_H
#define MISC_H

#define WAIT_WHILE(cond) \
  do { \
    volatile int8_t i=0; \
    while(cond) i++; \
  } while(0)

void i2c_buffer_flush();

#endif /* MISC_H */
