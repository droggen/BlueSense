#ifndef __BLOCK_H
#define __BLOCK_H

    // CODE DEFINES:
    #define ATOMIC_BLOCK(exitmode)   { exitmode cli();
    #define END_ATOMIC_BLOCK         }
   
    #define ATOMIC_RESTORESTATE      inline void __irestore(uint8_t *s) { SREG = *s; }         \
                                     uint8_t __isave __attribute__((__cleanup__(__irestore))) = SREG;
    #define ATOMIC_ASSUMEON          inline void __irestore(uint8_t *s) { sei(); *s = 0; }     \
                                     uint8_t __isave __attribute__((__cleanup__(__irestore))) = 0;


#endif