



#ifndef CPU_H
#define CPU_H




#define enter_critical() __asm__ volatile ("cpsid i");
#define leave_critical() __asm__ volatile ("cpsie i")

#endif //CPU_H
