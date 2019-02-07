#ifndef __BOOT_H
#define __BOOT_H

void boot_dbg(void);
void boot_bluetooth(void);
unsigned char bootloaderhook_dbg(unsigned char c);
unsigned char bootloaderhook_bluetooth(unsigned char c);
void boot_enterblink();

#endif