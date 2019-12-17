/// @file
/// Based on compiler-compat-mchip_xc16.h
/// Compatibility with Clang-style build tools.
/// Stubs out builtin functions so the code is compilable, but don't expect it to work right at
/// runtime!!!

#ifndef XC16_CLANG_TOOLS_STUBS_H
#define XC16_CLANG_TOOLS_STUBS_H

#if (defined __XC16 || defined __XC16__ || defined __C30 || defined __C30__)
#error "This file is only for compatibility with clang-style build tools"
#endif

#define __builtin_clrwdt(x)

#define inline
#define __inline
#define __attribute__(x)
#define asm(x)
#define __extension__
#define __asm__(x)
#define __pack_upper_byte
#define __eds__
#define __psv__
#define __prog__
#define __pmp__
#define __external__
#define __complex__
#define __real__
#define __imag__

/* CCI */
#define __at(x)
#define __section(x)
#define __bank(x)
#define __align(x)
#define __interrupt(x)
#define asm(x)
#define __far
#define __near
#define __persistent
#define __xdata
#define __ydata
#define __eeprom
#define __pack
#define __deprecate

/* built-in Function List */

int __builtin_addab(int Accum_a, int Accum_b);
int __builtin_add(int Accum, int value, const int shift);
void __builtin_btg(unsigned int *, unsigned int n);
int __builtin_clr(void);
int __builtin_clr_prefetch(
    int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr, int *AWB, int AWB_accum);
void __builtin_write_CRYOTP(void);
int __builtin_dataflashoffset(unsigned int *var);
void __builtin_disable_interrupts(void);
void __builtin_disi(int disi_count);
unsigned int __builtin_divf(unsigned int num, unsigned int den);
signed int __builtin_divmodsd(signed long dividend, signed int divisor, signed int *remainder);
unsigned int
__builtin_divmodud(unsigned long dividend, unsigned int divisor, unsigned int *remainder);
int __builtin_divsd(const long num, const int den);
unsigned int __builtin_divud(const unsigned long num, const unsigned int den);
unsigned int __builtin_dmapage(const void *p);
unsigned int __builtin_dmaoffset(const void *p);
int __builtin_ed(int sqr, int **xptr, int xincr, int **yptr, int yincr, int *distance);
int __builtin_edac(int Accum, int sqr, int **xptr, int xincr, int **yptr, int yincr, int *distance);
unsigned int __builtin_edspage(const void *p);
unsigned int __builtin_edsoffset(const void *p);
void __builtin_enable_interrupts(void);
int __builtin_fbcl(int value);
unsigned int __builtin_get_isr_state(void);
int __builtin_lac(int value, int shift);
int __builtin_mac(
    int Accum, int a, int b, int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr,
    int *AWB, int AWB_accum);
signed int __builtin_modsd(signed long dividend, signed int divisor);
unsigned int __builtin_modud(unsigned long dividend, unsigned int divisor);
int __builtin_movsac(
    int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr, int *AWB, int AWB_accum);
int __builtin_mpy(int a, int b, int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr);
int __builtin_mpyn(
    int a, int b, int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr);
int __builtin_msc(
    int Accum, int a, int b, int **xptr, int *xval, int xincr, int **yptr, int *yval, int yincr,
    int *AWB, int AWB_accum);
signed long __builtin_mulss(const signed int p0, const signed int p1);
signed long __builtin_mulsu(const signed int p0, const unsigned int p1);
signed long __builtin_mulus(const unsigned int p0, const signed int p1);
unsigned long __builtin_muluu(const unsigned int p0, const unsigned int p1);
void __builtin_nop(void);
unsigned int __builtin_psvpage(const void *p);
unsigned int __builtin_psvoffset(const void *p);
int __builtin_sac(int value, int shift);
int __builtin_sacr(int value, int shift);
unsigned long __builtin_section_begin(const char *section_name);
unsigned long __builtin_section_end(const char *section_name);
unsigned long __builtin_section_size(const char *section_name);
void __builtin_set_isr_state(unsigned int state);
int __builtin_sftac(int Accum, int shift);
void __builtin_software_breakpoint(void);
int __builtin_subab(int Accum_a, int Accum_b);
unsigned long __builtin_tbladdress(const void *p);
unsigned int __builtin_tblpage(const void *p);
unsigned int __builtin_tbloffset(const void *p);
unsigned int __builtin_tblrdh(unsigned int offset);
unsigned int __builtin_tblrdl(unsigned int offset);
void __builtin_tblwth(unsigned int offset, unsigned int data);
void __builtin_tblwtl(unsigned int offset, unsigned int data);
void __builtin_write_NVM(void);
void __builtin_write_NVM_secure(unsigned int key1, unsigned int key2);
void __builtin_write_PWMSFR(
    volatile unsigned int *PWM_sfr, unsigned int value, volatile unsigned int *PWM_KEY);
void __builtin_write_RTCWEN(void);
void __builtin_write_OSCCONL(unsigned char value);
void __builtin_write_OSCCONH(unsigned char value);

#endif