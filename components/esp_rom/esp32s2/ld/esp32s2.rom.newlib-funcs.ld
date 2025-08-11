/**
 * These are the newlib functions present in ESP32-S2 ROM.
 * See also esp32s2.rom.newlib-data.ld for the list of .data/.bss symbols used by these functions.

 * Unlike other ROM functions which are exported using PROVIDE, which declare weak symbols,
 * newlib related functions are exported using assignment, which declares strong symbols.
 * This is done so that ROM functions are always used instead of the ones provided by libc.a.
 */

abs = 0x40000618;
bzero = 0x400078c8;
creat = 0x4000788c;
div = 0x40000620;
fclose = 0x4001a804;
fputwc = 0x40001864;
isalnum = 0x400078d8;
isalpha = 0x400078e8;
isascii = 0x4001aaec;
isblank = 0x400078f8;
iscntrl = 0x40007918;
isdigit = 0x40007930;
isgraph = 0x40007968;
islower = 0x40007948;
isprint = 0x40007980;
ispunct = 0x40007994;
isspace = 0x400079ac;
isupper = 0x400079c4;
labs = 0x40000648;
ldiv = 0x40000650;
longjmp = 0x400005a4;
memccpy = 0x4001ab00;
memchr = 0x4001ab24;
memcmp = 0x4001ab40;
memcpy = 0x4001aba8;
memmove = 0x4001acb0;
memrchr = 0x4001acec;
memset = 0x4001ad3c;
open = 0x400080c4;
qsort = 0x400006f4;
setjmp = 0x40000540;
setlocale = 0x40001c44;
strcasecmp = 0x40007b38;
strcasestr = 0x40007b7c;
strcat = 0x4001ad90;
strchr = 0x4001adb0;
strcmp = 0x40007be4;
strcoll = 0x40007ce8;
strcpy = 0x40007cfc;
strcspn = 0x4001adcc;
strlcat = 0x40007db8;
strlcpy = 0x4001adf8;
strlen = 0x40007e08;
strlwr = 0x40007e68;
strncasecmp = 0x40007e94;
strncat = 0x4001ae34;
strncmp = 0x4001ae64;
strncpy = 0x40007f20;
strnlen = 0x4001ae9c;
strrchr = 0x40008040;
strsep = 0x4000806c;
strspn = 0x4001aebc;
strstr = 0x4001aee8;
strtok_r = 0x4001af7c;
strupr = 0x40008084;
toascii = 0x4001af90;
tolower = 0x40008158;
toupper = 0x40008174;
wcrtomb = 0x400012f4;

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( rand_r = 0x40007af4 );
PROVIDE ( srand = 0x40007a24 );
PROVIDE ( strdup = 0x40007d84 );
PROVIDE ( _strdup_r = 0x40007d98 );
PROVIDE ( strndup = 0x40007fe8 );
PROVIDE ( _strndup_r = 0x40007ffc );
PROVIDE ( __ascii_mbtowc = 0x40007a04 );
PROVIDE ( __ascii_wctomb = 0x400018d0 );
PROVIDE ( __assert = 0x4001a430 );
PROVIDE ( __assert_func = 0x4001a408 );
PROVIDE ( _cleanup_r = 0x4001a480 );
PROVIDE ( _fclose_r = 0x4001a714 );
PROVIDE ( fflush = 0x40001bb8 );
PROVIDE ( _fflush_r = 0x40001b30 );
PROVIDE ( __fp_unlock_all = 0x4001a64c );
PROVIDE ( __fputwc = 0x40001770 );
PROVIDE ( _fputwc_r = 0x400017f8 );
PROVIDE ( _fwalk = 0x4001bcec );
PROVIDE ( _fwalk_reent = 0x4001bd24 );
PROVIDE ( __locale_ctype_ptr = 0x40001c2c );
PROVIDE ( __locale_ctype_ptr_l = 0x40001c24 );
PROVIDE ( __locale_mb_cur_max = 0x40001c0c );
PROVIDE ( _mbtowc_r = 0x400079e0 );
PROVIDE ( __sclose = 0x4001a700 );
PROVIDE ( __seofread = 0x4001a690 );
PROVIDE ( _setlocale_r = 0x40001bdc );
PROVIDE ( __sflush_r = 0x400019dc );
PROVIDE ( __sfmoreglue = 0x4001a4c8 );
PROVIDE ( __sfp = 0x4001a590 );
PROVIDE ( __sfp_lock_acquire = 0x4001a508 );
PROVIDE ( __sfp_lock_release = 0x4001a514 );
PROVIDE ( __sinit = 0x4001a538 );
PROVIDE ( __sinit_lock_acquire = 0x4001a520 );
PROVIDE ( __sinit_lock_release = 0x4001a52c );
PROVIDE ( __sread = 0x4001a660 );
PROVIDE ( __sseek = 0x4001a6cc );
PROVIDE ( __strtok_r = 0x4001af18 );
PROVIDE ( __swbuf = 0x4000167c );
PROVIDE ( __swbuf_r = 0x400015bc );
PROVIDE ( __swrite = 0x4001a698 );
PROVIDE ( _wcrtomb_r = 0x400012a0 );
PROVIDE ( _wctomb_r = 0x400018ac );
