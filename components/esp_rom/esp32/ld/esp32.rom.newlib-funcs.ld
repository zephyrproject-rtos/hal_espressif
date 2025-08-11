/* These are the newlib functions present in ESP32 ROM.
   They should not be used when compiling with PSRAM cache workaround enabled.
   See also esp32.rom.newlib-data.ld for the list of .data/.bss symbols
   used by these functions, and esp32.rom.newlib-nano.ld for "nano" versions
   of printf/scanf family of functions.

   Unlike other ROM functions which are exported using PROVIDE, which declares
   weak symbols, newlib related functions are exported using assignment,
   which declares strong symbols. This is done so that ROM functions are always
   used instead of the ones provided by libc.a.

   Time functions were moved to the esp32.rom.newlib-time.ld file.
 */

abs = 0x40056340;
bzero = 0x4000c1f4;
creat = 0x40000e8c;
div = 0x40056348;
fclose = 0x400020ac;
fputwc = 0x40058ea8;
isalnum = 0x40000f04;
isalpha = 0x40000f18;
isascii = 0x4000c20c;
isblank = 0x40000f2c;
iscntrl = 0x40000f50;
isdigit = 0x40000f64;
isgraph = 0x40000f94;
islower = 0x40000f78;
isprint = 0x40000fa8;
ispunct = 0x40000fc0;
isspace = 0x40000fd4;
isupper = 0x40000fe8;
itoa = 0x400566b4;
labs = 0x40056370;
ldiv = 0x40056378;
longjmp = 0x400562cc;
memccpy = 0x4000c220;
memchr = 0x4000c244;
memcmp = 0x4000c260;
memcpy = 0x4000c2c8;
memmove = 0x4000c3c0;
memrchr = 0x4000c400;
memset = 0x4000c44c;
qsort = 0x40056424;
setjmp = 0x40056268;
strcasecmp = 0x400011cc;
strcasestr = 0x40001210;
strcat = 0x4000c518;
strchr = 0x4000c53c;
strcmp = 0x40001274;
strcoll = 0x40001398;
strcpy = 0x400013ac;
strcspn = 0x4000c558;
strlcat = 0x40001470;
strlcpy = 0x4000c584;
strlen = 0x400014c0;
strlwr = 0x40001524;
strncasecmp = 0x40001550;
strncat = 0x4000c5c4;
strncmp = 0x4000c5f4;
strncpy = 0x400015d4;
strnlen = 0x4000c628;
strrchr = 0x40001708;
strsep = 0x40001734;
strspn = 0x4000c648;
strstr = 0x4000c674;
strtok_r = 0x4000c70c;
strupr = 0x4000174c;
toascii = 0x4000c720;
tolower = 0x40001868;
toupper = 0x40001884;
ungetc = 0x400590f4;
utoa = 0x40056258;
wcrtomb = 0x40058920;
_wcrtomb_r = 0x400588d8;

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( atoi = 0x400566c4 );
PROVIDE ( _atoi_r = 0x400566d4 );
PROVIDE ( atol = 0x400566ec );
PROVIDE ( _atol_r = 0x400566fc );
PROVIDE ( rand = 0x40001058 );
PROVIDE ( rand_r = 0x400010d4 );
PROVIDE ( srand = 0x40001004 );
PROVIDE ( strdup = 0x4000143c );
PROVIDE ( _strdup_r = 0x40001450 );
PROVIDE ( strndup = 0x400016b0 );
PROVIDE ( _strndup_r = 0x400016c4 );
PROVIDE ( strtol = 0x4005681c );
PROVIDE ( _strtol_r = 0x40056714 );
PROVIDE ( strtoul = 0x4005692c );
PROVIDE ( _strtoul_r = 0x40056834 );
PROVIDE ( __ascii_wctomb = 0x40058ef0 );
PROVIDE ( _cleanup = 0x40001df8 );
PROVIDE ( _cleanup_r = 0x40001d48 );
PROVIDE ( __dummy_lock = 0x4000c728 );
PROVIDE ( __dummy_lock_try = 0x4000c730 );
PROVIDE ( __env_lock = 0x40001fd4 );
PROVIDE ( __env_unlock = 0x40001fe0 );
PROVIDE ( _fclose_r = 0x40001fec );
PROVIDE ( fflush = 0x40059394 );
PROVIDE ( _fflush_r = 0x40059320 );
PROVIDE ( _findenv_r = 0x40001f44 );
PROVIDE ( __fp_lock_all = 0x40001f1c );
PROVIDE ( __fp_unlock_all = 0x40001f30 );
PROVIDE ( __fputwc = 0x40058da0 );
PROVIDE ( _fputwc_r = 0x40058e4c );
PROVIDE ( _fwalk = 0x4000c738 );
PROVIDE ( _fwalk_reent = 0x4000c770 );
PROVIDE ( _getenv_r = 0x40001fbc );
PROVIDE ( __itoa = 0x40056678 );
PROVIDE ( __sccl = 0x4000c498 );
PROVIDE ( __sclose = 0x400011b8 );
PROVIDE ( __seofread = 0x40001148 );
PROVIDE ( __sflush_r = 0x400591e0 );
PROVIDE ( __sfmoreglue = 0x40001dc8 );
PROVIDE ( __sfp = 0x40001e90 );
PROVIDE ( __sfp_lock_acquire = 0x40001e08 );
PROVIDE ( __sfp_lock_release = 0x40001e14 );
PROVIDE ( __sinit = 0x40001e38 );
PROVIDE ( __sinit_lock_acquire = 0x40001e20 );
PROVIDE ( __sinit_lock_release = 0x40001e2c );
PROVIDE ( __sread = 0x40001118 );
PROVIDE ( __sseek = 0x40001184 );
PROVIDE ( __strtok_r = 0x4000c6a8 );
PROVIDE ( __submore = 0x40058f3c );
PROVIDE ( __swbuf = 0x40058cb4 );
PROVIDE ( __swbuf_r = 0x40058bec );
PROVIDE ( __swrite = 0x40001150 );
PROVIDE ( _ungetc_r = 0x40058fa0 );
PROVIDE ( __utoa = 0x400561f0 );
PROVIDE ( _wctomb_r = 0x40058f14 );
