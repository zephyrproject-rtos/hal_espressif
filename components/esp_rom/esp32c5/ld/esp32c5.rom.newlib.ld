/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32c5.rom.newlib.ld for esp32c5
 *
 *
 * Generated from ./target/esp32c5/interface-esp32c5.yml md5sum 4dfe6ccc311ca3575012c23e3753353c
 *
 * Compatible with ROM where ECO version equal or greater to 2.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. DO NOT EDIT.
 */

/***************************************
 Group newlib
 ***************************************/

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( strdup = 0x40000540 );
PROVIDE ( strndup = 0x40000564 );
PROVIDE ( rand = 0x400005a0 );
PROVIDE ( srand = 0x400005a4 );
PROVIDE ( rand_r = 0x4000059c );
PROVIDE ( atoi = 0x400005b0 );
PROVIDE ( atol = 0x400005b4 );
PROVIDE ( strtol = 0x400005b8 );
PROVIDE ( strtoul = 0x400005bc );
PROVIDE ( fflush = 0x400005c0 );
PROVIDE ( _fflush_r = 0x400005c4 );
PROVIDE ( _fwalk = 0x400005c8 );
PROVIDE ( _fwalk_reent = 0x400005cc );
PROVIDE ( __swbuf_r = 0x400005d8 );
/*******************************************/

/* Functions */
_isatty_r = 0x400004e4;
__smakebuf_r = 0x400005d0;
__swhatbuf_r = 0x400005d4;
__swbuf = 0x400005dc;
__swsetup_r = 0x400005e0;
toupper = 0x4000051c;
tolower = 0x40000520;
isalnum = 0x400004ec;
isalpha = 0x400004f0;
isdigit = 0x40000500;
islower = 0x40000504;
isspace = 0x40000514;
isupper = 0x40000518;
strcasecmp = 0x40000534;
strcoll = 0x4000054c;
strlwr = 0x40000558;
strncasecmp = 0x4000055c;
strupr = 0x4000057c;
