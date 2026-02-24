/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32c6.rom.newlib.ld for esp32c6
 *
 *
 * Generated from ./target/esp32c6/interface-esp32c6.yml md5sum 06c13e133e0743d09b87aba30d3e213b
 *
 * Compatible with ROM where ECO version equal or greater to 0.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. !!! BUT EDITED !!!
 * The file was originally generated for use with newlib, but it was split into
 * multiple files to make it compatible with picolibc.
 */

/***************************************
 Group newlib
 ***************************************/

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( strdup = 0x40000530 );
PROVIDE ( strndup = 0x40000554 );
PROVIDE ( rand = 0x40000590 );
PROVIDE ( srand = 0x40000594 );
PROVIDE ( rand_r = 0x4000058c );
PROVIDE ( atoi = 0x400005a0 );
PROVIDE ( atol = 0x400005a4 );
PROVIDE ( strtol = 0x400005a8 );
PROVIDE ( strtoul = 0x400005ac );
PROVIDE ( fflush = 0x400005b0 );
PROVIDE ( _fflush_r = 0x400005b4 );
PROVIDE ( _fwalk = 0x400005b8 );
PROVIDE ( _fwalk_reent = 0x400005bc );
PROVIDE ( __swbuf_r = 0x400005c8 );
/*******************************************/

/* Functions */
_isatty_r = 0x400004d4;
__smakebuf_r = 0x400005c0;
__swhatbuf_r = 0x400005c4;
__swbuf = 0x400005cc;
__swsetup_r = 0x400005d0;
toupper = 0x4000050c;
tolower = 0x40000510;
isalnum = 0x400004dc;
isalpha = 0x400004e0;
isdigit = 0x400004f0;
islower = 0x400004f4;
isspace = 0x40000504;
isupper = 0x40000508;
strcasecmp = 0x40000524;
strcoll = 0x4000053c;
strlwr = 0x40000548;
strncasecmp = 0x4000054c;
strupr = 0x4000056c;
