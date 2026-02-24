/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32h2.rom.newlib.ld for esp32h2
 *
 *
 * Generated from ./target/esp32h2/interface-esp32h2.yml md5sum c0ad4e113e5b29bb9d799f10f03edbc1
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

/* Functions */
_isatty_r = 0x400004cc;
/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( strdup = 0x40000528 );
PROVIDE ( strndup = 0x4000054c );
PROVIDE ( rand_r = 0x40000584 );
PROVIDE ( rand = 0x40000588 );
PROVIDE ( srand = 0x4000058c );
PROVIDE ( atoi = 0x40000598 );
PROVIDE ( atol = 0x4000059c );
PROVIDE ( strtol = 0x400005a0 );
PROVIDE ( strtoul = 0x400005a4 );
PROVIDE ( fflush = 0x400005a8 );
PROVIDE ( _fflush_r = 0x400005ac );
PROVIDE ( _fwalk = 0x400005b0 );
PROVIDE ( _fwalk_reent = 0x400005b4 );
__smakebuf_r = 0x400005b8;
__swhatbuf_r = 0x400005bc;
PROVIDE ( __swbuf_r = 0x400005c0 );
__swbuf = 0x400005c4;
__swsetup_r = 0x400005c8;
toupper = 0x40000504;
tolower = 0x40000508;
isalnum = 0x400004d4;
isalpha = 0x400004d8;
isdigit = 0x400004e8;
islower = 0x400004ec;
isspace = 0x400004fc;
isupper = 0x40000500;
strcasecmp = 0x4000051c;
strcoll = 0x40000534;
strlwr = 0x40000540;
strncasecmp = 0x40000544;
strupr = 0x40000564;
