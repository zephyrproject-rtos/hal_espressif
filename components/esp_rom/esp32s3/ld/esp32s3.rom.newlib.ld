/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32s3.rom.newlib.ld for esp32s3
 *
 *
 * Generated from ./interface-esp32s3.yml md5sum 39c4ce259b11323b9404c192b01b712b
 *
 * Compatible with ROM where ECO version equal or greater to 0.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. !!! BUT EDITED !!!
 * The file was originally generated for use with newlib, but it was split into
 * multiple files to make it compatible with picolibc.
 */

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( atoi = 0x400014d0 );
PROVIDE ( atol = 0x400014dc );
PROVIDE ( strdup = 0x40001380 );
PROVIDE ( strndup = 0x400013ec );
PROVIDE ( rand_r = 0x40001494 );
PROVIDE ( rand = 0x400014a0 );
PROVIDE ( srand = 0x400014ac );
PROVIDE ( strtol = 0x400014e8 );
PROVIDE ( strtoul = 0x400014f4 );
PROVIDE ( longjmp = 0x40001440 );
PROVIDE ( setjmp = 0x4000144c );
/*******************************************/

PROVIDE( fflush = 0x40001500 );
PROVIDE( _fflush_r = 0x4000150c );
PROVIDE( _fwalk = 0x40001518 );
PROVIDE( _fwalk_reent = 0x40001524 );
PROVIDE( __swbuf_r = 0x40001548 );
__swbuf = 0x40001554;
toupper = 0x40001314;
tolower = 0x40001320;
isalnum = 0x40001284;
isalpha = 0x40001290;
isdigit = 0x400012c0;
islower = 0x400012cc;
isspace = 0x400012fc;
isupper = 0x40001308;
strcasecmp = 0x4000135c;
strcoll = 0x400013a4;
strlwr = 0x400013c8;
strncasecmp = 0x400013d4;
strupr = 0x40001434;
hmac_md5_vector = 0x40001c80;
hmac_md5 = 0x40001c8c;
