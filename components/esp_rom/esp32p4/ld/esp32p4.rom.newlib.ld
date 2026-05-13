/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32p4.rom.newlib.ld for esp32p4
 *
 *
 * Generated from ./target/esp32p4/interface-esp32p4.yml md5sum 56d78222be1daa0502090a078288f4d5
 *
 * Compatible with ROM where ECO version equal or greater to 5.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. !!! BUT EDITED !!!
 * The file was originally generated for use with newlib, but it was split into
 * multiple files to make it compatible with picolibc.
 */

/***************************************
 Group newlib
 ***************************************/

/* ZEPHYR: Keep PROVIDE for these symbols: */
PROVIDE ( strdup = 0x4fc002f0 );
PROVIDE ( strndup = 0x4fc00314 );
PROVIDE ( rand_r = 0x4fc0034c );
PROVIDE ( rand = 0x4fc00350 );
PROVIDE ( srand = 0x4fc00354 );
PROVIDE ( atoi = 0x4fc00360 );
PROVIDE ( atol = 0x4fc00364 );
PROVIDE ( strtol = 0x4fc00368 );
PROVIDE ( strtoul = 0x4fc0036c );
PROVIDE ( fflush = 0x4fc00370 );
PROVIDE ( _fflush_r = 0x4fc00374 );
PROVIDE ( _fwalk = 0x4fc00378 );
PROVIDE ( _fwalk_reent = 0x4fc0037c );
PROVIDE ( __swbuf_r = 0x4fc00388 );
/*******************************************/

/* Functions */
_isatty_r = 0x4fc00294;
__smakebuf_r = 0x4fc00380;
__swhatbuf_r = 0x4fc00384;
__swbuf = 0x4fc0038c;
__swsetup_r = 0x4fc00390;
