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

/* Zephyr: reentrant ROM call, provided by the selected libc */
/* atoi    = 0x400014d0; */
/* atol    = 0x400014dc; */
/* strdup  = 0x40001380; */
/* strndup = 0x400013ec; */
/* rand_r  = 0x40001494; */
/* rand    = 0x400014a0; */
/* srand   = 0x400014ac; */
/* strtol  = 0x400014e8; */
/* strtoul = 0x400014f4; */
/* longjmp = 0x40001440; */
/* setjmp  = 0x4000144c; */
/* fflush       = 0x40001500; */
/* _fflush_r    = 0x4000150c; */
/* _fwalk       = 0x40001518; */
/* _fwalk_reent = 0x40001524; */
/* __swbuf_r    = 0x40001548; */
/* Zephyr: reentrant ROM call, provided by the selected libc */
/* __swbuf = 0x40001554; */
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
/* The ROM hmac_md5 implementations call the ROM malloc, which
 * dispatches through syscall_table_ptr. That table is never
 * installed in this port, so any call faults on a NULL pointer.
 * The supplicant builds its own hmac_md5/hmac_md5_vector from
 * src/crypto/md5.c, which must win the link instead.
 * hmac_md5_vector = 0x40001c80;
 * hmac_md5 = 0x40001c8c;
 */
