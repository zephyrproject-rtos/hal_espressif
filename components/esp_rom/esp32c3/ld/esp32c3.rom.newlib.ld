/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32c3.rom.newlib.ld for esp32c3
 *
 *
 * Generated from ./interface-esp32c3.yml md5sum 93b28a9e1fe42d212018eb4336849208
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

/* Zephyr: reentrant ROM call, provided by the selected libc */
/* strdup  = 0x400003dc; */
/* strndup = 0x40000400; */
/* rand    = 0x4000043c; */
/* srand   = 0x40000440; */
/* rand_r  = 0x40000438; */
/* atoi    = 0x4000044c; */
/* atol    = 0x40000450; */
/* strtol  = 0x40000454; */
/* strtoul = 0x40000458; */
/* fflush       = 0x4000045c; */
/* _fflush_r    = 0x40000460; */
/* _fwalk       = 0x40000464; */
/* _fwalk_reent = 0x40000468; */
/* __swbuf_r    = 0x40000474; */

/* Functions */
__swbuf = 0x40000478;
toupper = 0x400003b8;
tolower = 0x400003bc;
isalnum = 0x40000388;
isalpha = 0x4000038c;
isdigit = 0x4000039c;
islower = 0x400003a0;
isspace = 0x400003b0;
isupper = 0x400003b4;
strcasecmp = 0x400003d0;
strcoll = 0x400003e8;
strlwr = 0x400003f4;
strncasecmp = 0x400003f8;
strupr = 0x40000418;
hmac_md5_vector = 0x40000620;
hmac_md5 = 0x40000624;
