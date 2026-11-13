/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ROM function interface esp32c2.rom.newlib.ld for esp32c2
 *
 *
 * Generated from ./interface-esp32c2.yml md5sum c679b6ed5e9f0a9c3e7b93e5e0f2a1a3
 *
 * Compatible with ROM where ECO version equal or greater to 1.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED. !!! BUT EDITED !!!
 * The file was originally generated for use with newlib, but it was split into
 * multiple files to make it compatible with picolibc.
 */

/***************************************
 Group newlib
 ***************************************/

/* Zephyr: reentrant ROM call, provided by the selected libc */
/* strdup       = 0x40000510; */
/* strndup      = 0x40000534; */
/* rand_r       = 0x4000056c; */
/* rand         = 0x40000570; */
/* srand        = 0x40000574; */
/* atoi         = 0x40000580; */
/* atol         = 0x40000584; */
/* strtol       = 0x40000588; */
/* strtoul      = 0x4000058c; */
/* fflush       = 0x40000590; */
/* _fflush_r    = 0x40000594; */
/* _fwalk       = 0x40000598; */
/* _fwalk_reent = 0x4000059c; */
/* __swbuf_r    = 0x400005a8; */

/* Functions */
/* Zephyr: reentrant ROM call, provided by the selected libc */
/* _isatty_r = 0x400004b4; */
/* __smakebuf_r = 0x400005a0; */
/* __swhatbuf_r = 0x400005a4; */
/* Zephyr: reentrant ROM call, provided by the selected libc */
/* __swbuf = 0x400005ac; */
/* Zephyr: reentrant ROM call, provided by the selected libc */
/* __swsetup_r = 0x400005b0; */
toupper = 0x400004ec;
tolower = 0x400004f0;
isalnum = 0x400004bc;
isalpha = 0x400004c0;
isdigit = 0x400004d0;
islower = 0x400004d4;
isspace = 0x400004e4;
isupper = 0x400004e8;
strcasecmp = 0x40000504;
strcoll = 0x4000051c;
strlwr = 0x40000528;
strncasecmp = 0x4000052c;
strupr = 0x4000054c;
