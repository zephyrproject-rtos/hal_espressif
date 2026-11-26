/**
 * SPDX-FileCopyrightText: 2017-2026 Espressif Systems (Shanghai) CO LTD
 *
 *  SPDX-License-Identifier: Apache-2.0 OR MIT
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Group: Default */
/** Type of cmd register
 *  reserved
 */
typedef union {
    struct {
        /** conf_bitlen : R/W; bitpos: [22:0]; default: 0;
         *  Define the spi_clk cycles of  SPI_CONF state. Can be configured in CONF state.
         */
        uint32_t conf_bitlen:23;
        uint32_t reserved_23:1;
        /** usr : R/W; bitpos: [24]; default: 0;
         *  User define command enable.  An operation will be triggered when the bit is set.
         *  The bit will be cleared once the operation done.1: enable 0: disable. Can not be
         *  changed by CONF_buf.
         */
        uint32_t usr:1;
        uint32_t reserved_25:7;
    };
    uint32_t val;
} spi_cmd_reg_t;

/** Type of addr register
 *  [31:8]:address to slave, [7:0]:Reserved. Can be configured in CONF state.
 */
typedef union {
    struct {
        /** usr_addr_value : R/W; bitpos: [31:0]; default: 0;
         *  [31:8]:address to slave, [7:0]:Reserved. Can be configured in CONF state.
         */
        uint32_t usr_addr_value:32;
    };
    uint32_t val;
} spi_addr_reg_t;

/** Type of ctrl register
 *  reserved
 */
typedef union {
    struct {
        uint32_t reserved_0:2;
        /** ext_hold_en : R/W; bitpos: [2]; default: 0;
         *  Set the bit to hold spi. The bit is combined with
         *  spi_usr_prep_hold,spi_usr_cmd_hold,spi_usr_addr_hold,spi_usr_dummy_hold,spi_usr_din_hold,spi_usr_dout_hold
         *  and spi_usr_hold_pol. Can be configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t ext_hold_en:1;
        /** dummy_out : R/W; bitpos: [3]; default: 0;
         *  In the dummy phase the signal level of spi is output by the spi controller. Can be
         *  configured in CONF state.
         */
        uint32_t dummy_out:1;
        uint32_t reserved_4:1;
        /** faddr_dual : R/W; bitpos: [5]; default: 0;
         *  Apply 2 signals during addr phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t faddr_dual:1;
        /** faddr_quad : R/W; bitpos: [6]; default: 0;
         *  Apply 4 signals during addr phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t faddr_quad:1;
        /** faddr_oct : R/W; bitpos: [7]; default: 0;
         *  Apply 8 signals during addr phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t faddr_oct:1;
        /** fcmd_dual : R/W; bitpos: [8]; default: 0;
         *  Apply 2 signals during command phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t fcmd_dual:1;
        /** fcmd_quad : R/W; bitpos: [9]; default: 0;
         *  Apply 4 signals during command phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t fcmd_quad:1;
        /** fcmd_oct : R/W; bitpos: [10]; default: 0;
         *  Apply 8 signals during command phase 1:enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t fcmd_oct:1;
        uint32_t reserved_11:3;
        /** fread_dual : R/W; bitpos: [14]; default: 0;
         *  In the read operations, read-data phase apply 2 signals. 1: enable 0: disable. Can
         *  be configured in CONF state.
         */
        uint32_t fread_dual:1;
        /** fread_quad : R/W; bitpos: [15]; default: 0;
         *  In the read operations read-data phase apply 4 signals. 1: enable 0: disable.  Can
         *  be configured in CONF state.
         */
        uint32_t fread_quad:1;
        /** fread_oct : R/W; bitpos: [16]; default: 0;
         *  In the read operations read-data phase apply 8 signals. 1: enable 0: disable.  Can
         *  be configured in CONF state.
         */
        uint32_t fread_oct:1;
        uint32_t reserved_17:1;
        /** q_pol : R/W; bitpos: [18]; default: 1;
         *  The bit is used to set MISO line polarity, 1: high 0, low. Can be configured in
         *  CONF state.
         */
        uint32_t q_pol:1;
        /** d_pol : R/W; bitpos: [19]; default: 1;
         *  The bit is used to set MOSI line polarity, 1: high 0, low. Can be configured in
         *  CONF state.
         */
        uint32_t d_pol:1;
        uint32_t reserved_20:1;
        /** wp_reg : R/W; bitpos: [21]; default: 1;
         *  Write protect signal output when SPI is idle.  1: output high, 0: output low.  Can
         *  be configured in CONF state.
         */
        uint32_t wp_reg:1;
        uint32_t reserved_22:3;
        /** rd_bit_order : R/W; bitpos: [25]; default: 0;
         *  In read-data (MISO) phase 1: LSB first 0: MSB first. Can be configured in CONF
         *  state.
         */
        uint32_t rd_bit_order:1;
        /** wr_bit_order : R/W; bitpos: [26]; default: 0;
         *  In command address write-data (MOSI) phases 1: LSB firs 0: MSB first. Can be
         *  configured in CONF state.
         */
        uint32_t wr_bit_order:1;
        uint32_t reserved_27:5;
    };
    uint32_t val;
} spi_ctrl_reg_t;

/** Type of ctrl1 register
 *  reserved
 */
typedef union {
    struct {
        /** clk_mode : R/W; bitpos: [1:0]; default: 0;
         *  SPI clock mode bits. 0: SPI clock is off when CS inactive 1: SPI clock is delayed
         *  one cycle after CS inactive 2: SPI clock is delayed two cycles after CS inactive 3:
         *  SPI clock is always on. Can be configured in CONF state.
         */
        uint32_t clk_mode:2;
        /** clk_mode_13 : R/W; bitpos: [2]; default: 0;
         *  {CPOL, CPHA},1: support spi clk mode 1 and 3, first edge output data B[0]/B[7].  0:
         *  support spi clk mode 0 and 2, first edge output data B[1]/B[6].
         */
        uint32_t clk_mode_13:1;
        /** rsck_data_out : R/W; bitpos: [3]; default: 0;
         *  It saves half a cycle when tsck is the same as rsck. 1: output data at rsck posedge
         *  0: output data at tsck posedge
         */
        uint32_t rsck_data_out:1;
        /** w16_17_wr_ena : R/W; bitpos: [4]; default: 1;
         *  1:reg_buf[16] [17] can be written   0:reg_buf[16] [17] can not  be written. Can be
         *  configured in CONF state.
         */
        uint32_t w16_17_wr_ena:1;
        uint32_t reserved_5:9;
        /** cs_hold_delay : R/W; bitpos: [19:14]; default: 1;
         *  SPI cs signal is delayed by spi clock cycles. Can be configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t cs_hold_delay:6;
        uint32_t reserved_20:12;
    };
    uint32_t val;
} spi_ctrl1_reg_t;

/** Type of ctrl2 register
 *  reserved
 */
typedef union {
    struct {
        /** cs_setup_time : R/W; bitpos: [12:0]; default: 0;
         *  (cycles+1) of prepare phase by spi clock this bits are combined with spi_cs_setup
         *  bit. Can be configured in CONF state.
         */
        uint32_t cs_setup_time:13;
        /** cs_hold_time : R/W; bitpos: [25:13]; default: 1;
         *  delay cycles of cs pin by spi clock this bits are combined with spi_cs_hold bit.
         *  Can be configured in CONF state.
         */
        uint32_t cs_hold_time:13;
        /** cs_delay_mode : R/W; bitpos: [28:26]; default: 0;
         *  spi_cs signal is delayed by spi_clk . 0: zero 1: if spi_ck_out_edge or
         *  spi_ck_i_edge is set 1 delayed by half cycle  else delayed by one cycle 2: if
         *  spi_ck_out_edge or spi_ck_i_edge is set 1 delayed by one cycle, else delayed by
         *  half cycle 3: delayed one cycle. Can be configured in CONF state.
         */
        uint32_t cs_delay_mode:3;
        /** cs_delay_num : R/W; bitpos: [30:29]; default: 0;
         *  spi_cs signal is delayed by system clock cycles. Can be configured in CONF state.
         */
        uint32_t cs_delay_num:2;
        uint32_t reserved_31:1;
    };
    uint32_t val;
} spi_ctrl2_reg_t;

/** Type of clock register
 *  In the master mode 1: spi_clk is equal to system 0: spi_clk is divided from system
 *  clock. Can be configured in CONF state.
 */
typedef union {
    struct {
        /** clkcnt_l : R/W; bitpos: [5:0]; default: 3;
         *  In the master mode it must be equal to spi_clkcnt_N. In the slave mode it must be
         *  0. Can be configured in CONF state.
         */
        uint32_t clkcnt_l:6;
        /** clkcnt_h : R/W; bitpos: [11:6]; default: 1;
         *  In the master mode it must be floor((spi_clkcnt_N+1)/2-1). In the slave mode it
         *  must be 0. Can be configured in CONF state.
         */
        uint32_t clkcnt_h:6;
        /** clkcnt_n : R/W; bitpos: [17:12]; default: 3;
         *  In the master mode it is the divider of spi_clk. So spi_clk frequency is
         *  system/(spi_clkdiv_pre+1)/(spi_clkcnt_N+1). Can be configured in CONF state.
         */
        uint32_t clkcnt_n:6;
        /** clkdiv_pre : R/W; bitpos: [30:18]; default: 0;
         *  In the master mode it is pre-divider of spi_clk.  Can be configured in CONF state.
         */
        uint32_t clkdiv_pre:13;
        /** clk_equ_sysclk : R/W; bitpos: [31]; default: 1;
         *  In the master mode 1: spi_clk is equal to system 0: spi_clk is divided from system
         *  clock. Can be configured in CONF state.
         */
        uint32_t clk_equ_sysclk:1;
    };
    uint32_t val;
} spi_clock_reg_t;

/** Type of user register
 *  This bit enable the command phase of an operation. Can be configured in CONF state.
 */
typedef union {
    struct {
        /** doutdin : R/W; bitpos: [0]; default: 0;
         *  Set the bit to enable full duplex communication. 1: enable 0: disable. Can be
         *  configured in CONF state.
         */
        uint32_t doutdin:1;
        uint32_t reserved_1:2;
        /** qpi_mode : R/W; bitpos: [3]; default: 0;
         *  Both for master mode and slave mode. 1: spi controller is in QPI mode. 0: others.
         *  Can be configured in CONF state.
         */
        uint32_t qpi_mode:1;
        /** opi_mode : R/W; bitpos: [4]; default: 0;
         *  Just for master mode. 1: spi controller is in OPI mode (all in 8-b-m). 0: others.
         *  Can be configured in CONF state.
         */
        uint32_t opi_mode:1;
        /** tsck_i_edge : R/W; bitpos: [5]; default: 0;
         *  In the slave mode, this bit can be used to change the polarity of tsck. 0: tsck =
         *  spi_ck_i. 1:tsck = !spi_ck_i.
         */
        uint32_t tsck_i_edge:1;
        /** cs_hold : R/W; bitpos: [6]; default: 1;
         *  spi cs keep low when spi is in  done  phase. 1: enable 0: disable. Can be
         *  configured in CONF state.
         */
        uint32_t cs_hold:1;
        /** cs_setup : R/W; bitpos: [7]; default: 1;
         *  spi cs is enable when spi is in  prepare  phase. 1: enable 0: disable. Can be
         *  configured in CONF state.
         */
        uint32_t cs_setup:1;
        /** rsck_i_edge : R/W; bitpos: [8]; default: 0;
         *  In the slave mode, this bit can be used to change the polarity of rsck. 0: rsck =
         *  !spi_ck_i. 1:rsck = spi_ck_i.
         */
        uint32_t rsck_i_edge:1;
        /** ck_out_edge : R/W; bitpos: [9]; default: 0;
         *  the bit combined with spi_mosi_delay_mode bits to set mosi signal delay mode. Can
         *  be configured in CONF state.
         */
        uint32_t ck_out_edge:1;
        /** rd_byte_order : R/W; bitpos: [10]; default: 0;
         *  In read-data (MISO) phase 1: big-endian 0: little_endian. Can be configured in CONF
         *  state.
         */
        uint32_t rd_byte_order:1;
        /** wr_byte_order : R/W; bitpos: [11]; default: 0;
         *  In command address write-data (MOSI) phases 1: big-endian 0: litte_endian. Can be
         *  configured in CONF state.
         */
        uint32_t wr_byte_order:1;
        /** fwrite_dual : R/W; bitpos: [12]; default: 0;
         *  In the write operations read-data phase apply 2 signals. Can be configured in CONF
         *  state.
         */
        uint32_t fwrite_dual:1;
        /** fwrite_quad : R/W; bitpos: [13]; default: 0;
         *  In the write operations read-data phase apply 4 signals. Can be configured in CONF
         *  state.
         */
        uint32_t fwrite_quad:1;
        /** fwrite_oct : R/W; bitpos: [14]; default: 0;
         *  In the write operations read-data phase apply 8 signals. Can be configured in CONF
         *  state.
         */
        uint32_t fwrite_oct:1;
        /** usr_conf_nxt : R/W; bitpos: [15]; default: 0;
         *  1: Enable the DMA CONF phase of next seg-trans operation, which means seg-trans
         *  will continue. 0: The seg-trans will end after the current SPI seg-trans or this is
         *  not seg-trans mode. Can be configured in CONF state.
         */
        uint32_t usr_conf_nxt:1;
        /** sio : R/W; bitpos: [16]; default: 0;
         *  Set the bit to enable 3-line half duplex communication mosi and miso signals share
         *  the same pin. 1: enable 0: disable. Can be configured in CONF state.
         */
        uint32_t sio:1;
        /** usr_hold_pol : R/W; bitpos: [17]; default: 0;
         *  It is combined with hold bits to set the polarity of spi hold line 1: spi will be
         *  held when spi hold line is high 0: spi will be held when spi hold line is low. Can
         *  be configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_hold_pol:1;
        /** usr_dout_hold : R/W; bitpos: [18]; default: 0;
         *  spi is hold at data out state the bit are combined with spi_usr_hold_pol bit. Can
         *  be configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_dout_hold:1;
        /** usr_din_hold : R/W; bitpos: [19]; default: 0;
         *  spi is hold at data in state the bit are combined with spi_usr_hold_pol bit. Can be
         *  configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_din_hold:1;
        /** usr_dummy_hold : R/W; bitpos: [20]; default: 0;
         *  spi is hold at dummy state the bit are combined with spi_usr_hold_pol bit. Can be
         *  configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_dummy_hold:1;
        /** usr_addr_hold : R/W; bitpos: [21]; default: 0;
         *  spi is hold at address state the bit are combined with spi_usr_hold_pol bit. Can be
         *  configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_addr_hold:1;
        /** usr_cmd_hold : R/W; bitpos: [22]; default: 0;
         *  spi is hold at command state the bit are combined with spi_usr_hold_pol bit. Can be
         *  configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_cmd_hold:1;
        /** usr_prep_hold : R/W; bitpos: [23]; default: 0;
         *  spi is hold at prepare state the bit are combined with spi_usr_hold_pol bit. Can be
         *  configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t usr_prep_hold:1;
        /** usr_miso_highpart : R/W; bitpos: [24]; default: 0;
         *  read-data phase only access to high-part of the buffer spi_w8~spi_w15. 1: enable 0:
         *  disable. Can be configured in CONF state.
         */
        uint32_t usr_miso_highpart:1;
        /** usr_mosi_highpart : R/W; bitpos: [25]; default: 0;
         *  write-data phase only access to high-part of the buffer spi_w8~spi_w15. 1: enable
         *  0: disable.  Can be configured in CONF state.
         */
        uint32_t usr_mosi_highpart:1;
        /** usr_dummy_idle : R/W; bitpos: [26]; default: 0;
         *  spi clock is disable in dummy phase when the bit is enable. Can be configured in
         *  CONF state.
         */
        uint32_t usr_dummy_idle:1;
        /** usr_mosi : R/W; bitpos: [27]; default: 0;
         *  This bit enable the write-data phase of an operation. Can be configured in CONF
         *  state.
         */
        uint32_t usr_mosi:1;
        /** usr_miso : R/W; bitpos: [28]; default: 0;
         *  This bit enable the read-data phase of an operation. Can be configured in CONF
         *  state.
         */
        uint32_t usr_miso:1;
        /** usr_dummy : R/W; bitpos: [29]; default: 0;
         *  This bit enable the dummy phase of an operation. Can be configured in CONF state.
         */
        uint32_t usr_dummy:1;
        /** usr_addr : R/W; bitpos: [30]; default: 0;
         *  This bit enable the address phase of an operation. Can be configured in CONF state.
         */
        uint32_t usr_addr:1;
        /** usr_command : R/W; bitpos: [31]; default: 1;
         *  This bit enable the command phase of an operation. Can be configured in CONF state.
         */
        uint32_t usr_command:1;
    };
    uint32_t val;
} spi_user_reg_t;

/** Type of user1 register
 *  The length in bits of address phase. The register value shall be (bit_num-1). Can
 *  be configured in CONF state.
 */
typedef union {
    struct {
        /** usr_dummy_cyclelen : R/W; bitpos: [7:0]; default: 7;
         *  The length in spi_clk cycles of dummy phase. The register value shall be
         *  (cycle_num-1). Can be configured in CONF state.
         */
        uint32_t usr_dummy_cyclelen:8;
        uint32_t reserved_8:19;
        /** usr_addr_bitlen : R/W; bitpos: [31:27]; default: 23;
         *  The length in bits of address phase. The register value shall be (bit_num-1). Can
         *  be configured in CONF state.
         */
        uint32_t usr_addr_bitlen:5;
    };
    uint32_t val;
} spi_user1_reg_t;

/** Type of user2 register
 *  The length in bits of command phase. The register value shall be (bit_num-1). Can
 *  be configured in CONF state.
 */
typedef union {
    struct {
        /** usr_command_value : R/W; bitpos: [15:0]; default: 0;
         *  The value of  command. Can be configured in CONF state.
         */
        uint32_t usr_command_value:16;
        uint32_t reserved_16:12;
        /** usr_command_bitlen : R/W; bitpos: [31:28]; default: 7;
         *  The length in bits of command phase. The register value shall be (bit_num-1). Can
         *  be configured in CONF state.
         */
        uint32_t usr_command_bitlen:4;
    };
    uint32_t val;
} spi_user2_reg_t;

/** Type of mosi_dlen register
 *  reserved
 */
typedef union {
    struct {
        /** usr_mosi_dbitlen : R/W; bitpos: [22:0]; default: 0;
         *  The length in bits of write-data. The register value shall be (bit_num-1). Can be
         *  configured in CONF state.
         */
        uint32_t usr_mosi_dbitlen:23;
        uint32_t reserved_23:9;
    };
    uint32_t val;
} spi_mosi_dlen_reg_t;

/** Type of miso_dlen register
 *  reserved
 */
typedef union {
    struct {
        /** usr_miso_dbitlen : R/W; bitpos: [22:0]; default: 0;
         *  The length in bits of  read-data. The register value shall be (bit_num-1). Can be
         *  configured in CONF state.
         */
        uint32_t usr_miso_dbitlen:23;
        uint32_t reserved_23:9;
    };
    uint32_t val;
} spi_miso_dlen_reg_t;

/** Type of misc register
 *  1:  spi quad input swap enable  0:  spi quad input swap disable. Can be configured
 *  in CONF state.
 */
typedef union {
    struct {
        /** cs0_dis : R/W; bitpos: [0]; default: 0;
         *  SPI CS0 pin enable, 1: disable CS0, 0: spi_cs0 signal is from/to CS0 pin. Can be
         *  configured in CONF state.
         */
        uint32_t cs0_dis:1;
        /** cs1_dis : R/W; bitpos: [1]; default: 1;
         *  SPI CS1 pin enable, 1: disable CS1, 0: spi_cs1 signal is from/to CS1 pin. Can be
         *  configured in CONF state.
         */
        uint32_t cs1_dis:1;
        /** cs2_dis : R/W; bitpos: [2]; default: 1;
         *  SPI CS2 pin enable, 1: disable CS2, 0: spi_cs2 signal is from/to CS2 pin. Can be
         *  configured in CONF state.
         */
        uint32_t cs2_dis:1;
        /** cs3_dis : R/W; bitpos: [3]; default: 1;
         *  reserved
         */
        uint32_t cs3_dis:1;
        /** cs4_dis : R/W; bitpos: [4]; default: 1;
         *  SPI CS4 pin enable, 1: disable CS4, 0: spi_cs4 signal is from/to CS4 pin. Can be
         *  configured in CONF state.
         */
        uint32_t cs4_dis:1;
        /** cs5_dis : R/W; bitpos: [5]; default: 1;
         *  SPI CS5 pin enable, 1: disable CS5, 0: spi_cs5 signal is from/to CS5 pin. Can be
         *  configured in CONF state.
         */
        uint32_t cs5_dis:1;
        /** ck_dis : R/W; bitpos: [6]; default: 0;
         *  1: spi clk out disable,  0: spi clk out enable. Can be configured in CONF state.
         */
        uint32_t ck_dis:1;
        /** master_cs_pol : R/W; bitpos: [12:7]; default: 0;
         *  In the master mode the bits are the polarity of spi cs line, the value is
         *  equivalent to spi_cs ^ spi_master_cs_pol. Can be configured in CONF state.
         */
        uint32_t master_cs_pol:6;
        uint32_t reserved_13:3;
        /** clk_data_dtr_en : R/W; bitpos: [16]; default: 0;
         *  1: SPI master DTR mode is applied to SPI clk, data and spi_dqs
         */
        uint32_t clk_data_dtr_en:1;
        /** data_dtr_en : R/W; bitpos: [17]; default: 0;
         *  1: SPI clk and data of SPI_DOUT and SPI_DIN state are in DTR mode, including master
         *  1/2/4/8-bm. Can be configured in CONF state.
         */
        uint32_t data_dtr_en:1;
        /** addr_dtr_en : R/W; bitpos: [18]; default: 0;
         *  1: SPI clk and data of SPI_SEND_ADDR state are in DTR mode, including master
         *  1/2/4/8-bm. Can be configured in CONF state.
         */
        uint32_t addr_dtr_en:1;
        /** cmd_dtr_en : R/W; bitpos: [19]; default: 0;
         *  1: SPI clk and data of SPI_SEND_CMD state are in DTR mode, including master
         *  1/2/4/8-bm. Can be configured in CONF state.
         */
        uint32_t cmd_dtr_en:1;
        /** cd_data_set : R/W; bitpos: [20]; default: 0;
         *  1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_DOUT or SPI_DIN state.  0:
         *  spi_cd = spi_cd_idle_edge. Can be configured in CONF state.
         */
        uint32_t cd_data_set:1;
        /** cd_dummy_set : R/W; bitpos: [21]; default: 0;
         *  1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_DUMMY state.  0: spi_cd =
         *  spi_cd_idle_edge. Can be configured in CONF state.
         */
        uint32_t cd_dummy_set:1;
        /** cd_addr_set : R/W; bitpos: [22]; default: 0;
         *  1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_SEND_ADDR state.  0:
         *  spi_cd = spi_cd_idle_edge. Can be configured in CONF state.
         */
        uint32_t cd_addr_set:1;
        /** slave_cs_pol : R/W; bitpos: [23]; default: 0;
         *  spi slave input cs polarity select. 1: inv  0: not change. Can be configured in
         *  CONF state.
         */
        uint32_t slave_cs_pol:1;
        /** dqs_idle_edge : R/W; bitpos: [24]; default: 0;
         *  The default value of spi_dqs. Can be configured in CONF state.
         */
        uint32_t dqs_idle_edge:1;
        /** cd_cmd_set : R/W; bitpos: [25]; default: 0;
         *  1: spi_cd = !spi_cd_idle_edge when spi_st[3:0] is in SPI_SEND_CMD state.  0: spi_cd
         *  = spi_cd_idle_edge. Can be configured in CONF state.
         */
        uint32_t cd_cmd_set:1;
        /** cd_idle_edge : R/W; bitpos: [26]; default: 0;
         *  The default value of spi_cd. Can be configured in CONF state.
         */
        uint32_t cd_idle_edge:1;
        uint32_t reserved_27:2;
        /** ck_idle_edge : R/W; bitpos: [29]; default: 0;
         *  1: spi clk line is high when idle     0: spi clk line is low when idle. Can be
         *  configured in CONF state.
         */
        uint32_t ck_idle_edge:1;
        /** cs_keep_active : R/W; bitpos: [30]; default: 0;
         *  spi cs line keep low when the bit is set. Can be configured in CONF state.
         */
        uint32_t cs_keep_active:1;
        /** quad_din_pin_swap : R/W; bitpos: [31]; default: 0;
         *  1:  spi quad input swap enable  0:  spi quad input swap disable. Can be configured
         *  in CONF state.
         */
        uint32_t quad_din_pin_swap:1;
    };
    uint32_t val;
} spi_misc_reg_t;

/** Type of slave register
 *  Software reset enable, reset the spi clock line cs line and data lines. Can be
 *  configured in CONF state.
 */
typedef union {
    struct {
        uint32_t reserved_0:4;
        /** trans_done : R/W; bitpos: [4]; default: 0;
         *  The interrupt raw bit for the completion of any operation in both the master mode
         *  and the slave mode.  Can not be changed by CONF_buf.
         */
        uint32_t trans_done:1;
        /** int_rd_buf_done_en : R/W; bitpos: [5]; default: 0;
         *  spi_slv_rd_buf Interrupt enable. 1: enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t int_rd_buf_done_en:1;
        /** int_wr_buf_done_en : R/W; bitpos: [6]; default: 0;
         *  spi_slv_wr_buf Interrupt enable. 1: enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t int_wr_buf_done_en:1;
        /** int_rd_dma_done_en : R/W; bitpos: [7]; default: 0;
         *  spi_slv_rd_dma Interrupt enable. 1: enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t int_rd_dma_done_en:1;
        /** int_wr_dma_done_en : R/W; bitpos: [8]; default: 0;
         *  spi_slv_wr_dma Interrupt enable. 1: enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t int_wr_dma_done_en:1;
        /** int_trans_done_en : R/W; bitpos: [9]; default: 1;
         *  spi_trans_done Interrupt enable. 1: enable 0: disable. Can be configured in CONF
         *  state.
         */
        uint32_t int_trans_done_en:1;
        /** int_dma_seg_trans_en : R/W; bitpos: [10]; default: 0;
         *  spi_dma_seg_trans_done Interrupt enable. 1: enable 0: disable. Can be configured in
         *  CONF state.
         */
        uint32_t int_dma_seg_trans_en:1;
        /** seg_magic_err_int_en : R/W; bitpos: [11]; default: 0;
         *  1: Enable seg magic value error interrupt. 0: Others. Can be configured in CONF
         *  state.
         */
        uint32_t seg_magic_err_int_en:1;
        uint32_t reserved_12:11;
        /** trans_cnt : RO; bitpos: [26:23]; default: 0;
         *  The operations counter in both the master mode and the slave mode.
         */
        uint32_t trans_cnt:4;
        uint32_t reserved_27:2;
        /** trans_done_auto_clr_en : R/W; bitpos: [29]; default: 0;
         *  spi_trans_done auto clear enable, clear it 3 apb cycles after the pos edge of
         *  spi_trans_done.  0:disable. 1: enable. Can be configured in CONF state.
         */
        uint32_t trans_done_auto_clr_en:1;
        /** slave_mode : R/W; bitpos: [30]; default: 0;
         *  Set SPI work mode. 1: slave mode 0: master mode.
         */
        uint32_t slave_mode:1;
        /** soft_reset : R/W; bitpos: [31]; default: 0;
         *  Software reset enable, reset the spi clock line cs line and data lines. Can be
         *  configured in CONF state.
         */
        uint32_t soft_reset:1;
    };
    uint32_t val;
} spi_slave_reg_t;

/** Type of slave1 register
 *  In the slave mode it is the value of address.
 */
typedef union {
    struct {
        uint32_t reserved_0:10;
        /** slv_addr_err_clr : R/W; bitpos: [10]; default: 0;
         *  1: Clear spi_slv_addr_err. 0: not valid. Can be changed by CONF_buf.
         */
        uint32_t slv_addr_err_clr:1;
        /** slv_cmd_err_clr : R/W; bitpos: [11]; default: 0;
         *  1: Clear spi_slv_cmd_err. 0: not valid.  Can be changed by CONF_buf.
         */
        uint32_t slv_cmd_err_clr:1;
        /** slv_no_qpi_en : R/W; bitpos: [12]; default: 0;
         *  1: spi slave QPI mode is not supported. 0: spi slave QPI mode is supported.
         */
        uint32_t slv_no_qpi_en:1;
        /** slv_addr_err : RO; bitpos: [13]; default: 0;
         *  1: The address value of the last SPI transfer is not supported by SPI slave. 0: The
         *  address value is supported or no address value is received.
         */
        uint32_t slv_addr_err:1;
        /** slv_cmd_err : RO; bitpos: [14]; default: 0;
         *  1: The command value of the last SPI transfer is not supported by SPI slave. 0: The
         *  command value is supported or no command value is received.
         */
        uint32_t slv_cmd_err:1;
        /** slv_wr_dma_done : R/W; bitpos: [15]; default: 0;
         *  The interrupt raw bit for the completion of dma write operation in the slave mode.
         *  Can not be changed by CONF_buf.
         */
        uint32_t slv_wr_dma_done:1;
        /** slv_last_command : R/W; bitpos: [23:16]; default: 0;
         *  In the slave mode it is the value of command.
         */
        uint32_t slv_last_command:8;
        /** slv_last_addr : R/W; bitpos: [31:24]; default: 0;
         *  In the slave mode it is the value of address.
         */
        uint32_t slv_last_addr:8;
    };
    uint32_t val;
} spi_slave1_reg_t;

/** Type of slv_wrbuf_dlen register
 *  The basic spi_clk cycles of CONF state. The real cycle length of CONF state, if
 *  spi_usr_conf is enabled, is spi_conf_base_bitlen[6:0] + spi_conf_bitlen[23:0].
 */
typedef union {
    struct {
        uint32_t reserved_0:24;
        /** slv_wr_buf_done : R/W; bitpos: [24]; default: 0;
         *  The interrupt raw bit for the completion of write-buffer operation in the slave
         *  mode.  Can not be changed by CONF_buf.
         */
        uint32_t slv_wr_buf_done:1;
        /** conf_base_bitlen : R/W; bitpos: [31:25]; default: 108;
         *  The basic spi_clk cycles of CONF state. The real cycle length of CONF state, if
         *  spi_usr_conf is enabled, is spi_conf_base_bitlen[6:0] + spi_conf_bitlen[23:0].
         */
        uint32_t conf_base_bitlen:7;
    };
    uint32_t val;
} spi_slv_wrbuf_dlen_reg_t;

/** Type of slv_rdbuf_dlen register
 *  reserved
 */
typedef union {
    struct {
        /** slv_dma_rd_bytelen : R/W; bitpos: [19:0]; default: 0;
         *  In the slave mode it is the length in bytes for read operations. The register value
         *  shall be byte_num.
         */
        uint32_t slv_dma_rd_bytelen:20;
        uint32_t reserved_20:4;
        /** slv_rd_buf_done : R/W; bitpos: [24]; default: 0;
         *  The interrupt raw bit for the completion of read-buffer operation in the slave
         *  mode.  Can not be changed by CONF_buf.
         */
        uint32_t slv_rd_buf_done:1;
        /** seg_magic_err : R/W; bitpos: [25]; default: 0;
         *  1: The recent magic value in CONF buffer is not right in master DMA seg-trans mode.
         *  0: others.
         */
        uint32_t seg_magic_err:1;
        uint32_t reserved_26:6;
    };
    uint32_t val;
} spi_slv_rdbuf_dlen_reg_t;

/** Type of slv_rd_byte register
 *  1: Enable the DMA CONF phase of current seg-trans operation, which means seg-trans
 *  will start. 0: This is not seg-trans mode.
 */
typedef union {
    struct {
        /** slv_data_bytelen : R/W; bitpos: [19:0]; default: 0;
         *  The full-duplex or half-duplex data byte length of the last SPI transfer in slave
         *  mode. In half-duplex mode, this value is controlled by bits [23:20].
         */
        uint32_t slv_data_bytelen:20;
        /** slv_rddma_bytelen_en : R/W; bitpos: [20]; default: 0;
         *  1: spi_slv_data_bytelen stores data byte length of master-read-slave data length in
         *  DMA controlled mode(Rd_DMA). 0: others
         */
        uint32_t slv_rddma_bytelen_en:1;
        /** slv_wrdma_bytelen_en : R/W; bitpos: [21]; default: 0;
         *  1: spi_slv_data_bytelen stores data byte length of master-write-to-slave data
         *  length in DMA controlled mode(Wr_DMA). 0: others
         */
        uint32_t slv_wrdma_bytelen_en:1;
        /** slv_rdbuf_bytelen_en : R/W; bitpos: [22]; default: 0;
         *  1: spi_slv_data_bytelen stores data byte length of master-read-slave data length in
         *  CPU controlled mode(Rd_BUF). 0: others
         */
        uint32_t slv_rdbuf_bytelen_en:1;
        /** slv_wrbuf_bytelen_en : R/W; bitpos: [23]; default: 0;
         *  1: spi_slv_data_bytelen stores data byte length of master-write-to-slave data
         *  length in CPU controlled mode(Wr_BUF). 0: others
         */
        uint32_t slv_wrbuf_bytelen_en:1;
        /** dma_seg_magic_value : R/W; bitpos: [27:24]; default: 10;
         *  The magic value of BM table in master DMA seg-trans.
         */
        uint32_t dma_seg_magic_value:4;
        uint32_t reserved_28:2;
        /** slv_rd_dma_done : R/W; bitpos: [30]; default: 0;
         *  The interrupt raw bit for the completion of Rd-DMA operation in the slave mode.
         *  Can not be changed by CONF_buf.
         */
        uint32_t slv_rd_dma_done:1;
        /** usr_conf : R/W; bitpos: [31]; default: 0;
         *  1: Enable the DMA CONF phase of current seg-trans operation, which means seg-trans
         *  will start. 0: This is not seg-trans mode.
         */
        uint32_t usr_conf:1;
    };
    uint32_t val;
} spi_slv_rd_byte_reg_t;

/** Type of fsm register
 *  Define the master DMA read byte length in non seg-trans or seg-trans mode. Invalid
 *  when spi_rx_eof_en is 0. Can be configured in CONF state..
 */
typedef union {
    struct {
        /** st : RO; bitpos: [3:0]; default: 0;
         *  The status of spi state machine. 0: idle state, 1: preparation state, 2: send
         *  command state, 3: send data state, 4: red data state, 5:write data state, 6: wait
         *  state, 7: done state.
         */
        uint32_t st:4;
        uint32_t reserved_4:8;
        /** mst_dma_rd_bytelen : R/W; bitpos: [31:12]; default: 0;
         *  Define the master DMA read byte length in non seg-trans or seg-trans mode. Invalid
         *  when spi_rx_eof_en is 0. Can be configured in CONF state..
         */
        uint32_t mst_dma_rd_bytelen:20;
    };
    uint32_t val;
} spi_fsm_reg_t;

/** Type of hold register
 *  reserved
 */
typedef union {
    struct {
        /** int_hold_ena : R/W; bitpos: [1:0]; default: 0;
         *  This register is for two SPI masters to share the same cs clock and data signals.
         *  The bits of one SPI are set, if the other SPI is busy, the SPI will be hold. 1(3):
         *  hold at  idle  phase 2: hold at  prepare  phase. Can be configured in CONF state.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t int_hold_ena:2;
        /** hold_val_reg : R/W; bitpos: [2]; default: 0;
         *  spi hold output value, which should be used with spi_hold_out_en. Can be configured
         *  in CONF state.
         */
        uint32_t hold_val_reg:1;
        /** hold_out_en : R/W; bitpos: [3]; default: 0;
         *  Enable set spi output hold value to spi_hold_reg. It can be used to hold spi state
         *  machine with spi_ext_hold_en and other usr hold signals. Can be configured in CONF
         *  state.
         */
        uint32_t hold_out_en:1;
        /** hold_out_time : R/W; bitpos: [6:4]; default: 0;
         *  set the hold cycles of output spi_hold signal when spi_hold_out_en is enable. Can
         *  be configured in CONF state.
         */
        uint32_t hold_out_time:3;
        /** dma_seg_trans_done : R/W; bitpos: [7]; default: 0;
         *  1:  spi master DMA full-duplex/half-duplex seg-trans ends or slave half-duplex
         *  seg-trans ends. And data has been pushed to corresponding memory.  0:  seg-trans is
         *  not ended or not occurred.  Can not be changed by CONF_buf.
         */
        uint32_t dma_seg_trans_done:1;
        uint32_t reserved_8:24;
    };
    uint32_t val;
} spi_hold_reg_t;

/** Type of dma_conf register
 *  reserved
 */
typedef union {
    struct {
        uint32_t reserved_0:2;
        /** in_rst : R/W; bitpos: [2]; default: 0;
         *  The bit is used to reset in dma fsm and in data fifo pointer.
         */
        uint32_t in_rst:1;
        /** out_rst : R/W; bitpos: [3]; default: 0;
         *  The bit is used to reset out dma fsm and out data fifo pointer.
         */
        uint32_t out_rst:1;
        /** ahbm_fifo_rst : R/W; bitpos: [4]; default: 0;
         *  Reset spi dma ahb master fifo pointer.
         */
        uint32_t ahbm_fifo_rst:1;
        /** ahbm_rst : R/W; bitpos: [5]; default: 0;
         *  Reset spi dma ahb master.
         */
        uint32_t ahbm_rst:1;
        /** in_loop_test : R/W; bitpos: [6]; default: 0;
         *  Set bit to test in link.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t in_loop_test:1;
        /** out_loop_test : R/W; bitpos: [7]; default: 0;
         *  Set bit to test out link.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t out_loop_test:1;
        /** out_auto_wrback : R/W; bitpos: [8]; default: 0;
         *  when the bit is set, DMA continue to use the next inlink node when the length of
         *  inlink is 0.
         *  This field is only for internal debugging purposes. Do not use it in applications.
         */
        uint32_t out_auto_wrback:1;
        /** out_eof_mode : R/W; bitpos: [9]; default: 1;
         *  out eof flag generation mode . 1: when dma pop all data from fifo  0:when ahb push
         *  all data to fifo.
         */
        uint32_t out_eof_mode:1;
        /** outdscr_burst_en : R/W; bitpos: [10]; default: 0;
         *  read descriptor use burst mode when read data for memory.
         */
        uint32_t outdscr_burst_en:1;
        /** indscr_burst_en : R/W; bitpos: [11]; default: 0;
         *  read descriptor use burst mode when write data to memory.
         */
        uint32_t indscr_burst_en:1;
        /** out_data_burst_en : R/W; bitpos: [12]; default: 0;
         *  spi dma read data from memory in burst mode.
         */
        uint32_t out_data_burst_en:1;
        /** mem_trans_en : R/W; bitpos: [13]; default: 0; */
        uint32_t mem_trans_en:1;
        /** dma_rx_stop : R/W; bitpos: [14]; default: 0;
         *  spi dma read data stop  when in continue tx/rx mode.
         */
        uint32_t dma_rx_stop:1;
        /** dma_tx_stop : R/W; bitpos: [15]; default: 0;
         *  spi dma write data stop when in continue tx/rx mode.
         */
        uint32_t dma_tx_stop:1;
        /** dma_continue : R/W; bitpos: [16]; default: 0;
         *  spi dma continue tx/rx data.
         */
        uint32_t dma_continue:1;
        /** slv_last_seg_pop_clr : R/W; bitpos: [17]; default: 0;
         *  1: Clear spi_slv_seg_frt_pop_mask. 0 : others
         */
        uint32_t slv_last_seg_pop_clr:1;
        /** dma_slv_seg_trans_en : R/W; bitpos: [18]; default: 0;
         *  Enable dma segment transfer in spi dma half slave mode. 1: enable. 0: disable.
         */
        uint32_t dma_slv_seg_trans_en:1;
        /** slv_rx_seg_trans_clr_en : R/W; bitpos: [19]; default: 0;
         *  1: spi_dma_infifo_full_vld is cleared by spi slave cmd 5. 0:
         *  spi_dma_infifo_full_vld is cleared by spi_trans_done.
         */
        uint32_t slv_rx_seg_trans_clr_en:1;
        /** slv_tx_seg_trans_clr_en : R/W; bitpos: [20]; default: 0;
         *  1: spi_dma_outfifo_empty_vld is cleared by spi slave cmd 6. 0:
         *  spi_dma_outfifo_empty_vld is cleared by spi_trans_done.
         */
        uint32_t slv_tx_seg_trans_clr_en:1;
        /** rx_eof_en : R/W; bitpos: [21]; default: 0;
         *  1: spi_dma_inlink_eof is set when the number of dma pushed data bytes is equal to
         *  the value of spi_slv/mst_dma_rd_bytelen[19:0] in spi dma transition.  0:
         *  spi_dma_inlink_eof is set by spi_trans_done in non-seg-trans or
         *  spi_dma_seg_trans_done in seg-trans.
         */
        uint32_t rx_eof_en:1;
        /** dma_infifo_full_clr : R/W; bitpos: [22]; default: 0;
         *  1:Clear spi_dma_infifo_full_vld. 0: Do not control it.
         */
        uint32_t dma_infifo_full_clr:1;
        /** dma_outfifo_empty_clr : R/W; bitpos: [23]; default: 0;
         *  1:Clear spi_dma_outfifo_empty_vld. 0: Do not control it.
         */
        uint32_t dma_outfifo_empty_clr:1;
        uint32_t reserved_24:2;
        /** ext_mem_bk_size : R/W; bitpos: [27:26]; default: 0;
         *  Select the external memory block size.
         */
        uint32_t ext_mem_bk_size:2;
        /** dma_seg_trans_clr : R/W; bitpos: [28]; default: 0;
         *  1: End slave seg-trans, which acts as 0x05 command. 2 or more end seg-trans signals
         *  will induce error in DMA RX.
         */
        uint32_t dma_seg_trans_clr:1;
        uint32_t reserved_29:3;
    };
    uint32_t val;
} spi_dma_conf_reg_t;

/** Type of dma_out_link register
 *  spi dma write data status bit.
 */
typedef union {
    struct {
        /** outlink_addr : R/W; bitpos: [19:0]; default: 0;
         *  The address of the first outlink descriptor.
         */
        uint32_t outlink_addr:20;
        uint32_t reserved_20:8;
        /** outlink_stop : R/W; bitpos: [28]; default: 0;
         *  Set the bit to stop to use outlink descriptor.
         */
        uint32_t outlink_stop:1;
        /** outlink_start : R/W; bitpos: [29]; default: 0;
         *  Set the bit to start to use outlink descriptor.
         */
        uint32_t outlink_start:1;
        /** outlink_restart : R/W; bitpos: [30]; default: 0;
         *  Set the bit to mount on new outlink descriptors.
         */
        uint32_t outlink_restart:1;
        /** dma_tx_ena : R/W; bitpos: [31]; default: 0;
         *  spi dma write data status bit.
         */
        uint32_t dma_tx_ena:1;
    };
    uint32_t val;
} spi_dma_out_link_reg_t;

/** Type of dma_in_link register
 *  spi dma read data status bit.
 */
typedef union {
    struct {
        /** inlink_addr : R/W; bitpos: [19:0]; default: 0;
         *  The address of the first inlink descriptor.
         */
        uint32_t inlink_addr:20;
        /** inlink_auto_ret : R/W; bitpos: [20]; default: 0;
         *  when the bit is set, the inlink descriptor returns to the first link node when a
         *  packet is error.
         */
        uint32_t inlink_auto_ret:1;
        uint32_t reserved_21:7;
        /** inlink_stop : R/W; bitpos: [28]; default: 0;
         *  Set the bit to stop to use inlink descriptor.
         */
        uint32_t inlink_stop:1;
        /** inlink_start : R/W; bitpos: [29]; default: 0;
         *  Set the bit to start to use inlink descriptor.
         */
        uint32_t inlink_start:1;
        /** inlink_restart : R/W; bitpos: [30]; default: 0;
         *  Set the bit to mount on new inlink descriptors.
         */
        uint32_t inlink_restart:1;
        /** dma_rx_ena : R/W; bitpos: [31]; default: 0;
         *  spi dma read data status bit.
         */
        uint32_t dma_rx_ena:1;
    };
    uint32_t val;
} spi_dma_in_link_reg_t;

/** Type of dma_int_ena register
 *  reserved
 */
typedef union {
    struct {
        /** inlink_dscr_empty_int_ena : R/W; bitpos: [0]; default: 0;
         *  The enable bit for lack of enough inlink descriptors. Can be configured in CONF
         *  state.
         */
        uint32_t inlink_dscr_empty_int_ena:1;
        /** outlink_dscr_error_int_ena : R/W; bitpos: [1]; default: 0;
         *  The enable bit for outlink descriptor error. Can be configured in CONF state.
         */
        uint32_t outlink_dscr_error_int_ena:1;
        /** inlink_dscr_error_int_ena : R/W; bitpos: [2]; default: 0;
         *  The enable bit for inlink descriptor error. Can be configured in CONF state.
         */
        uint32_t inlink_dscr_error_int_ena:1;
        /** in_done_int_ena : R/W; bitpos: [3]; default: 0;
         *  The enable bit for completing usage of a inlink descriptor. Can be configured in
         *  CONF state.
         */
        uint32_t in_done_int_ena:1;
        /** in_err_eof_int_ena : R/W; bitpos: [4]; default: 0;
         *  The enable bit for receiving error. Can be configured in CONF state.
         */
        uint32_t in_err_eof_int_ena:1;
        /** in_suc_eof_int_ena : R/W; bitpos: [5]; default: 0;
         *  The enable bit for completing receiving all the packets from host. Can be
         *  configured in CONF state.
         */
        uint32_t in_suc_eof_int_ena:1;
        /** out_done_int_ena : R/W; bitpos: [6]; default: 0;
         *  The enable bit for completing usage of a outlink descriptor . Can be configured in
         *  CONF state.
         */
        uint32_t out_done_int_ena:1;
        /** out_eof_int_ena : R/W; bitpos: [7]; default: 0;
         *  The enable bit for sending a packet to host done. Can be configured in CONF state.
         */
        uint32_t out_eof_int_ena:1;
        /** out_total_eof_int_ena : R/W; bitpos: [8]; default: 0;
         *  The enable bit for sending all the packets to host done. Can be configured in CONF
         *  state.
         */
        uint32_t out_total_eof_int_ena:1;
        /** infifo_full_err_int_ena : R/W; bitpos: [9]; default: 0;
         *  The enable bit for infifo full error interrupt.
         */
        uint32_t infifo_full_err_int_ena:1;
        /** outfifo_empty_err_int_ena : R/W; bitpos: [10]; default: 0;
         *  The enable bit for outfifo empty error interrupt.
         */
        uint32_t outfifo_empty_err_int_ena:1;
        /** slv_cmd6_int_ena : R/W; bitpos: [11]; default: 0;
         *  The enable bit for SPI slave CMD6 interrupt.
         */
        uint32_t slv_cmd6_int_ena:1;
        /** slv_cmd7_int_ena : R/W; bitpos: [12]; default: 0;
         *  The enable bit for SPI slave CMD7 interrupt.
         */
        uint32_t slv_cmd7_int_ena:1;
        /** slv_cmd8_int_ena : R/W; bitpos: [13]; default: 0;
         *  The enable bit for SPI slave CMD8 interrupt.
         */
        uint32_t slv_cmd8_int_ena:1;
        /** slv_cmd9_int_ena : R/W; bitpos: [14]; default: 0;
         *  The enable bit for SPI slave CMD9 interrupt.
         */
        uint32_t slv_cmd9_int_ena:1;
        /** slv_cmda_int_ena : R/W; bitpos: [15]; default: 0;
         *  The enable bit for SPI slave CMDA interrupt.
         */
        uint32_t slv_cmda_int_ena:1;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_dma_int_ena_reg_t;

/** Type of dma_int_raw register
 *  reserved
 */
typedef union {
    struct {
        /** inlink_dscr_empty_int_raw : RO; bitpos: [0]; default: 0;
         *  The raw bit for lack of enough inlink descriptors. Can be configured in CONF state.
         */
        uint32_t inlink_dscr_empty_int_raw:1;
        /** outlink_dscr_error_int_raw : RO; bitpos: [1]; default: 0;
         *  The raw bit for outlink descriptor error. Can be configured in CONF state.
         */
        uint32_t outlink_dscr_error_int_raw:1;
        /** inlink_dscr_error_int_raw : RO; bitpos: [2]; default: 0;
         *  The raw bit for inlink descriptor error. Can be configured in CONF state.
         */
        uint32_t inlink_dscr_error_int_raw:1;
        /** in_done_int_raw : RO; bitpos: [3]; default: 0;
         *  The raw bit for completing usage of a inlink descriptor. Can be configured in CONF
         *  state.
         */
        uint32_t in_done_int_raw:1;
        /** in_err_eof_int_raw : RO; bitpos: [4]; default: 0;
         *  The raw bit for receiving error. Can be configured in CONF state.
         */
        uint32_t in_err_eof_int_raw:1;
        /** in_suc_eof_int_raw : RO; bitpos: [5]; default: 0;
         *  The raw bit for completing receiving all the packets from host. Can be configured
         *  in CONF state.
         */
        uint32_t in_suc_eof_int_raw:1;
        /** out_done_int_raw : RO; bitpos: [6]; default: 0;
         *  The raw bit for completing usage of a outlink descriptor. Can be configured in CONF
         *  state.
         */
        uint32_t out_done_int_raw:1;
        /** out_eof_int_raw : RO; bitpos: [7]; default: 0;
         *  The raw bit for sending a packet to host done. Can be configured in CONF state.
         */
        uint32_t out_eof_int_raw:1;
        /** out_total_eof_int_raw : RO; bitpos: [8]; default: 0;
         *  The raw bit for sending all the packets to host done. Can be configured in CONF
         *  state.
         */
        uint32_t out_total_eof_int_raw:1;
        /** infifo_full_err_int_raw : RO; bitpos: [9]; default: 0;
         *  1:spi_dma_infifo_full and spi_push_data_prep are valid, which means that DMA Rx
         *  buffer is full but push is valid.  0: Others.  Can not be changed by CONF_buf.
         */
        uint32_t infifo_full_err_int_raw:1;
        /** outfifo_empty_err_int_raw : RO; bitpos: [10]; default: 0;
         *  1:spi_dma_outfifo_empty and spi_pop_data_prep are valid, which means that there is
         *  no data to pop but pop is valid.  0: Others.  Can not be changed by CONF_buf.
         */
        uint32_t outfifo_empty_err_int_raw:1;
        /** slv_cmd6_int_raw : R/W; bitpos: [11]; default: 0;
         *  The raw bit for SPI slave CMD6 interrupt.
         */
        uint32_t slv_cmd6_int_raw:1;
        /** slv_cmd7_int_raw : R/W; bitpos: [12]; default: 0;
         *  The raw bit for SPI slave CMD7 interrupt.
         */
        uint32_t slv_cmd7_int_raw:1;
        /** slv_cmd8_int_raw : R/W; bitpos: [13]; default: 0;
         *  The raw bit for SPI slave CMD8 interrupt.
         */
        uint32_t slv_cmd8_int_raw:1;
        /** slv_cmd9_int_raw : R/W; bitpos: [14]; default: 0;
         *  The raw bit for SPI slave CMD9 interrupt.
         */
        uint32_t slv_cmd9_int_raw:1;
        /** slv_cmda_int_raw : R/W; bitpos: [15]; default: 0;
         *  The raw bit for SPI slave CMDA interrupt.
         */
        uint32_t slv_cmda_int_raw:1;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_dma_int_raw_reg_t;

/** Type of dma_int_st register
 *  reserved
 */
typedef union {
    struct {
        /** inlink_dscr_empty_int_st : RO; bitpos: [0]; default: 0;
         *  The status bit for lack of enough inlink descriptors.
         */
        uint32_t inlink_dscr_empty_int_st:1;
        /** outlink_dscr_error_int_st : RO; bitpos: [1]; default: 0;
         *  The status bit for outlink descriptor error.
         */
        uint32_t outlink_dscr_error_int_st:1;
        /** inlink_dscr_error_int_st : RO; bitpos: [2]; default: 0;
         *  The status bit for inlink descriptor error.
         */
        uint32_t inlink_dscr_error_int_st:1;
        /** in_done_int_st : RO; bitpos: [3]; default: 0;
         *  The status bit for completing usage of a inlink descriptor.
         */
        uint32_t in_done_int_st:1;
        /** in_err_eof_int_st : RO; bitpos: [4]; default: 0;
         *  The status bit for receiving error.
         */
        uint32_t in_err_eof_int_st:1;
        /** in_suc_eof_int_st : RO; bitpos: [5]; default: 0;
         *  The status bit for completing receiving all the packets from host.
         */
        uint32_t in_suc_eof_int_st:1;
        /** out_done_int_st : RO; bitpos: [6]; default: 0;
         *  The status bit for completing usage of a outlink descriptor.
         */
        uint32_t out_done_int_st:1;
        /** out_eof_int_st : RO; bitpos: [7]; default: 0;
         *  The status bit for sending a packet to host done.
         */
        uint32_t out_eof_int_st:1;
        /** out_total_eof_int_st : RO; bitpos: [8]; default: 0;
         *  The status bit for sending all the packets to host done.
         */
        uint32_t out_total_eof_int_st:1;
        /** infifo_full_err_int_st : RO; bitpos: [9]; default: 0;
         *  The status bit for infifo full error.
         */
        uint32_t infifo_full_err_int_st:1;
        /** outfifo_empty_err_int_st : RO; bitpos: [10]; default: 0;
         *  The status bit for outfifo empty error.
         */
        uint32_t outfifo_empty_err_int_st:1;
        /** slv_cmd6_int_st : R/W; bitpos: [11]; default: 0;
         *  The status bit for SPI slave CMD6 interrupt.
         */
        uint32_t slv_cmd6_int_st:1;
        /** slv_cmd7_int_st : R/W; bitpos: [12]; default: 0;
         *  The status bit for SPI slave CMD7 interrupt.
         */
        uint32_t slv_cmd7_int_st:1;
        /** slv_cmd8_int_st : R/W; bitpos: [13]; default: 0;
         *  The status bit for SPI slave CMD8 interrupt.
         */
        uint32_t slv_cmd8_int_st:1;
        /** slv_cmd9_int_st : R/W; bitpos: [14]; default: 0;
         *  The status bit for SPI slave CMD9 interrupt.
         */
        uint32_t slv_cmd9_int_st:1;
        /** slv_cmda_int_st : R/W; bitpos: [15]; default: 0;
         *  The status bit for SPI slave CMDA interrupt.
         */
        uint32_t slv_cmda_int_st:1;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_dma_int_st_reg_t;

/** Type of dma_int_clr register
 *  reserved
 */
typedef union {
    struct {
        /** inlink_dscr_empty_int_clr : R/W; bitpos: [0]; default: 0;
         *  The clear bit for lack of enough inlink descriptors. Can be configured in CONF
         *  state.
         */
        uint32_t inlink_dscr_empty_int_clr:1;
        /** outlink_dscr_error_int_clr : R/W; bitpos: [1]; default: 0;
         *  The clear bit for outlink descriptor error. Can be configured in CONF state.
         */
        uint32_t outlink_dscr_error_int_clr:1;
        /** inlink_dscr_error_int_clr : R/W; bitpos: [2]; default: 0;
         *  The clear bit for inlink descriptor error. Can be configured in CONF state.
         */
        uint32_t inlink_dscr_error_int_clr:1;
        /** in_done_int_clr : R/W; bitpos: [3]; default: 0;
         *  The clear bit for completing usage of a inlink descriptor. Can be configured in
         *  CONF state.
         */
        uint32_t in_done_int_clr:1;
        /** in_err_eof_int_clr : R/W; bitpos: [4]; default: 0;
         *  The clear bit for receiving error. Can be configured in CONF state.
         */
        uint32_t in_err_eof_int_clr:1;
        /** in_suc_eof_int_clr : R/W; bitpos: [5]; default: 0;
         *  The clear bit for completing receiving all the packets from host. Can be configured
         *  in CONF state.
         */
        uint32_t in_suc_eof_int_clr:1;
        /** out_done_int_clr : R/W; bitpos: [6]; default: 0;
         *  The clear bit for completing usage of a outlink descriptor. Can be configured in
         *  CONF state.
         */
        uint32_t out_done_int_clr:1;
        /** out_eof_int_clr : R/W; bitpos: [7]; default: 0;
         *  The clear bit for sending a packet to host done. Can be configured in CONF state.
         */
        uint32_t out_eof_int_clr:1;
        /** out_total_eof_int_clr : R/W; bitpos: [8]; default: 0;
         *  The clear bit for sending all the packets to host done. Can be configured in CONF
         *  state.
         */
        uint32_t out_total_eof_int_clr:1;
        /** infifo_full_err_int_clr : R/W; bitpos: [9]; default: 0;
         *  1: Clear spi_dma_infifo_full_err. 0: not valid. Can be changed by CONF_buf.
         */
        uint32_t infifo_full_err_int_clr:1;
        /** outfifo_empty_err_int_clr : R/W; bitpos: [10]; default: 0;
         *  1: Clear spi_dma_outfifo_empty_err signal. 0: not valid. Can be changed by CONF_buf.
         */
        uint32_t outfifo_empty_err_int_clr:1;
        /** slv_cmd6_int_clr : R/W; bitpos: [11]; default: 0;
         *  The clear bit for SPI slave CMD6 interrupt.
         */
        uint32_t slv_cmd6_int_clr:1;
        /** slv_cmd7_int_clr : R/W; bitpos: [12]; default: 0;
         *  The clear bit for SPI slave CMD7 interrupt.
         */
        uint32_t slv_cmd7_int_clr:1;
        /** slv_cmd8_int_clr : R/W; bitpos: [13]; default: 0;
         *  The clear bit for SPI slave CMD8 interrupt.
         */
        uint32_t slv_cmd8_int_clr:1;
        /** slv_cmd9_int_clr : R/W; bitpos: [14]; default: 0;
         *  The clear bit for SPI slave CMD9 interrupt.
         */
        uint32_t slv_cmd9_int_clr:1;
        /** slv_cmda_int_clr : R/W; bitpos: [15]; default: 0;
         *  The clear bit for SPI slave CMDA interrupt.
         */
        uint32_t slv_cmda_int_clr:1;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_dma_int_clr_reg_t;

/** Type of in_err_eof_des_addr register
 *  The inlink descriptor address when spi dma produce receiving error.
 */
typedef union {
    struct {
        /** dma_in_err_eof_des_addr : RO; bitpos: [31:0]; default: 0;
         *  The inlink descriptor address when spi dma produce receiving error.
         */
        uint32_t dma_in_err_eof_des_addr:32;
    };
    uint32_t val;
} spi_in_err_eof_des_addr_reg_t;

/** Type of in_suc_eof_des_addr register
 *  The last inlink descriptor address when spi dma produce from_suc_eof.
 */
typedef union {
    struct {
        /** dma_in_suc_eof_des_addr : RO; bitpos: [31:0]; default: 0;
         *  The last inlink descriptor address when spi dma produce from_suc_eof.
         */
        uint32_t dma_in_suc_eof_des_addr:32;
    };
    uint32_t val;
} spi_in_suc_eof_des_addr_reg_t;

/** Type of inlink_dscr register
 *  The content of current in descriptor pointer.
 */
typedef union {
    struct {
        /** dma_inlink_dscr : RO; bitpos: [31:0]; default: 0;
         *  The content of current in descriptor pointer.
         */
        uint32_t dma_inlink_dscr:32;
    };
    uint32_t val;
} spi_inlink_dscr_reg_t;

/** Type of inlink_dscr_bf0 register
 *  The content of next in descriptor pointer.
 */
typedef union {
    struct {
        /** dma_inlink_dscr_bf0 : RO; bitpos: [31:0]; default: 0;
         *  The content of next in descriptor pointer.
         */
        uint32_t dma_inlink_dscr_bf0:32;
    };
    uint32_t val;
} spi_inlink_dscr_bf0_reg_t;

/** Type of inlink_dscr_bf1 register
 *  The content of current in descriptor data buffer pointer.
 */
typedef union {
    struct {
        /** dma_inlink_dscr_bf1 : RO; bitpos: [31:0]; default: 0;
         *  The content of current in descriptor data buffer pointer.
         */
        uint32_t dma_inlink_dscr_bf1:32;
    };
    uint32_t val;
} spi_inlink_dscr_bf1_reg_t;

/** Type of out_eof_bfr_des_addr register
 *  The address of buffer relative to the outlink descriptor that produce eof.
 */
typedef union {
    struct {
        /** dma_out_eof_bfr_des_addr : RO; bitpos: [31:0]; default: 0;
         *  The address of buffer relative to the outlink descriptor that produce eof.
         */
        uint32_t dma_out_eof_bfr_des_addr:32;
    };
    uint32_t val;
} spi_out_eof_bfr_des_addr_reg_t;

/** Type of out_eof_des_addr register
 *  The last outlink descriptor address when spi dma produce to_eof.
 */
typedef union {
    struct {
        /** dma_out_eof_des_addr : RO; bitpos: [31:0]; default: 0;
         *  The last outlink descriptor address when spi dma produce to_eof.
         */
        uint32_t dma_out_eof_des_addr:32;
    };
    uint32_t val;
} spi_out_eof_des_addr_reg_t;

/** Type of outlink_dscr register
 *  The content of current out descriptor pointer.
 */
typedef union {
    struct {
        /** dma_outlink_dscr : RO; bitpos: [31:0]; default: 0;
         *  The content of current out descriptor pointer.
         */
        uint32_t dma_outlink_dscr:32;
    };
    uint32_t val;
} spi_outlink_dscr_reg_t;

/** Type of outlink_dscr_bf0 register
 *  The content of next out descriptor pointer.
 */
typedef union {
    struct {
        /** dma_outlink_dscr_bf0 : RO; bitpos: [31:0]; default: 0;
         *  The content of next out descriptor pointer.
         */
        uint32_t dma_outlink_dscr_bf0:32;
    };
    uint32_t val;
} spi_outlink_dscr_bf0_reg_t;

/** Type of outlink_dscr_bf1 register
 *  The content of current out descriptor data buffer pointer.
 */
typedef union {
    struct {
        /** dma_outlink_dscr_bf1 : RO; bitpos: [31:0]; default: 0;
         *  The content of current out descriptor data buffer pointer.
         */
        uint32_t dma_outlink_dscr_bf1:32;
    };
    uint32_t val;
} spi_outlink_dscr_bf1_reg_t;

/** Type of dma_outstatus register
 *  SPI dma outfifo is empty.
 */
typedef union {
    struct {
        /** dma_outdscr_addr : RO; bitpos: [17:0]; default: 0;
         *  SPI dma out descriptor address.
         */
        uint32_t dma_outdscr_addr:18;
        /** dma_outdscr_state : RO; bitpos: [19:18]; default: 0;
         *  SPI dma out descriptor state.
         */
        uint32_t dma_outdscr_state:2;
        /** dma_out_state : RO; bitpos: [22:20]; default: 0;
         *  SPI dma out data state.
         */
        uint32_t dma_out_state:3;
        /** dma_outfifo_cnt : RO; bitpos: [29:23]; default: 0;
         *  The remains of SPI dma outfifo data.
         */
        uint32_t dma_outfifo_cnt:7;
        /** dma_outfifo_full : RO; bitpos: [30]; default: 0;
         *  SPI dma outfifo is full.
         */
        uint32_t dma_outfifo_full:1;
        /** dma_outfifo_empty : RO; bitpos: [31]; default: 1;
         *  SPI dma outfifo is empty.
         */
        uint32_t dma_outfifo_empty:1;
    };
    uint32_t val;
} spi_dma_out_status_reg_t;

/** Type of dma_instatus register
 *  SPI dma infifo is empty.
 */
typedef union {
    struct {
        /** dma_indscr_addr : RO; bitpos: [17:0]; default: 0;
         *  SPI dma in descriptor address.
         */
        uint32_t dma_indscr_addr:18;
        /** dma_indscr_state : RO; bitpos: [19:18]; default: 0;
         *  SPI dma in descriptor state.
         */
        uint32_t dma_indscr_state:2;
        /** dma_in_state : RO; bitpos: [22:20]; default: 0;
         *  SPI dma in data state.
         */
        uint32_t dma_in_state:3;
        /** dma_infifo_cnt : RO; bitpos: [29:23]; default: 0;
         *  The remains of SPI dma infifo data.
         */
        uint32_t dma_infifo_cnt:7;
        /** dma_infifo_full : RO; bitpos: [30]; default: 0;
         *  SPI dma infifo is full.
         */
        uint32_t dma_infifo_full:1;
        /** dma_infifo_empty : RO; bitpos: [31]; default: 1;
         *  SPI dma infifo is empty.
         */
        uint32_t dma_infifo_empty:1;
    };
    uint32_t val;
} spi_dma_in_status_reg_t;

/** Type of w0 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf0 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf0:32;
    };
    uint32_t val;
} spi_w0_reg_t;

/** Type of w1 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf1 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf1:32;
    };
    uint32_t val;
} spi_w1_reg_t;

/** Type of w2 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf2 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf2:32;
    };
    uint32_t val;
} spi_w2_reg_t;

/** Type of w3 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf3 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf3:32;
    };
    uint32_t val;
} spi_w3_reg_t;

/** Type of w4 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf4 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf4:32;
    };
    uint32_t val;
} spi_w4_reg_t;

/** Type of w5 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf5 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf5:32;
    };
    uint32_t val;
} spi_w5_reg_t;

/** Type of w6 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf6 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf6:32;
    };
    uint32_t val;
} spi_w6_reg_t;

/** Type of w7 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf7 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf7:32;
    };
    uint32_t val;
} spi_w7_reg_t;

/** Type of w8 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf8 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf8:32;
    };
    uint32_t val;
} spi_w8_reg_t;

/** Type of w9 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf9 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf9:32;
    };
    uint32_t val;
} spi_w9_reg_t;

/** Type of w10 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf10 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf10:32;
    };
    uint32_t val;
} spi_w10_reg_t;

/** Type of w11 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf11 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf11:32;
    };
    uint32_t val;
} spi_w11_reg_t;

/** Type of w12 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf12 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf12:32;
    };
    uint32_t val;
} spi_w12_reg_t;

/** Type of w13 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf13 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf13:32;
    };
    uint32_t val;
} spi_w13_reg_t;

/** Type of w14 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf14 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf14:32;
    };
    uint32_t val;
} spi_w14_reg_t;

/** Type of w15 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf15 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf15:32;
    };
    uint32_t val;
} spi_w15_reg_t;

/** Type of w16 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf16 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf16:32;
    };
    uint32_t val;
} spi_w16_reg_t;

/** Type of w17 register
 *  data buffer
 */
typedef union {
    struct {
        /** buf17 : R/W; bitpos: [31:0]; default: 0;
         *  data buffer
         */
        uint32_t buf17:32;
    };
    uint32_t val;
} spi_w17_reg_t;

/** Type of din_mode register
 *  reserved
 */
typedef union {
    struct {
        /** din0_mode : R/W; bitpos: [2:0]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din0_mode:3;
        /** din1_mode : R/W; bitpos: [5:3]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din1_mode:3;
        /** din2_mode : R/W; bitpos: [8:6]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din2_mode:3;
        /** din3_mode : R/W; bitpos: [11:9]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din3_mode:3;
        /** din4_mode : R/W; bitpos: [14:12]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din4_mode:3;
        /** din5_mode : R/W; bitpos: [17:15]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din5_mode:3;
        /** din6_mode : R/W; bitpos: [20:18]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din6_mode:3;
        /** din7_mode : R/W; bitpos: [23:21]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: input without delayed, 1:
         *  input with the posedge of clk_apb,2 input with the negedge of clk_apb, 3: input
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t din7_mode:3;
        /** timing_clk_ena : R/W; bitpos: [24]; default: 0;
         *  1:enable hclk in spi_timing.v.  0: disable it. Can be configured in CONF state.
         */
        uint32_t timing_clk_ena:1;
        uint32_t reserved_25:7;
    };
    uint32_t val;
} spi_din_mode_reg_t;

/** Type of din_num register
 *  reserved
 */
typedef union {
    struct {
        /** din0_num : R/W; bitpos: [1:0]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din0_num:2;
        /** din1_num : R/W; bitpos: [3:2]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din1_num:2;
        /** din2_num : R/W; bitpos: [5:4]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din2_num:2;
        /** din3_num : R/W; bitpos: [7:6]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din3_num:2;
        /** din4_num : R/W; bitpos: [9:8]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din4_num:2;
        /** din5_num : R/W; bitpos: [11:10]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din5_num:2;
        /** din6_num : R/W; bitpos: [13:12]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din6_num:2;
        /** din7_num : R/W; bitpos: [15:14]; default: 0;
         *  the input signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,...  Can be configured in CONF state.
         */
        uint32_t din7_num:2;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_din_num_reg_t;

/** Type of dout_mode register
 *  reserved
 */
typedef union {
    struct {
        /** dout0_mode : R/W; bitpos: [2:0]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout0_mode:3;
        /** dout1_mode : R/W; bitpos: [5:3]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout1_mode:3;
        /** dout2_mode : R/W; bitpos: [8:6]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout2_mode:3;
        /** dout3_mode : R/W; bitpos: [11:9]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout3_mode:3;
        /** dout4_mode : R/W; bitpos: [14:12]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout4_mode:3;
        /** dout5_mode : R/W; bitpos: [17:15]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout5_mode:3;
        /** dout6_mode : R/W; bitpos: [20:18]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout6_mode:3;
        /** dout7_mode : R/W; bitpos: [23:21]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t dout7_mode:3;
        uint32_t reserved_24:8;
    };
    uint32_t val;
} spi_dout_mode_reg_t;

/** Type of dout_num register
 *  reserved
 */
typedef union {
    struct {
        /** dout0_num : R/W; bitpos: [1:0]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout0_num:2;
        /** dout1_num : R/W; bitpos: [3:2]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout1_num:2;
        /** dout2_num : R/W; bitpos: [5:4]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout2_num:2;
        /** dout3_num : R/W; bitpos: [7:6]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout3_num:2;
        /** dout4_num : R/W; bitpos: [9:8]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout4_num:2;
        /** dout5_num : R/W; bitpos: [11:10]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout5_num:2;
        /** dout6_num : R/W; bitpos: [13:12]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout6_num:2;
        /** dout7_num : R/W; bitpos: [15:14]; default: 0;
         *  the output signals are delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t dout7_num:2;
        uint32_t reserved_16:16;
    };
    uint32_t val;
} spi_dout_num_reg_t;

/** Type of lcd_ctrl register
 *  1: Enable LCD mode output vsync, hsync, de. 0: Disable. Can be configured in CONF
 *  state.
 */
typedef union {
    struct {
        /** lcd_hb_front : R/W; bitpos: [10:0]; default: 0;
         *  It is the horizontal blank front porch of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_hb_front:11;
        /** lcd_va_height : R/W; bitpos: [20:11]; default: 0;
         *  It is the vertical active height of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_va_height:10;
        /** lcd_vt_height : R/W; bitpos: [30:21]; default: 0;
         *  It is the vertical total height of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_vt_height:10;
        /** lcd_mode_en : R/W; bitpos: [31]; default: 0;
         *  1: Enable LCD mode output vsync, hsync, de. 0: Disable. Can be configured in CONF
         *  state.
         */
        uint32_t lcd_mode_en:1;
    };
    uint32_t val;
} spi_lcd_ctrl_reg_t;

/** Type of lcd_ctrl1 register
 *  It is the horizontal total width of a frame. Can be configured in CONF state.
 */
typedef union {
    struct {
        /** lcd_vb_front : R/W; bitpos: [7:0]; default: 0;
         *  It is the vertical blank front porch of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_vb_front:8;
        /** lcd_ha_width : R/W; bitpos: [19:8]; default: 0;
         *  It is the horizontal active width of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_ha_width:12;
        /** lcd_ht_width : R/W; bitpos: [31:20]; default: 0;
         *  It is the horizontal total width of a frame. Can be configured in CONF state.
         */
        uint32_t lcd_ht_width:12;
    };
    uint32_t val;
} spi_lcd_ctrl1_reg_t;

/** Type of lcd_ctrl2 register
 *  It is the position of spi_hsync active pulse in a line. Can be configured in CONF
 *  state.
 */
typedef union {
    struct {
        /** lcd_vsync_width : R/W; bitpos: [6:0]; default: 1;
         *  It is the position of spi_vsync active pulse in a line. Can be configured in CONF
         *  state.
         */
        uint32_t lcd_vsync_width:7;
        /** vsync_idle_pol : R/W; bitpos: [7]; default: 0;
         *  It is the idle value of spi_vsync. Can be configured in CONF state.
         */
        uint32_t vsync_idle_pol:1;
        uint32_t reserved_8:8;
        /** lcd_hsync_width : R/W; bitpos: [22:16]; default: 1;
         *  It is the position of spi_hsync active pulse in a line. Can be configured in CONF
         *  state.
         */
        uint32_t lcd_hsync_width:7;
        /** hsync_idle_pol : R/W; bitpos: [23]; default: 0;
         *  It is the idle value of spi_hsync. Can be configured in CONF state.
         */
        uint32_t hsync_idle_pol:1;
        /** lcd_hsync_position : R/W; bitpos: [31:24]; default: 0;
         *  It is the position of spi_hsync active pulse in a line. Can be configured in CONF
         *  state.
         */
        uint32_t lcd_hsync_position:8;
    };
    uint32_t val;
} spi_lcd_ctrl2_reg_t;

/** Type of lcd_d_mode register
 *  reserved
 */
typedef union {
    struct {
        /** d_dqs_mode : R/W; bitpos: [2:0]; default: 0;
         *  the output spi_dqs is delayed by system clock cycles, 0: output without delayed, 1:
         *  output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3: output
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t d_dqs_mode:3;
        /** d_cd_mode : R/W; bitpos: [5:3]; default: 0;
         *  the output spi_cd is delayed by system clock cycles, 0: output without delayed, 1:
         *  output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3: output
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t d_cd_mode:3;
        /** d_de_mode : R/W; bitpos: [8:6]; default: 0;
         *  the output spi_de is delayed by system clock cycles, 0: output without delayed, 1:
         *  output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3: output
         *  with the spi_clk. Can be configured in CONF state.
         */
        uint32_t d_de_mode:3;
        /** d_hsync_mode : R/W; bitpos: [11:9]; default: 0;
         *  the output spi_hsync is delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t d_hsync_mode:3;
        /** d_vsync_mode : R/W; bitpos: [14:12]; default: 0;
         *  the output spi_vsync is delayed by system clock cycles, 0: output without delayed,
         *  1: output with the posedge of clk_apb,2 output with the negedge of clk_apb, 3:
         *  output with the spi_clk. Can be configured in CONF state.
         */
        uint32_t d_vsync_mode:3;
        /** de_idle_pol : R/W; bitpos: [15]; default: 0;
         *  It is the idle value of spi_de.
         */
        uint32_t de_idle_pol:1;
        /** hs_blank_en : R/W; bitpos: [16]; default: 0;
         *  1: The pulse of spi_hsync is out in vertical blanking lines in seg-trans or one
         *  trans. 0: spi_hsync pulse is valid only in active region lines in seg-trans.
         */
        uint32_t hs_blank_en:1;
        uint32_t reserved_17:15;
    };
    uint32_t val;
} spi_lcd_d_mode_reg_t;

/** Type of lcd_d_num register
 *  reserved
 */
typedef union {
    struct {
        /** d_dqs_num : R/W; bitpos: [1:0]; default: 0;
         *  the output spi_dqs is delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t d_dqs_num:2;
        /** d_cd_num : R/W; bitpos: [3:2]; default: 0;
         *  the output spi_cd is delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t d_cd_num:2;
        /** d_de_num : R/W; bitpos: [5:4]; default: 0;
         *  the output spi_de is delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t d_de_num:2;
        /** d_hsync_num : R/W; bitpos: [7:6]; default: 0;
         *  the output spi_hsync is delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t d_hsync_num:2;
        /** d_vsync_num : R/W; bitpos: [9:8]; default: 0;
         *  the output spi_vsync is delayed by system clock cycles, 0: delayed by 1 cycle, 1:
         *  delayed by 2 cycles,... Can be configured in CONF state.
         */
        uint32_t d_vsync_num:2;
        uint32_t reserved_10:22;
    };
    uint32_t val;
} spi_lcd_d_num_reg_t;

/** Type of reg_date register
 *  reserved
 */
typedef union {
    struct {
        /** date : RW; bitpos: [27:0]; default: 26243648;
         *  SPI register version.
         */
        uint32_t date:28;
        uint32_t reserved_28:4;
    };
    uint32_t val;
} spi_reg_date_reg_t;


typedef struct {
    volatile spi_cmd_reg_t cmd;
    volatile spi_addr_reg_t addr;
    volatile spi_ctrl_reg_t ctrl;
    volatile spi_ctrl1_reg_t ctrl1;
    volatile spi_ctrl2_reg_t ctrl2;
    volatile spi_clock_reg_t clock;
    volatile spi_user_reg_t user;
    volatile spi_user1_reg_t user1;
    volatile spi_user2_reg_t user2;
    volatile spi_mosi_dlen_reg_t mosi_dlen;
    volatile spi_miso_dlen_reg_t miso_dlen;
    volatile spi_misc_reg_t misc;
    volatile spi_slave_reg_t slave;
    volatile spi_slave1_reg_t slave1;
    volatile spi_slv_wrbuf_dlen_reg_t slv_wrbuf_dlen;
    volatile spi_slv_rdbuf_dlen_reg_t slv_rdbuf_dlen;
    volatile spi_slv_rd_byte_reg_t slv_rd_byte;
    volatile spi_fsm_reg_t fsm;
    volatile spi_hold_reg_t hold;
    volatile spi_dma_conf_reg_t dma_conf;
    volatile spi_dma_out_link_reg_t dma_out_link;
    volatile spi_dma_in_link_reg_t dma_in_link;
    volatile spi_dma_int_ena_reg_t dma_int_ena;
    volatile spi_dma_int_raw_reg_t dma_int_raw;
    volatile spi_dma_int_st_reg_t dma_int_st;
    volatile spi_dma_int_clr_reg_t dma_int_clr;
    volatile spi_in_err_eof_des_addr_reg_t in_err_eof_des_addr;
    volatile spi_in_suc_eof_des_addr_reg_t in_suc_eof_des_addr;
    volatile spi_inlink_dscr_reg_t inlink_dscr;
    volatile spi_inlink_dscr_bf0_reg_t inlink_dscr_bf0;
    volatile spi_inlink_dscr_bf1_reg_t inlink_dscr_bf1;
    volatile spi_out_eof_bfr_des_addr_reg_t out_eof_bfr_des_addr;
    volatile spi_out_eof_des_addr_reg_t out_eof_des_addr;
    volatile spi_outlink_dscr_reg_t outlink_dscr;
    volatile spi_outlink_dscr_bf0_reg_t outlink_dscr_bf0;
    volatile spi_outlink_dscr_bf1_reg_t outlink_dscr_bf1;
    volatile spi_dma_out_status_reg_t dma_out_status;
    volatile spi_dma_in_status_reg_t dma_in_status;
    volatile uint32_t data_buf[18];
    volatile spi_din_mode_reg_t din_mode;
    volatile spi_din_num_reg_t din_num;
    volatile spi_dout_mode_reg_t dout_mode;
    volatile spi_dout_num_reg_t dout_num;
    volatile spi_lcd_ctrl_reg_t lcd_ctrl;
    volatile spi_lcd_ctrl1_reg_t lcd_ctrl1;
    volatile spi_lcd_ctrl2_reg_t lcd_ctrl2;
    volatile spi_lcd_d_mode_reg_t lcd_d_mode;
    volatile spi_lcd_d_num_reg_t lcd_d_num;
    uint32_t reserved_104[190];
    volatile spi_reg_date_reg_t reg_date;
} spi_dev_t;

extern spi_dev_t GPSPI2;   //FSPI
extern spi_dev_t GPSPI3;   //HSPI

#ifndef __cplusplus
_Static_assert(sizeof(spi_dev_t) == 0x400, "Invalid size of spi_dev_t structure");
#endif

#ifdef __cplusplus
}
#endif
