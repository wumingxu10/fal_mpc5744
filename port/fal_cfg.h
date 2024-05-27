/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-17     armink       the first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_


#define FAL_PART_HAS_TABLE_CFG 1 //使能自定义分区表


/* ===================== Flash device Configuration ========================= */
extern const struct fal_flash_dev mpc5744_onchip_flash_16k_p1;
extern const struct fal_flash_dev mpc5744_onchip_flash_32k;
extern const struct fal_flash_dev mpc5744_onchip_flash_16k_p2;
extern const struct fal_flash_dev mpc5744_onchip_flash_64k;
extern const struct fal_flash_dev mpc5744_onchip_flash_256k;

/* flash device table */
#define FAL_FLASH_DEV_TABLE                    \
{                                              \
    &mpc5744_onchip_flash_16k_p1,              \
	&mpc5744_onchip_flash_32k,                 \
	&mpc5744_onchip_flash_16k_p2,              \
	&mpc5744_onchip_flash_64k,                 \
	&mpc5744_onchip_flash_256k                 \
}
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#define FAL_PART_TABLE                                                                              \
{                                                                                                   \
	{FAL_PART_MAGIC_WORD,    "init_param",        "onchip_32k",            0,     2*32*1024, 0},    \
    {FAL_PART_MAGIC_WORD,  "bl_start_loc",     "onchip_16k_p2",            0,       16*1024, 0},    \
    {FAL_PART_MAGIC_WORD, "app_start_loc",     "onchip_16k_p2",      16*1024,       16*1024, 0},    \
	{FAL_PART_MAGIC_WORD,            "bl",        "onchip_64k",            0,     3*64*1024, 0},    \
	{FAL_PART_MAGIC_WORD,            "ef",        "onchip_64k",    3*64*1024,     3*64*1024, 0},    \
    {FAL_PART_MAGIC_WORD,         "app_1",       "onchip_256k",       0*1024,     4*256*1024, 0},   \
	{FAL_PART_MAGIC_WORD,         "app_2",       "onchip_256k",   4*256*1024,     4*256*1024, 0},   \
}
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
