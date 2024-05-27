/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-01-26     armink       the first version
 */

#include <fal.h>

#include "flash_c55_driver.h"

#define MPC5744_FLASH_END_ADDRESS (0x091FFFFF)

#define FLASH_TOTAL_BLOCKCOUNT 		20
#define FLASH_LOWPART_BLOCKCOUNT 	4 //16KB
#define FLASH_MIDPART_BLOCKCOUNT 	2 //32KB
#define FLASH_HIGHPART_BLOCKCOUNT 	6 //64KB
#define FLASH_256KPART_BLOCKCOUNT 	8 //256KB

//low
#define FLASH_BLOCKPART_LOW_BLOCKNUM_0_START 	0x00800000
#define FLASH_BLOCKPART_LOW_BLOCKNUM_0_END		0x00803FFF
#define FLASH_BLOCKPART_LOW_BLOCKNUM_1_START 	0x00804000
#define FLASH_BLOCKPART_LOW_BLOCKNUM_1_END 		0x00807FFF
#define FLASH_BLOCKPART_LOW_BLOCKNUM_2_START 	0x00F98000
#define FLASH_BLOCKPART_LOW_BLOCKNUM_2_END 		0x00F9BFFF
#define FLASH_BLOCKPART_LOW_BLOCKNUM_3_START 	0x00F9C000
#define FLASH_BLOCKPART_LOW_BLOCKNUM_3_END 		0x00F9FFFF

//mid
#define FLASH_BLOCKPART_MID_BLOCKNUM_0_START 	0x00808000
#define FLASH_BLOCKPART_MID_BLOCKNUM_0_END 		0x0080FFFF
#define FLASH_BLOCKPART_MID_BLOCKNUM_1_START 	0x00810000
#define FLASH_BLOCKPART_MID_BLOCKNUM_1_END 		0x00817FFF

//high
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_0_START	0x00FA0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_0_END 	0x00FAFFFF
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_1_START 	0x00FB0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_1_END 	0x00FBFFFF
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_2_START 	0x00FC0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_2_END 	0x00FCFFFF
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_3_START 	0x00FD0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_3_END 	0x00FDFFFF
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_4_START 	0x00FE0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_4_END 	0x00FEFFFF
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_5_START 	0x00FF0000
#define FLASH_BLOCKPART_HIGH_BLOCKNUM_5_END 	0x00FFFFFF

//256k
#define FLASH_BLOCKPART_256K_BLOCKNUM_0_START 	0x01000000
#define FLASH_BLOCKPART_256K_BLOCKNUM_0_END 	0x0103FFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_1_START 	0x01040000
#define FLASH_BLOCKPART_256K_BLOCKNUM_1_END 	0x0107FFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_2_START 	0x01080000
#define FLASH_BLOCKPART_256K_BLOCKNUM_2_END 	0x010BFFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_3_START 	0x010C0000
#define FLASH_BLOCKPART_256K_BLOCKNUM_3_END 	0x010FFFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_4_START 	0x01100000
#define FLASH_BLOCKPART_256K_BLOCKNUM_4_END 	0x0113FFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_5_START 	0x01140000
#define FLASH_BLOCKPART_256K_BLOCKNUM_5_END 	0x0117FFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_6_START 	0x01180000
#define FLASH_BLOCKPART_256K_BLOCKNUM_6_END 	0x011BFFFF
#define FLASH_BLOCKPART_256K_BLOCKNUM_7_START 	0x011C0000
#define FLASH_BLOCKPART_256K_BLOCKNUM_7_END 	0x011FFFFF


/* Lock State , shift is in the setlock function*/
#define LOCK_LOW_BLOCKS               0x00003FFFU  //lock0
#define LOCK_MID_BLOCKS               0x0000FFFFU  //lock0
#define LOCK_HIGH_BLOCKS              0x0000FFFFU  //lock1
#define LOCK_FIRST256_BLOCKS          0xFFFFFFFFU  //lock2
//#define UNLOCK_SECOND256_BLOCKS         0x0000FFFFU  //lock3

#define UNLOCK_LOW_BLOCKS               0x00000000U
#define UNLOCK_MID_BLOCKS               0x00000000U
#define UNLOCK_HIGH_BLOCKS              0x00000000U
#define UNLOCK_FIRST256_BLOCKS          0x00000000U
//#define UNLOCK_SECOND256_BLOCKS         0x00000000U


#define NUMBER_OF_WORD_BLANK_CHECK      0x90
#define NUMBER_OF_WORD_PGM_VERIFY       0x80
#define NUMBER_OF_WORD_CHECK_SUM        0x120

/* Platform Flash */
#define FLASH_FMC                       PFLASH_BASE
#define FLASH_PFCR1                     0x000000000U
#define FLASH_PFCR2                     0x000000004U
#define FLASH_FMC_BFEN_MASK             0x000000001U

//Block part and number structure
typedef struct
{
	uint32_t BlockPart; //Low,Mid,High or 256K
	uint32_t BlockNum; //Block 0...n
}Flash_BlockPartAndNum;


static uint32_t Flash_Block_Lock[4]={
	LOCK_LOW_BLOCKS,
	LOCK_MID_BLOCKS,
	LOCK_HIGH_BLOCKS,
	LOCK_FIRST256_BLOCKS,
};

static uint32_t Flash_Block_UnLock[4]={
	UNLOCK_LOW_BLOCKS,
	UNLOCK_MID_BLOCKS,
	UNLOCK_HIGH_BLOCKS,
	UNLOCK_FIRST256_BLOCKS,
};

//every part block count
static uint32_t Flash_Block_Count[4]={
	FLASH_LOWPART_BLOCKCOUNT,
	FLASH_MIDPART_BLOCKCOUNT,
	FLASH_HIGHPART_BLOCKCOUNT,
	FLASH_256KPART_BLOCKCOUNT,
};


//low space block start array
static uint32_t Flash_Block_Low_Start_Array[FLASH_LOWPART_BLOCKCOUNT]={
	FLASH_BLOCKPART_LOW_BLOCKNUM_0_START,
	FLASH_BLOCKPART_LOW_BLOCKNUM_1_START,
    FLASH_BLOCKPART_LOW_BLOCKNUM_2_START,
    FLASH_BLOCKPART_LOW_BLOCKNUM_3_START,
};

//mid space block start array
static uint32_t Flash_Block_Mid_Start_Array[FLASH_MIDPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_MID_BLOCKNUM_0_START,
    FLASH_BLOCKPART_MID_BLOCKNUM_1_START,
};

//high space block start array
static uint32_t Flash_Block_High_Start_Array[FLASH_HIGHPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_HIGH_BLOCKNUM_0_START,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_1_START,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_2_START,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_3_START,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_4_START,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_5_START,

};

//256k space block start array
static uint32_t Flash_Block_256K_Start_Array[FLASH_256KPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_256K_BLOCKNUM_0_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_1_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_2_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_3_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_4_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_5_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_6_START,
    FLASH_BLOCKPART_256K_BLOCKNUM_7_START,

};


//low space block end array
static uint32_t Flash_Block_Low_End_Array[FLASH_LOWPART_BLOCKCOUNT]={
	FLASH_BLOCKPART_LOW_BLOCKNUM_0_END,
	FLASH_BLOCKPART_LOW_BLOCKNUM_1_END,
    FLASH_BLOCKPART_LOW_BLOCKNUM_2_END,
    FLASH_BLOCKPART_LOW_BLOCKNUM_3_END,
};

//mid space block end array
static uint32_t Flash_Block_Mid_End_Array[FLASH_MIDPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_MID_BLOCKNUM_0_END,
    FLASH_BLOCKPART_MID_BLOCKNUM_1_END,
};

//high space block end array
static uint32_t Flash_Block_High_End_Array[FLASH_HIGHPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_HIGH_BLOCKNUM_0_END,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_1_END,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_2_END,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_3_END,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_4_END,
    FLASH_BLOCKPART_HIGH_BLOCKNUM_5_END,

};

//256k space block end array
static uint32_t Flash_Block_256K_End_Array[FLASH_256KPART_BLOCKCOUNT]={
    FLASH_BLOCKPART_256K_BLOCKNUM_0_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_1_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_2_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_3_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_4_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_5_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_6_END,
    FLASH_BLOCKPART_256K_BLOCKNUM_7_END,

};

static uint32_t pflash_pfcr1, pflash_pfcr2;
static void (*user_callback)(void) = NULL;

/*Disable Flash Cache*/
static void DisableFlashControllerCache(uint32_t flashConfigReg, uint32_t disableVal, uint32_t *origin_pflash_pfcr)
{
    *origin_pflash_pfcr = REG_READ32(FLASH_FMC + flashConfigReg);

    REG_BIT_CLEAR32(FLASH_FMC + flashConfigReg, disableVal);
}

/*Restore configuration register of FCM*/
static void RestoreFlashControllerCache(uint32_t flashConfigReg, uint32_t pflash_pfcr)
{
    REG_WRITE32(FLASH_FMC + flashConfigReg, pflash_pfcr);
}


/**
  * @brief  Gets the sector of a given address
  * @param
  * @retval The sector of a given address
  */
static flash_block_select_t Flash_GetSelectBlock(uint32_t startAddr, uint32_t endAddr)
{
	flash_block_select_t blockSelect={.lowBlockSelect=0,.midBlockSelect=0,.highBlockSelect=0,.first256KBlockSelect = 0,.second256KBlockSelect=0};
	uint8_t i;

	for(i=0;i<FLASH_LOWPART_BLOCKCOUNT;i++)
	{
		if( !(startAddr>Flash_Block_Low_End_Array[i] || endAddr<Flash_Block_Low_Start_Array[i] ) )
		{
			blockSelect.lowBlockSelect |= 1<<i;
		}
	}

	for(i=0;i<FLASH_MIDPART_BLOCKCOUNT;i++)
	{
		if( !(startAddr>Flash_Block_Mid_End_Array[i] || endAddr<Flash_Block_Mid_Start_Array[i] ) )
		{
			blockSelect.midBlockSelect |= 1<<i;
		}
	}

	for(i=0;i<FLASH_HIGHPART_BLOCKCOUNT;i++)
	{
		if( !(startAddr>Flash_Block_High_End_Array[i] || endAddr<Flash_Block_High_Start_Array[i] ) )
		{
			blockSelect.highBlockSelect |= 1<<i;
		}
	}

	for(i=0;i<FLASH_256KPART_BLOCKCOUNT;i++)
	{
		if( !(startAddr>Flash_Block_256K_End_Array[i] || endAddr<Flash_Block_256K_Start_Array[i] ) )
		{
			blockSelect.first256KBlockSelect |= 1<<i;
		}
	}

	return blockSelect;
}


static status_t Flash_Unlock(uint32_t startAddr, uint32_t endAddr)
{
	status_t ret = STATUS_SUCCESS;
	uint32_t blkLockState;         /* block lock status to be retrieved */
	flash_block_select_t blockSelect={.lowBlockSelect=0,.midBlockSelect=0,.highBlockSelect=0,.first256KBlockSelect = 0,.second256KBlockSelect=0};

	/* Invalidate flash controller cache */
	DisableFlashControllerCache(FLASH_PFCR1, FLASH_FMC_BFEN_MASK, &pflash_pfcr1);
	DisableFlashControllerCache(FLASH_PFCR2, FLASH_FMC_BFEN_MASK, &pflash_pfcr2);

	/* Flash Initialization */
	ret = FLASH_DRV_Init();
	DEV_ASSERT(ret == STATUS_SUCCESS);

	/* Lock to protect UTest address space */
	ret = FLASH_DRV_GetLock(C55_BLOCK_UTEST, &blkLockState);

	if (!(blkLockState & 0x00000001U))
	{
		ret = FLASH_DRV_SetLock(C55_BLOCK_UTEST, 0x1U);
		if (STATUS_SUCCESS != ret)
		{
			return ret;
		}
	}

	blockSelect = Flash_GetSelectBlock(startAddr, endAddr);

	//UnLock all block in range
	if(blockSelect.lowBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(0, Flash_Block_UnLock[0]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.midBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(1, Flash_Block_UnLock[1]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.highBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(2, Flash_Block_UnLock[2]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.first256KBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(3, Flash_Block_UnLock[3]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	return ret;
}

status_t Flash_Lock(uint32_t startAddr, uint32_t endAddr)
{
	status_t ret = STATUS_SUCCESS;

	flash_block_select_t blockSelect={.lowBlockSelect=0,.midBlockSelect=0,.highBlockSelect=0,.first256KBlockSelect = 0,.second256KBlockSelect=0};

	//UnLock block
	blockSelect = Flash_GetSelectBlock(startAddr, endAddr);

	//UnLock all block in range
	if(blockSelect.lowBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(0, Flash_Block_Lock[0]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.midBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(1, Flash_Block_Lock[1]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.highBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(2, Flash_Block_Lock[2]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	if(blockSelect.first256KBlockSelect!=0)
	{
		ret = FLASH_DRV_SetLock(3, Flash_Block_Lock[3]);
		if (STATUS_SUCCESS != ret)
			return ret;
	}

	/* Restore flash controller cache */
	RestoreFlashControllerCache(FLASH_PFCR1, pflash_pfcr1);
	RestoreFlashControllerCache(FLASH_PFCR2, pflash_pfcr2);

	return ret;
}

void register_fal_user_callback(void (*cb)(void))
{
    user_callback = cb;
}

int mpc5744_flash_init(void)
{
	static int mpc_flash_init_ok = 0;
	if(mpc_flash_init_ok == 0)
	{
		mpc_flash_init_ok = 1;
		DisableFlashControllerCache(FLASH_PFCR1, FLASH_FMC_BFEN_MASK, &pflash_pfcr1);
		DisableFlashControllerCache(FLASH_PFCR2, FLASH_FMC_BFEN_MASK, &pflash_pfcr2);

		FLASH_DRV_Init();

		RestoreFlashControllerCache(FLASH_PFCR1, pflash_pfcr1);
		RestoreFlashControllerCache(FLASH_PFCR2, pflash_pfcr2);
	}
	return 1;
}

/**
 * Read data from flash.
 * @note This operation's units is word.
 *
 * @param addr flash address
 * @param buf buffer to store read data
 * @param size read bytes size
 *
 * @return result
 */
static int mpc5744_flash_read(uint32_t addr, uint8_t *buf, size_t size)
{
    size_t i;

    if ((addr + size) > MPC5744_FLASH_END_ADDRESS)
    {
        log_e("read outrange flash size! addr is (0x%p)", (void *)(addr + size));
        return -1;
    }

    for (i = 0; i < size; i++, buf++, addr++)
    {
        *buf = *(uint8_t *) addr;
    }

    return size;
}

/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
static int mpc5744_flash_write(uint32_t addr, const uint8_t *buf, size_t size)
{
	size_t i, j;
	int result     	  = 0;
	uint64_t write_data = 0, temp_data = 0;
	flash_context_data_t pCtxData;
	flash_state_t opResult = C55_OK;
	uint32_t start_addr = addr;
	uint32_t end_addr = addr + size - 1;

	if ((addr + size) > MPC5744_FLASH_END_ADDRESS)
	{
		log_e("write outrange flash size! addr is (0x%p)", (void *)(addr + size));
		return -1;
	}

	if(addr % C55_DWORD_SIZE != 0)
	{
		log_e("write addr must be 8-byte alignment");
		return -1;
	}

	if (size < 1)
	{
		log_e("size < 1");
		return -1;
	}

	Flash_Unlock(start_addr,end_addr);
	for (i = 0; i < size;)
	{
		if ((size - i) < C55_DWORD_SIZE)
		{
			for (j = 0; (size - i) > 0; i++, j++)
			{
				//注意大小端
				temp_data = *buf;
				write_data |= (temp_data << (64 - (8 * (j + 1)) ));//8: 1byte = 8bit
				buf ++;
			}
			uint8_t FF_buf[8] = {0};
			for(size_t m = 0;m < (C55_DWORD_SIZE - j);m++ )
			{
				FF_buf[7-m] = 0XFF;
			}
			write_data |= (*(uint64_t *)FF_buf);
		}
		else
		{
			for (j = 0; j < C55_DWORD_SIZE; j++, i++)
			{
				//注意大小端
				temp_data = *buf;
				write_data |= (temp_data << (64 - (8 * (j + 1)) ));//8: 1byte = 8bit
				buf ++;
			}
		}

		/* write data */
		if (FLASH_DRV_Program(&pCtxData, addr, C55_DWORD_SIZE, &write_data) == STATUS_SUCCESS)
		{
			status_t returnCode = STATUS_FLASH_INPROGRESS;

			/* Check the return code status */
			while (returnCode == STATUS_FLASH_INPROGRESS)
			{
			    if(user_callback != NULL)
			        user_callback();
				/* Check status */
				returnCode = FLASH_DRV_CheckProgramStatus(&pCtxData,
														  &opResult);
			}

			if ((returnCode != STATUS_SUCCESS) && (opResult != C55_OK))
			{
				log_e("ERROR: write data error\n");
				result = -1;
				goto __exit;
			}
			/* Check the written value */
			if (*(uint64_t*)addr != write_data)
			{
				log_e("ERROR: write data != read data\n");
				result = -1;
				goto __exit;
			}
		}
		else
		{
			log_e("ERROR: flash program error");
			result = -1;
			goto __exit;
		}

		temp_data = 0;
		write_data = 0;

		addr += 8;
	}

__exit:
	Flash_Lock(start_addr,end_addr);
	if (result != 0)
	{
		return result;
	}

	return size;
}

/**
 * Erase data on flash.
 * @note This operation is irreversible.
 * @note This operation's units is different which on many chips.
 *
 * @param addr flash address
 * @param size erase bytes size
 *
 * @return result
 */
static int mpc5744_flash_erase(uint32_t addr, size_t size)
{
	uint32_t start_addr = addr;
	uint32_t end_addr = addr + size -1;
	status_t ret = STATUS_SUCCESS;
	flash_state_t opResult;
	flash_block_select_t blockSelect;

	if ((addr + size) > MPC5744_FLASH_END_ADDRESS)
	{
		log_e("write outrange flash size! addr is (0x%p)", (void *)(addr + size));
		return -1;
	}

	Flash_Unlock(start_addr,end_addr);
	blockSelect = Flash_GetSelectBlock(start_addr,end_addr);

	/* Erase block */
	ret = FLASH_DRV_Erase(ERS_OPT_MAIN_SPACE, &blockSelect);

	if (STATUS_SUCCESS == ret)
	{
		do
		{
		    if(user_callback != NULL)
                user_callback();
			ret = FLASH_DRV_CheckEraseStatus(&opResult);
		}while(ret == STATUS_FLASH_INPROGRESS);
	}

	Flash_Lock(start_addr,end_addr);
	if (STATUS_SUCCESS != ret)
	{
		return -1;
	}

	return size;
}

#define FLASH_SIZE_16K_P1 	(2 * 16 * 1024)
#define FLASH_SIZE_32K 		(2 * 32 * 1024)
#define FLASH_SIZE_16K_P2 	(2 * 16 * 1024)
#define FLASH_SIZE_64K 		(6 * 64 * 1024)
#define FLASH_SIZE_256K 	(8 * 256 * 1024)

#define FLASH_START_ADDR_16K_P1 (0x00800000)
#define FLASH_START_ADDR_32K 	(0x00808000)
#define FLASH_START_ADDR_16K_P2 (0x00F98000)
#define FLASH_START_ADDR_64K 	(0x00FA0000)
#define FLASH_START_ADDR_256K 	(0x01000000)

static int fal_flash_read_16k_p1(long offset, uint8_t *buf, size_t size);
static int fal_flash_read_32k(long offset, uint8_t *buf, size_t size);
static int fal_flash_read_16k_p2(long offset, uint8_t *buf, size_t size);
static int fal_flash_read_64k(long offset, uint8_t *buf, size_t size);
static int fal_flash_read_256k(long offset, uint8_t *buf, size_t size);

static int fal_flash_write_16k_p1(long offset, const uint8_t *buf, size_t size);
static int fal_flash_write_32k(long offset, const uint8_t *buf, size_t size);
static int fal_flash_write_16k_p2(long offset, const uint8_t *buf, size_t size);
static int fal_flash_write_64k(long offset, const uint8_t *buf, size_t size);
static int fal_flash_write_256k(long offset, const uint8_t *buf, size_t size);

static int fal_flash_erase_16k_p1(long offset, size_t size);
static int fal_flash_erase_32k(long offset, size_t size);
static int fal_flash_erase_16k_p2(long offset, size_t size);
static int fal_flash_erase_64k(long offset, size_t size);
static int fal_flash_erase_256k(long offset, size_t size);

const struct fal_flash_dev mpc5744_onchip_flash_16k_p1 =
{
    .name       = "onchip_16k_p1",
    .addr       = FLASH_START_ADDR_16K_P1,
    .len        = FLASH_SIZE_16K_P1,
    .blk_size   = (16 * 1024),
    .ops        = {mpc5744_flash_init, fal_flash_read_16k_p1, fal_flash_write_16k_p1, fal_flash_erase_16k_p1},
    .write_gran = 64
};
const struct fal_flash_dev mpc5744_onchip_flash_32k =
{
    .name       = "onchip_32k",
    .addr       = FLASH_START_ADDR_32K,
    .len        = FLASH_SIZE_32K,
    .blk_size   = (32 * 1024),
    .ops        = {mpc5744_flash_init, fal_flash_read_32k, fal_flash_write_32k, fal_flash_erase_32k},
    .write_gran = 64
};
const struct fal_flash_dev mpc5744_onchip_flash_16k_p2 =
{
    .name       = "onchip_16k_p2",
    .addr       = FLASH_START_ADDR_16K_P2,
    .len        = FLASH_SIZE_16K_P2,
    .blk_size   = (16 * 1024),
    .ops        = {mpc5744_flash_init, fal_flash_read_16k_p2, fal_flash_write_16k_p2, fal_flash_erase_16k_p2},
    .write_gran = 64
};
const struct fal_flash_dev mpc5744_onchip_flash_64k =
{
    .name       = "onchip_64k",
    .addr       = FLASH_START_ADDR_64K,
    .len        = FLASH_SIZE_64K,
    .blk_size   = (64 * 1024),
    .ops        = {mpc5744_flash_init, fal_flash_read_64k, fal_flash_write_64k, fal_flash_erase_64k},
    .write_gran = 64
};
const struct fal_flash_dev mpc5744_onchip_flash_256k =
{
    .name       = "onchip_256k",
    .addr       = FLASH_START_ADDR_256K,
    .len        = FLASH_SIZE_256K,
    .blk_size   = (256 * 1024),
    .ops        = {mpc5744_flash_init, fal_flash_read_256k, fal_flash_write_256k, fal_flash_erase_256k},
    .write_gran = 64
};


static int fal_flash_read_16k_p1(long offset, uint8_t *buf, size_t size)
{
	return mpc5744_flash_read(mpc5744_onchip_flash_16k_p1.addr + offset, buf, size);
}
static int fal_flash_read_32k(long offset, uint8_t *buf, size_t size)
{
	return mpc5744_flash_read(mpc5744_onchip_flash_32k.addr + offset, buf, size);
}
static int fal_flash_read_16k_p2(long offset, uint8_t *buf, size_t size)
{
	return mpc5744_flash_read(mpc5744_onchip_flash_16k_p2.addr + offset, buf, size);
}
static int fal_flash_read_64k(long offset, uint8_t *buf, size_t size)
{
	return mpc5744_flash_read(mpc5744_onchip_flash_64k.addr + offset, buf, size);
}
static int fal_flash_read_256k(long offset, uint8_t *buf, size_t size)
{
	return mpc5744_flash_read(mpc5744_onchip_flash_256k.addr + offset, buf, size);
}

static int fal_flash_write_16k_p1(long offset, const uint8_t *buf, size_t size)
{
	return mpc5744_flash_write(mpc5744_onchip_flash_16k_p1.addr + offset, buf, size);
}
static int fal_flash_write_32k(long offset, const uint8_t *buf, size_t size)
{
	return mpc5744_flash_write(mpc5744_onchip_flash_32k.addr + offset, buf, size);
}
static int fal_flash_write_16k_p2(long offset, const uint8_t *buf, size_t size)
{
	return mpc5744_flash_write(mpc5744_onchip_flash_16k_p2.addr + offset, buf, size);
}
static int fal_flash_write_64k(long offset, const uint8_t *buf, size_t size)
{
	return mpc5744_flash_write(mpc5744_onchip_flash_64k.addr + offset, buf, size);
}
static int fal_flash_write_256k(long offset, const uint8_t *buf, size_t size)
{
	return mpc5744_flash_write(mpc5744_onchip_flash_256k.addr + offset, buf, size);
}

static int fal_flash_erase_16k_p1(long offset, size_t size)
{
	return mpc5744_flash_erase(mpc5744_onchip_flash_16k_p1.addr + offset, size);
}
static int fal_flash_erase_32k(long offset, size_t size)
{
	return mpc5744_flash_erase(mpc5744_onchip_flash_32k.addr + offset, size);
}
static int fal_flash_erase_16k_p2(long offset, size_t size)
{
	return mpc5744_flash_erase(mpc5744_onchip_flash_16k_p2.addr + offset, size);
}
static int fal_flash_erase_64k(long offset, size_t size)
{
	return mpc5744_flash_erase(mpc5744_onchip_flash_64k.addr + offset, size);
}
static int fal_flash_erase_256k(long offset, size_t size)
{
	return mpc5744_flash_erase(mpc5744_onchip_flash_256k.addr + offset, size);
}
