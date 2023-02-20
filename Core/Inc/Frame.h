#ifndef FRAME_H_
#define FRAME_H_

#include "main.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;


#define MY_UART huart6

#define COMMAND_FRAME_LENGHT 10
#define HEADER_FRAME_LENGHT 13
#define DATA_LENGHT 1024
#define DATA_FRAME_LENGHT 9 + DATA_LENGHT


#define FLASH_SLOT_0_ADD	0x08020000
#define FLASH_SLOT_1_ADD	0x08060000
#define FLASH_SLOT_2_ADD	0x080A0000

/* Number of frame data, for example: Length of file is 9.5Kbyte then Number = 10 */
static uint8_t Frame_NumberOfKbyteData = 0;
static uint16_t Frame_DataLenghtPerFrame = 0;

typedef enum{
	FLASH_SLOT_0 = 0x00,
	FLASH_SLOT_1,
	FLASH_SLOT_2
}defFlashSlot;

typedef enum{
    SOF_SIGNAL = 0x00,
    EOF_SIGNAL
}defTerminateFrame;

typedef enum{
    START_COMMAND = 0x00,
    STOP_COMMAND,
    HEADER_COMMAND,
}defCommandType;

typedef enum{
    COMMAND_TYPE = 0x00,
    HEADER_TYPE,
    DATA_TYPE,
    RESPONSE_TYPE
}defPacketType;


typedef enum{
    RESPONSE_ACK = 0x00,
    RESPONSE_NACK,
    RESPONSE_ABORT
}defResponseType;

typedef enum{
    HANDLE_OK = 0x00,
    HANDLE_FAILED
}defReturnType;

/* Convert from 4 bytes to uin32 type */
typedef union{
	uint8_t DataArray[4];
    uint32_t DataUint32;
}uint32To4Bytes;

typedef struct{
	uint8_t DataSize[4];
    uint8_t Reserved1[4];
    uint8_t Reserved2[4];
    uint8_t Reserved3[4];
}defMetaInfo;

typedef struct{
	uint8_t Sof;
	uint8_t Type;
	uint8_t DataLength[2];
    uint8_t Command;
    uint32To4Bytes Crc;
    uint8_t Eof;
}defCommandPacket;

typedef struct{
	uint8_t Sof;
	uint8_t Type;
	uint8_t DataLength[2];
	uint8_t Datainfo[4];
    uint32To4Bytes Crc;
    uint8_t Eof;
}defHeaderPacket;

typedef struct{
	uint8_t Sof;
	uint8_t Type;
	uint8_t DataLenght[2];
	uint8_t* Data;
    uint32To4Bytes Crc;
    uint8_t Eof;
}defDataPacket;

typedef struct{
	uint8_t Sof;
	uint8_t Type;
	uint8_t DataLength[2];
    uint8_t Command;
    uint32To4Bytes Crc;
    uint8_t Eof;
}defResponsePacket;

typedef enum{
	RESET_NORMAL = 0x00,
	RESET_OTA
}defResetCause;

typedef struct{
	uint8_t FlashOnGoing;
	uint8_t ResetCause;
}defFlashCfg;

static defFlashCfg	Flash_Cfg = {
	.FlashOnGoing = FLASH_SLOT_0,
	.ResetCause = RESET_NORMAL,
};

void Frame_SendResponseFrame(defResponseType response);
defReturnType Frame_ReadStartFrame();
defReturnType Frame_ReadStopFrame();
defReturnType Frame_ReadHeaderFrame();
defReturnType Frame_ReadDataFrame(uint8_t* data);
defResponseType Frame_CheckSum(uint8_t* res_frame);
defReturnType Frame_WriteToFlash(uint8_t slot, uint8_t* data);
defReturnType Frame_STM32OTA();
defReturnType Frame_InitFlash(uint8_t slot); //call only one time.
defReturnType Frame_WriteFlashToSlot(uint8_t slot, uint8_t* data, uint16_t lenght);

defReturnType Frame_WriteCfg(){//write to sector 4 in STM32F407
	defReturnType ret;

	ret = HAL_FLASH_Unlock();
	if(ret != (uint8_t)HAL_OK){
		return HANDLE_FAILED;
	}

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;

	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;

	EraseInitStruct.Sector        = FLASH_SECTOR_11;

	EraseInitStruct.NbSectors     = 1;           //erase 1 sectors
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

	ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
	if( ret != (uint8_t)HAL_OK )
	{
		return HANDLE_FAILED;
	}

	uint32_t baseaddr = 0x080E0000; //Sector 4 in STM32F407

	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, baseaddr, Flash_Cfg.FlashOnGoing) != HAL_OK)
			return HANDLE_FAILED;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, baseaddr + 1, Flash_Cfg.ResetCause) != HAL_OK)
			return HANDLE_FAILED;

	HAL_FLASH_Lock();
	return HANDLE_OK;
}


defReturnType Frame_ReadCfg(defFlashCfg* Cfg){//write to sector 4 in STM32F407
	defReturnType ret;

	ret = HAL_FLASH_Unlock();
	if(ret != (uint8_t)HAL_OK){
		return HANDLE_FAILED;
	}


	uint32_t baseaddr = 0x080E0000; //Sector 4 in STM32F407

	uint32_t Rx_Buf = *(__IO uint32_t *)baseaddr;
	Cfg->FlashOnGoing = (uint8_t)((Rx_Buf)&0xFF);
	Cfg->ResetCause = (uint8_t)(((Rx_Buf)>>8)&0xFF);

	HAL_FLASH_Lock();
	return HANDLE_OK;
}


#endif 
