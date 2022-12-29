#include "storage.h"
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal.h"

typedef struct {
	uint32_t baseAddress;
	char data[kStorageSlotSize];
	uint8_t synced;
} Storage;

static Storage storage = {0};

static const uint32_t kFlashBase = 0x08000000;

static void* getSectorStart(uint32_t sector)
{
	return (void*)(kFlashBase + sector * kStorageSectorSize);
}

static size_t countNonErasedBytes(uint8_t* p, size_t size)
{
	size_t count = 0;
	for(size_t n = 0; n < size; ++n)
	{
		if(0xff != p[n])
			++count;
	}
	return count;
}

void storageInit(uint32_t sector, uint32_t slot)
{
	storage.baseAddress = (uint32_t)getSectorStart(sector) + slot * kStorageSlotSize;
	storage.synced = 0;
}

void storageRead()
{
	memcpy(&storage.data, ((const char*)storage.baseAddress), kStorageSlotSize);
	storage.synced = 1;
}

static void printError(const char* id, uint32_t error, uint32_t loc)
{
	printf("%s storage error %ld at location %lx\r\n", id, error, loc);
}

int storageErase(uint32_t sector)
{
	FLASH_EraseInitTypeDef eraseInit;
	uint32_t errorLoc;
	eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	eraseInit.Banks = FLASH_BANK_1;
	eraseInit.Sector = sector;
	eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef ret = HAL_FLASHEx_Erase(&eraseInit, &errorLoc);
	HAL_FLASH_Lock();
	if(HAL_OK != ret) {
		printError("erase", HAL_FLASH_GetError(), errorLoc);
		return -1;
	}
	size_t count = countNonErasedBytes(getSectorStart(sector), kStorageSectorSize);
	if(count) {
		printf("Error erasing flash sector %lu: %zu bytes still non-zero\n\r", sector, count);
		return 1;
	}
	printf("Successfully erased flash sector %lu\n\r", sector);
	return 0;
}

int storageWrite()
{
	const uint8_t* ptr = (uint8_t*)storage.baseAddress;
	size_t count = countNonErasedBytes((void*)storage.baseAddress, kStorageSlotSize);
	if(count)
	{
		printf("Cannot write to flash because the destination has %zu non-erased bytes in the slot stargin at %p\n\r", count, ptr);
	}
	HAL_FLASH_Unlock();
	uint32_t address = storage.baseAddress;
	const size_t kFlashWordSize = FLASH_NB_32BITWORD_IN_FLASHWORD * sizeof(uint32_t);
	for(unsigned int idx = 0; idx < kStorageSlotSize; idx += kFlashWordSize)
	{
		// writes one flash word at a time
		HAL_StatusTypeDef ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)(storage.data + idx));
		if(HAL_OK != ret) {
			printError("write", HAL_FLASH_GetError(), address);
			return -2;
		}
		address += kFlashWordSize ;
	}
	HAL_FLASH_Lock();
	storage.synced = 1;
	printf("Successfully written flash starting at %#lx (%#lx bytes written)\n\r", storage.baseAddress, address - storage.baseAddress);

	return 0;
}

static StorageWord_t* storageGetPtr(uint32_t index)
{
	return ((StorageWord_t*)storage.data) + index;
}

uint8_t storageIsSynced()
{
	return storage.synced;
}

StorageWord_t storageGet(uint32_t index)
{
	return storageGetPtr(index)[0];
}

void storageSet(uint32_t index, StorageWord_t value)
{
	if(index < storageGetLength()) {
		StorageWord_t* ptr = storageGetPtr(index);
		if(ptr[0] != value) {
			ptr[0] = value;
			storageWasSet();
		}
	}
}

StorageWord_t* storageGetData()
{
	return (StorageWord_t*)storage.data;
}

uint32_t storageGetLength()
{
	return kStorageSlotSize / sizeof(StorageWord_t);
}

void storageWasSet()
{
	storage.synced = 0;
}
