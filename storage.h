#pragma once

#include <stdint.h>
#include "stm32h7xx_hal.h" // FLASH_SECTOR_SIZE
enum { kStorageSectorSize = FLASH_SECTOR_SIZE }; // the size of a flash sector, i.e.: the minimum unit that can be erased at once
enum { kStorageSlotSize = 512 }; // the amount of flash that will be managed at a time
enum { kStorageNumSlots = kStorageSectorSize / kStorageSlotSize };

typedef uint8_t StorageWord_t;

/**
 * Storage class.
 *
 * Reads data into memory. Memory can be get and set and committed to storage
 * with dedicated methods. Reading and writing is done in slots of up to
 * kStorageSlotSize, but erasing (which is a pre-requisite for writing) is only
 * done on one kStorageSectorSize sector at a time.
*/
/**
 * Call once to initialise the storage. It does NOT read data from storage into memory.
 */
void storageInit(uint32_t sector, uint32_t slot);
/**
 * Read from storage into memory.
 */
void storageRead();
/**
 * Erases the flash at the specified sector.
 * "static method"
 */
int storageErase(uint32_t sector);
/**
 * Write to storage from memory.
 */
int storageWrite();
/**
 * Has the latest content of the memory buffer been written to storage?
 * This function does not access the storage, and will only return TRUE if data
 * has been manipulated with storageSet() and/or storageWasSet().
 */
uint8_t storageIsSynced();
/**
 * Get a value from the memory buffer.
 */
StorageWord_t storageGet(uint32_t index);
/**
 * Set the value in the memory buffer.
 */
void storageSet(uint32_t index, StorageWord_t value);
/**
 * Get the length of the storage expressed in StorageWord_t elements.
 */
uint32_t storageGetLength();
/**
 * Get access to the memory buffer.
 *
 * After writing to the memory buffer through the use of this pointer, the user
 * should call storageWasSet() in order for storageIsSynced() to return a valid
 * value.
 */
StorageWord_t* storageGetData();
/**
 * Notify the storage object that data in memory has been modified through the pointer returned by storageGetData().
 */
void storageWasSet();
