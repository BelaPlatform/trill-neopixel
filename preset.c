#include "preset.h"
#include "storage.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define getSectorFromSlot(slot) \
		((kPresetStarts - kFlashBase + kStorageSlotSize * slot) / kStorageSectorSize)


// we have an STM32G4KEU6: 512k of flash, divided in 256 2k pages.
// For presets, we use the space that starts 384k into flash at 0x08060000
static const uint32_t kPresetStarts = 0x08060000;
static const uint32_t kPresetSectorStarts = getSectorFromSlot(0);
static const uint32_t kPresetSectorStops = getSectorFromSlot(kStorageNumSlots);
// Signature used to recognise a valid preset slot.
// this gets shifted left by however many places needed to fit the actual
// preset length into a uint32_t. If you want to force a change and invalidate
// all currently stored presets, edit the rightmost part of it.
static const uint32_t kPresetHeader = 0x23456C;
static uint32_t gPresetSignature; // computed at runtime based on kPresetHeader and actual size

static struct {
	uint32_t slot;
	uint32_t checkSaveTimeout;
	uint32_t lastChangedTime;
	uint32_t (*getTime)();
} p;

static PresetDesc_t presetDescs[kNumPresets];

void presetDescSet(size_t idx, PresetDesc_t* desc)
{
	if(idx < kNumPresets)
		presetDescs[idx] = *desc;
	else {
		printf("presetDescSet %d out of %d\n\r", idx, kNumPresets);
		assert(0);
	}
}

static PresetDesc_t* getPresetDesc(const PresetField_t field)
{
	for(unsigned int n = 0; n < kNumPresets; ++n)
		if(field == presetDescs[n].field)
			return &presetDescs[n];
	return NULL;
}

static void presetLoadCallbackAll()
{
	for(unsigned int n = 0; n < kNumPresets; ++n)
	{
		PresetDesc_t* desc = &(presetDescs[n]);
		const void* data = presetGetField(desc->field);
		if(desc->loadCallback)
			desc->loadCallback(desc->field, desc->size, data);
	}
}

static uint32_t getSlotAdd(int32_t slot, int32_t add)
{
	return  (slot + kStorageNumSlots + add) % kStorageNumSlots;
}

static uint32_t getPrevSlot(uint32_t slot)
{
	return getSlotAdd(slot, -1);
}

static uint32_t getNextSlot(uint32_t slot)
{
	return getSlotAdd(slot, 1);
}

static uint32_t getPresetNumber() {
	return p.slot;
}

int presetInit(PresetInitOptions_t option, uint32_t checkSaveTimeout, uint32_t (*getTime)())
{
	unsigned int offset = sizeof(kPresetHeader);
	for(unsigned int n = 0; n < kNumPresets; ++n) {
		presetDescs[n].offset = offset;
		offset += presetDescs[n].size;
	}
	if(offset > kStorageSlotSize)
	{
		printf("Error: the preset structure is larger than kStorageSlotSize\n\r");
		return -1;
	} else {
		printf("preset structure size: %u\n\r", offset);
	}
	uint32_t bit = 0;
	for(unsigned int n = 0; n < sizeof(bit) * 8; ++n)
	{
		if((1 << n) >= kStorageSlotSize)
		{
			bit = n;
			break;
		}
	}
	// a unique signature
	gPresetSignature = offset | (kPresetHeader << bit);
	printf("gPresetSignature: %lx\n\r", gPresetSignature);
	enum { kNotFound = -1 };
	int32_t found = kNotFound;
	if(kPresetInit_LoadDefault != option) {
		// go through the available slots in flash and keep the last good one
		// we find
		for(uint32_t s = 0; s < kStorageNumSlots; ++s) {
			storageInit(kPresetSectorStarts, s);
			// TODO: add checksum to ensure a slot is actually valid.
			// (currently we are just checking that it contains a valid
			// signature at the beginning of the slot)
			storageRead();
			uint32_t* ptr = (uint32_t*)storageGetData();
			if(gPresetSignature == ptr[0]) {
				// if we find a valid slot, remember it
				found = s;
			} else {
				// first invalid slot we break
				break;
			}
		}
	}
	uint32_t slot;
	if(kNotFound == found)
	{
	  // if we didn't find a valid one (or we are supposed to load defaults)
		printf("no preset found\n\r");
		// select the last slot so upon writing we will write the first one
		slot = getPrevSlot(0);
		storageInit(kPresetSectorStarts, slot);
		uint32_t* ptr = (uint32_t*)storageGetData();
		presetLoadDefaults();
		ptr[0] = gPresetSignature;
	} else {
		slot = found;
		storageInit(kPresetSectorStarts, slot);
		storageRead();
		printf("preset found at %lu\n\r", slot);
	}
	presetLoadCallbackAll();
	p.getTime = getTime;
	if(p.getTime())
		p.lastChangedTime = p.getTime();
	p.slot = slot;
	p.checkSaveTimeout = checkSaveTimeout;
	return getPresetNumber();
}

static void defaultDefaulter(PresetField_t field, PresetFieldSize_t size, void* data)
{
	memset(data, 0, size);
}

static void presetSetFieldDone()
{
	if(p.getTime)
		p.lastChangedTime = p.getTime();
	storageWasSet();
}
// to be used for internal purposes only: presetSetFieldDone() needs to be called
// once the caller is done with modifying the pointer.
static void* presetGetFieldRw(PresetField_t field)
{
	return (void*)presetGetField(field);
}

void presetLoadDefaults()
{
	for(unsigned int n = 0; n < kNumPresets; ++n)
	{
		PresetDesc_t* desc = &(presetDescs[n]);
		void* data = presetGetFieldRw(desc->field);
		if(desc->defaulter)
			desc->defaulter(desc->field, desc->size, data);
		else
			defaultDefaulter(desc->field, desc->size, data);
		presetSetFieldDone();
	}
}

int presetLoad()
{
	storageRead();
	presetLoadCallbackAll();
	return 0;
}

#define makeNegative(a) (a > 0 ? -a : a)
int presetSave()
{
	// open the next page in flash
	uint32_t oldSector = getSectorFromSlot(p.slot);
	uint32_t oldSlot = p.slot;
	p.slot = getNextSlot(p.slot);
	uint32_t newSector = getSectorFromSlot(p.slot);
	// if we moved to a new sector or going back to an earlier slot in current sector
	// we need to erase the sector so we can write to it
	if(oldSector != newSector || p.slot < oldSlot)
	{
		storageErase(newSector);
	}
	storageInit(kPresetSectorStarts, p.slot);
	// write to flash
	int ret = storageWrite();
	if(ret)
		return makeNegative(ret);
	if(newSector < oldSector)
	{
		// we wrapped around: erase all other sectors
		for(size_t s = newSector + 1; s < kPresetSectorStops; ++s)
			storageErase(s);
	}
	storageRead();
	return getPresetNumber();
}

int presetCheckSave()
{
	uint8_t should;
	if(presetIsSynced())
		should = 0;
	else if(!p.getTime || 0 == p.lastChangedTime)
		should = 1;
	else {
		uint32_t diff = p.getTime() - p.lastChangedTime;
		if(diff > p.checkSaveTimeout)
			should = 1;
		else
			should = 0;
	}
	if(should) {
		return presetSave();
	} else {
		return -1;
	}
}

PresetFieldSize_t presetGetFieldSize(const PresetField_t field)
{
	PresetDesc_t* desc = getPresetDesc(field);
	if(desc)
		return desc->size;
	else
		return 0;
}

void presetSetField(PresetField_t field, const void* ptr)
{
	PresetDesc_t* desc = getPresetDesc(field);
	if(desc) {
		for(unsigned int n = 0; n < desc->size; ++n)
			storageSet(desc->offset + n, ((StorageWord_t*)ptr)[n]);
		presetSetFieldDone();
	}
}

const void* presetGetField(PresetField_t field)
{
	PresetDesc_t* desc = getPresetDesc(field);
	if(desc)
		return storageGetData() + desc->offset;
	return NULL;
}

uint8_t presetIsSynced()
{
	return storageIsSynced();
}

int presetEraseAll()
{
	int ret = 0;
	for(size_t n = kPresetSectorStarts; n < kPresetSectorStops; ++n)
		ret |= storageErase(n);
	return ret;
}
