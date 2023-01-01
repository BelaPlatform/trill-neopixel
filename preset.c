#include "preset.h"
#include "storage.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

// we have an STM32H733VGT6: 1024k of flash, divided in 8 128k sectors.
// we use the last sector
static const uint32_t kPresetSector = 7;
static const uint32_t kPresetHeader = 0x1234568;

static struct {
	uint32_t slot;
	uint32_t checkSaveTimeout;
	uint32_t lastChangedTime;
	uint32_t (*getTime)();
} p;

typedef struct
{
	PresetField_t field;
	PresetFieldSize_t size; // field size expressed in StorageWord_t units
	uint32_t offset;
	PresetDefaulter_t defaulter;
	PresetLoadCallback_t loadCallback;
} PresetDesc_t;

// includes containing PresetDefaulter and PresetLoadCallbacks
#include "keymotion.h"
#include "irReceiver.h"
#include "irEmitter.h"
static PresetDesc_t presetDescs[kNumPresets] = {
#ifdef USE_ANALOG
	{
		.field = kCalibration,
		.size = sizeof(KeyMotionCalibration_t),
		.defaulter = keymotionPresetDefaulter,
		.loadCallback = keymotionPresetLoadCallback,
	},
	{
		.field = kNoteThresholds,
		.size = 8,
		.defaulter = keymotionPresetDefaulter,
		.loadCallback = keymotionPresetLoadCallback
	},
	{
		.field = kVelocityCurve,
		.size = 3,
		.defaulter = keymotionPresetDefaulter,
		.loadCallback = keymotionPresetLoadCallback,
	},
	{
		.field = kTransmissionOptions,
		.size = 4,
		.defaulter = keymotionPresetDefaulter,
		.loadCallback = keymotionPresetLoadCallback,
	},
#endif // USE_ANALOG
#ifdef USE_RGB
	{
		.field = kEnableLeds,
		.size = 1,
		.defaulter = keymotionPresetDefaulter,
		.loadCallback = keymotionPresetLoadCallback,
	},
#endif // USE_RGB
#ifdef USE_ANALOG
	{
		.field = kIrReceiverGain,
		.size = 3,
		.defaulter = irReceiverPresetDefaulter,
		.loadCallback = irReceiverPresetLoadCallback,
	},
	{
		.field = kIrEmitterBrightness,
		.size = 4,
		.defaulter = irEmitterPresetDefaulter,
		.loadCallback = irEmitterPresetLoadCallback,
	}
#endif // USE_ANALOG
};

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
	}
	enum { kNotFound = -1 };
	int32_t found = kNotFound;
	if(kPresetInit_LoadDefault != option) {
		// go through the available slots in flash and keep the last good one
		// we find
		for(uint32_t s = 0; s < kStorageNumSlots; ++s) {
			storageInit(kPresetSector, s);
			// TODO: add checksum to ensure a slot is actually valid.
			// (currently we are just checking that it contains a valid
			// signature at the beginning of the slot)
			storageRead();
			uint32_t* ptr = (uint32_t*)storageGetData();
			if(kPresetHeader == ptr[0]) {
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
		storageInit(kPresetSector, slot);
		uint32_t* ptr = (uint32_t*)storageGetData();
		presetLoadDefaults();
		ptr[0] = kPresetHeader;
	} else {
		slot = found;
		storageInit(kPresetSector, slot);
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
	uint32_t oldSlot = p.slot;
	p.slot = getNextSlot(p.slot);
	if(p.slot < oldSlot)
	{
		// we wrapped around, so we need to erase the full sector in order to
		// write to it. If a reset happens here, we are out of luck and we may
		// have no valid preset in memory.
		storageErase(kPresetSector);
	}
	storageInit(kPresetSector, p.slot);
	// write to flash
	int ret = storageWrite();
	if(ret)
		return makeNegative(ret);
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
