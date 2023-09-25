#pragma once

#ifdef __cplusplus
extern "C" {
#endif //  __cplusplus
#include <stdint.h>
#include <stddef.h>

typedef void* PresetField_t;
typedef uint8_t PresetFieldSize_t;
/**
 * A callback that sets the preset data to its default value.
 */
typedef void (*PresetDefaulter_t)(PresetField_t field, PresetFieldSize_t size, void* data);
/**
 * A callback called when the content of a field is loaded from storage.
 */
typedef void (*PresetLoadCallback_t)(PresetField_t field, PresetFieldSize_t size, const void* data);

typedef struct
{
	PresetField_t field;
	PresetFieldSize_t size; // field size expressed in StorageWord_t units
	PresetDefaulter_t defaulter;
	PresetLoadCallback_t loadCallback;
	// the below are set automatically by presetInit()
	uint32_t offset;
} PresetDesc_t;

enum { kNumPresets  = 7 };

typedef enum {
	kPresetInit_LoadDefault,
	kPresetInit_LoadLatest,
} PresetInitOptions_t;

/**
 * Initialise the preset.
 *
 * @param checkSaveTimeout how long after the last change should
 * presetCheckSave() write to storage. This is expressed in the units returned
 * by getTime(). If presetCheckSave() is never called, this is ignored.
 * @param getTime a function that returns the current time at any time.
 * @return -1 if the defaults are loaded, or the index of the block that was loaded
 */
int presetInit(PresetInitOptions_t option, uint32_t checkSaveTimeout, uint32_t (*getTime)());
/**
 * Set a description for a preset.
 */
void presetDescSet(size_t idx, PresetDesc_t* desc);
/**
 * Load the default preset.
 */
void presetLoadDefaults();
/**
 * Load the presets from storage.
 */
int presetLoad();
/**
 * Store the current state of the preset to storage.
 */
int presetSave();
/**
 * Store the current state of the preset to storage if enough time has passed
 * since it was last changed.
 * Call this periodically in alternative to calling presetSave().
 *
 * @return -1 if no write was performed, or the positive number of the written
 * location otherwise.
 */
int presetCheckSave();
/// The methods below do not perform any I/O operation to/from storage.
/**
 * Set a field.
 */
void presetSetField(PresetField_t field, const void* ptr);
/**
 * Get the value of a field.
 */
const void* presetGetField(PresetField_t field);
/**
 * Get the size of a field.
 */
PresetFieldSize_t presetGetFieldSize(PresetField_t field);
/**
 * Check whether the preset has been edited since it was last written to storage.
 */
uint8_t presetIsSynced();
/**
 * Erases all flash sectors dedicated to presets.
 * "static method"
 */
int presetEraseAll();
/**
 * Call this if you want that next time presetCheckSave() is called
 * it will indeed write to storage.
 */
int presetTriggerFlushToStorage();
#ifdef __cplusplus
}
#endif //  __cplusplus
