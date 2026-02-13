#include <SPI.h>
#include "SdFat.h"

#define SD_FAT_TYPE 3

const uint8_t SD_MOSI   = 42;
const uint8_t SD_MISO   = 21;
const uint8_t SD_SCK    = 39;
const uint8_t SD_CS_PIN = 45;

// For SD_FAT_TYPE = 3, use SdFs + FsFile (matches the example pattern)
SdFs sd;
FsFile file;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    yield();
  }

  Serial.println("Initializing SPI...");
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);

  // IMPORTANT: This is what actually initializes the SD card for SdFat
  // Keep SPI.begin as-is; just add this sd.begin() + config step.
  SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI);

  Serial.println("Initializing SD...");
  if (!sd.begin(sdConfig)) {
    sd.initErrorHalt(&Serial);  // prints a useful reason
  }
  Serial.println("SD init done.");

  // --- Rewritten to match the SoftSPI example style ---

  if (!file.open("test.txt", O_RDWR | O_CREAT | O_TRUNC)) {
    sd.errorHalt(&Serial, F("open failed"));
  }

  file.println(F("testing 1, 2, 3."));
  file.println(F("hello sd card!"));

  file.rewind();

  Serial.println(F("test.txt:"));
  while (file.available()) {
    Serial.write(file.read());
  }

  file.close();
  Serial.println(F("Done."));
}

void loop() {}