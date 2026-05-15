// TODO:
// - Overlay audio mixing thru normalization w/ cap: mixed = base * BASE_GAIN + overlay * OVERLAY_GAIN;
// - Add in buttons for play/pause, record (start/stop, hold for recording window, or fixed time?)
// - Safety checks:
//      - Don't corrupt whole mixtape if power lost or sd card pulled
//      - If overlay is corrupted, don't use
// - Performance: Right now, writing the track occurs after it's closed and before playing the next one
//      - Can we try to write the previous track w/ overlay while playing current one? What are the PSRAM limitations for that?

#include <ArduinoJson.h>
#include <SPI.h>
#include <FS.h>
#include <math.h>
#include "Audio.h"  // https://github.com/schreibfaul1/ESP32-audioI2S/wiki
#include <cstdio>  // for printing
#include <cinttypes>  // for printing
extern "C" {
  #include "driver/i2s_std.h"
}
#include <SD.h>  // Audio.h playback uses the ESP32 SD filesystem object
#include <esp_heap_caps.h>

// =========================
// --- SD setup ---
// =========================
const uint8_t SD_MOSI   = 42;
const uint8_t SD_MISO   = 21;
const uint8_t SD_SCK    = 39;
const uint8_t SD_CS_PIN = 45;


void setupSD() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("SD init failed");
    while (true) delay(1000);
  }
}

// =========================
// --- Audio setup ---
// =========================
const i2s_port_t MIC_I2S_PORT = I2S_NUM_1;  // separate I2S mic path from playback path
#define MIC_SAMPLE_RATE   16000
#define MIC_PIN_SCK       TX
#define MIC_PIN_WS        RX  // LRCK
#define MIC_PIN_SD        9

i2s_chan_handle_t micRxChan = NULL;

#define OUTPUT_BCLK 6
#define OUTPUT_LRCK 7
#define OUTPUT_DOUT 8

#define SAMPLE_RATE 44100
#define TONE_FREQ   440.0
#define OUTPUT_BITS_PER_SAMPLE 16
#define AMPLITUDE 2048  // ~ -12 dBFS (out of 32767)
#define BUFFER_SAMPLES 256  // stereo frames

Audio audio;

// =========================
// audio recording config
// =========================
#define WAV_BITS_PER_SAMPLE 16  // 16-bit PCM WAV output
#define BUFFER_SIZE 1024  // Buffer size
#define VOLUME_SCALE 4.0f  // Mic gain after high-pass filter. Increase for audible debug WAV; clamp prevents overflow.
#define MIC_SAMPLE_SHIFT 14  // SPH0645 useful bits are in the high part of the 32-bit word.
#define MIC_STARTUP_SKIP_MS 250  // Suppress mic startup transient, while still preserving timing with zeros.
#define DURATION_SEC 10

// =========================
// --- Button setup ---
// =========================
const int PLAY_BUTTON_PIN = 12;
const int RECORD_BUTTON_PIN = 13;
bool isPaused = false;
bool lastPlayButtonState = HIGH;
bool lastRecordButtonState = HIGH;

// Global buffers for recording
int32_t buffer32[BUFFER_SIZE];  // Buffer for 32-bit data from the microphone
int16_t buffer16[BUFFER_SIZE];  // Buffer for 16-bit data to be saved to the file

// --- In-memory queue ---
static const int MAX_TRACKS   = 32;
static const int MAX_NAME_LEN = 48;
static const int MAX_PATH_LEN = 96;

// --- File info ---
char* MANIFEST_PATH = NULL;
char* TRACKS_DIR = NULL;

// --- Track info ---
struct TrackEntry {
  char name[MAX_NAME_LEN];
  char path[MAX_PATH_LEN];   // absolute path like "/demo/TRACKS/001.wav"
};

TrackEntry queue[MAX_TRACKS];
int queueLen = 0;
int currentTrack = 0;
bool trackStarted = false;
bool playbackDone = false;
uint32_t currentTrackStartMillis = 0;

// =========================
// TODO FEATURE 3: overlay capture + deferred mix state
// =========================
// First pass design:
// - when the record button is pressed, record mic audio into RAM while playback continues
// - when Audio::evt_eof fires, defer mixing until loop() so heavy file I/O does not run inside callback
// - render a new WAV copy with the mono mic overlay mixed into both L/R channels
// - update the queue path to point at the rendered copy for later replay
static const uint32_t OVERLAY_RECORD_SECONDS = 10;

// --- Knobs for recording normalization ---
// Desired overlay loudness relative to the base track in percentage
// (i.e. 0.75 means overlay RMS aims for 75% of base RMS).
static const float OVERLAY_TARGET_RELATIVE_RMS = 0.75f;

// Manual extra overlay boost after automatic RMS matching.
static const float OVERLAY_MANUAL_GAIN = 1.0f;

// Base-track ducking while overlay is active relative to track loudness
// (i.e. 0.70 = reduce base to 70%).
static const float BASE_DUCK_GAIN_DURING_OVERLAY = 0.75f;

// Hard safety limiter threshold for 16-bit WAV; keep below 32767 to leave headroom.
static const int32_t MIX_LIMIT = 30000;

// Prevent quiet background hiss from being boosted like crazy.
static const float MAX_AUTO_OVERLAY_GAIN = 6.0f;
static const float MIN_AUTO_OVERLAY_GAIN = 0.25f;

// Ignore tiny overlay RMS values below this threshold.
static const float OVERLAY_NOISE_FLOOR_RMS = 200.0f;

int16_t* overlaySamples = NULL;
volatile bool overlayRecording = false;
volatile bool overlayReady = false;
volatile uint32_t overlaySampleCount = 0;
int overlayTargetIndex = -1;
char overlayBasePath[MAX_PATH_LEN];
char overlayOutputPath[MAX_PATH_LEN];
bool mixPending = false;
int mixPendingIndex = -1;
uint32_t overlayStartMillisIntoTrack = 0;

void overlayRecordTask(void* param);
bool beginOverlayCaptureForCurrentTrack(int index);
void servicePendingOverlayMix();
bool renderOverlayMixToFile(const char* basePath, const char* outputPath, const int16_t* overlay, uint32_t overlayCount);

// DEBUG ONLY: standalone WAV of the mic overlay capture so recorded audio can be verified
// before debugging the later overlay/mixing step. This does not replace the future overlay path.
static const char* OVERLAY_DEBUG_WAV_PATH = "/overlay_debug.wav";
bool writeDebugMicWav(const char* path, const int16_t* samples, uint32_t sampleCount);

// DEBUG ONLY: temporarily stop the current track as soon as the 5-second overlay
// recording finishes so Audio::evt_eof / overlay rendering can be tested without
// waiting for the full song. Comment out the assignment in overlayRecordTask() and
// the block in loop() marked DEBUG FORCE EOF to restore normal behavior.
volatile bool debugForceEndTrackAfterOverlay = false;

void scheduleOverlayMixForTrackEnd();

struct WavInfo {
  uint32_t sampleRate;
  uint16_t channels;
  uint16_t bitsPerSample;
  uint32_t dataOffset;
  uint32_t dataSize;
};

bool readWavInfo(File& f, WavInfo& info);
bool computeBaseRMSForOverlayWindow(File& in, WavInfo& info, uint32_t overlayCount, float& baseRMS);

void scheduleOverlayMixForTrackEnd() {
  if (overlayTargetIndex == currentTrack && (overlayReady || overlayRecording)) {
    mixPending = true;
    mixPendingIndex = currentTrack;
  }
}

// event handler
void my_audio_info(Audio::msg_t m) {
  switch(m.e){
    case Audio::evt_info:           Serial.printf("info: ....... %s\n", m.msg); break;
    case Audio::evt_eof: {
      Serial.printf("end of file:  %s\n", m.msg); 
      // TODO FEATURE 3: mix only after playback finishes so we never write into a file while Audio.h is reading it.
      scheduleOverlayMixForTrackEnd();
      trackStarted = false;
      currentTrack++;
      break;
    }
    case Audio::evt_bitrate:        Serial.printf("bitrate: .... %s\n", m.msg); break; // icy-bitrate or bitrate from metadata
    case Audio::evt_icyurl:         Serial.printf("icy URL: .... %s\n", m.msg); break;
    case Audio::evt_id3data:        Serial.printf("ID3 data: ... %s\n", m.msg); break; // id3-data or metadata
    case Audio::evt_lasthost:       Serial.printf("last URL: ... %s\n", m.msg); break;
    case Audio::evt_name:           Serial.printf("station name: %s\n", m.msg); break; // station name or icy-name
    case Audio::evt_streamtitle:    Serial.printf("stream title: %s\n", m.msg); break;
    case Audio::evt_icylogo:        Serial.printf("icy logo: ... %s\n", m.msg); break;
    case Audio::evt_icydescription: Serial.printf("icy descr: .. %s\n", m.msg); break;
    case Audio::evt_image: for(int i = 0; i < m.vec.size(); i += 2){
                                    Serial.printf("cover image:  segment %02i, pos %07lu, len %05lu\n", i / 2, m.vec[i], m.vec[i + 1]);} break; // APIC
    case Audio::evt_lyrics:         Serial.printf("sync lyrics:  %s\n", m.msg); break;
    case Audio::evt_log   :         Serial.printf("audio_logs:   %s\n", m.msg); break;
    default:                        Serial.printf("message:..... %s\n", m.msg); break;
  }
}

// immediate recursive search function -- stretch to-do: fix so that we can find mixtape immediately
// ---- Directory utilities (ESP32 SD flavor) ----
static const char* baseNameFromPath(const char* path) {
  if (!path) return "";
  const char* slash = strrchr(path, '/');
  return slash ? slash + 1 : path;
}

char* findPath(const char* target, uint8_t levels, bool isTargetFile = false, const char* currentPath = "/") {
  if (levels == 0) return NULL;

  File root = SD.open(currentPath, FILE_READ);
  if (!root) {
    Serial.println("Failed to open directory");
    return NULL;
  }

  if (!root.isDirectory()) {
    root.close();
    return NULL;
  }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    const char* entryPathRaw = entry.name();
    const char* name = baseNameFromPath(entryPathRaw);

    char newPath[256];
    if (entryPathRaw && entryPathRaw[0] == '/') {
      snprintf(newPath, sizeof(newPath), "%s", entryPathRaw);
    } else if (strcmp(currentPath, "/") == 0) {
      snprintf(newPath, sizeof(newPath), "/%s", name);
    } else {
      snprintf(newPath, sizeof(newPath), "%s/%s", currentPath, name);
    }

    // Match a file if requested
    if (isTargetFile && !entry.isDirectory() && strcmp(name, target) == 0) {
      char* result = (char*)malloc(strlen(newPath) + 1);
      if (result != NULL) {
        strcpy(result, newPath);
      }
      entry.close();
      root.close();
      return result;
    }

    // Match a directory in the original mode
    if (!isTargetFile && entry.isDirectory() && strcmp(name, target) == 0) {
      char* result = (char*)malloc(strlen(newPath) + 1);
      if (result != NULL) {
        strcpy(result, newPath);
      }
      entry.close();
      root.close();
      return result;
    }

    // Recurse only into subdirectories
    if (entry.isDirectory()) {
      entry.close();
      char* found = findPath(target, levels - 1, isTargetFile, newPath);
      if (found != NULL) {
        root.close();
        return found;
      }
    } else {
      entry.close();
    }
  }

  root.close();
  return NULL;
}

//
void debugAllEntries(const char* currentPath = "/", uint8_t levels = 5) {
  if (levels == 0) return;

  File root = SD.open(currentPath, FILE_READ);
  if (!root) {
    Serial.print("Failed to open: ");
    Serial.println(currentPath);
    return;
  }

  if (!root.isDirectory()) {
    Serial.print("Not a dir: ");
    Serial.println(currentPath);
    root.close();
    return;
  }

  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;

    const char* entryPathRaw = entry.name();
    const char* name = baseNameFromPath(entryPathRaw);

    char fullPath[256];
    if (entryPathRaw && entryPathRaw[0] == '/') {
      snprintf(fullPath, sizeof(fullPath), "%s", entryPathRaw);
    } else if (strcmp(currentPath, "/") == 0) {
      snprintf(fullPath, sizeof(fullPath), "/%s", name);
    } else {
      snprintf(fullPath, sizeof(fullPath), "%s/%s", currentPath, name);
    }

    Serial.print(entry.isDirectory() ? "DIR  : " : "FILE : ");
    Serial.println(fullPath);

    if (entry.isDirectory()) {
      entry.close();
      debugAllEntries(fullPath, levels - 1);
    } else {
      entry.close();
    }
  }

  root.close();
}

bool fileExists(const char* path) {
  if (path == NULL || path[0] == '\0') return false;
  return SD.exists(path);
}

// Convert in-place:
// - backslashes -> slashes
// - ensure path starts with '/'
static void normalizePath(char* s) {
  if (!s || !s[0]) return;

  // convert '\' to '/'
  for (char* p = s; *p; p++) {
    if (*p == '\\') *p = '/';
  }

  // if it starts with something like "demo/..." add leading '/'
  if (s[0] != '/') {
    // shift right by 1 if we have room
    size_t len = strlen(s);
    if (len + 1 < MAX_PATH_LEN) {
      memmove(s + 1, s, len + 1);
      s[0] = '/';
    }
  }
}

// --- Queue utilities ---
static const size_t JSON_BUF_SIZE = 2048;
char jsonBuf[JSON_BUF_SIZE];

bool readFileToBuffer(const char* path, char* out, size_t outSize) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  size_t n = f.read((uint8_t*)out, outSize - 1);
  out[n] = '\0';
  f.close();
  return n > 0;
}

bool validateQueueFilesExist() {
  int out = 0;

  for (int i = 0; i < queueLen; i++) {
    if (fileExists(queue[i].path)) {
      if (out != i) queue[out] = queue[i];
      out++;
    } else {
      Serial.print("Missing track file: ");
      Serial.println(queue[i].path);
    }
  }

  queueLen = out;
  return queueLen > 0;
}

bool parsePlaylistAndBuildQueue(const char* json) {
  queueLen = 0;

  StaticJsonDocument<4096> doc; // bump if your manifest grows
  DeserializationError err = deserializeJson(doc, json);

  if (err) {
    Serial.print("deserializeJson failed: ");
    Serial.println(err.c_str());
    return false;
  }

  // Read top-level metadata (optional validations)
  uint32_t version = doc["version"] | 0;
  if (version != 1) {
    Serial.print("Unsupported manifest version: ");
    Serial.println(version);
    // You can choose to fail hard or allow it.
    // return false;
  }

  // Grab tracks array
  JsonArray tracks = doc["tracks"].as<JsonArray>();
  if (tracks.isNull()) {
    Serial.println("manifest missing tracks[] array");
    return false;
  }

  for (JsonObject t : tracks) {
    if (queueLen >= MAX_TRACKS) {
      Serial.println("Too many tracks; truncating to MAX_TRACKS");
      break;
    }

    const char* id   = t["id"]   | "";
    const char* path = t["path"] | "";

    // Use id as the display name (since your schema doesn't have "name")
    strncpy(queue[queueLen].name, id, MAX_NAME_LEN - 1);
    queue[queueLen].name[MAX_NAME_LEN - 1] = '\0';

    // Build an absolute path.
    // Cases handled:
    //   "save.wav"                         -> TRACKS_DIR + "/save.wav"
    //   "/MIXTAPE1/TRACKS/save.wav"        -> use as-is
    //   "MIXTAPE1/TRACKS/save.wav"         -> add leading slash, do NOT prepend TRACKS_DIR
    // This avoids invalid paths like /MIXTAPE1/TRACKS//MIXTAPE1/TRACKS/save.wav.
    while (*path == ' ' || *path == '\t' || *path == '\r' || *path == '\n') {
      path++;
    }

    bool pathHasSlash = strchr(path, '/') != NULL || strchr(path, '\\') != NULL;

    if (path[0] == '/') {
      strncpy(queue[queueLen].path, path, MAX_PATH_LEN - 1);
      queue[queueLen].path[MAX_PATH_LEN - 1] = '\0';
    } else if (pathHasSlash) {
      // Path already includes folders but is missing the leading slash.
      snprintf(queue[queueLen].path, MAX_PATH_LEN, "/%s", path);
    } else if (TRACKS_DIR != NULL && TRACKS_DIR[0] != '\0') {
      // Plain filename from manifest; resolve under discovered TRACKS directory.
      snprintf(queue[queueLen].path, MAX_PATH_LEN, "%s/%s", TRACKS_DIR, path);
    } else {
      strncpy(queue[queueLen].path, path, MAX_PATH_LEN - 1);
      queue[queueLen].path[MAX_PATH_LEN - 1] = '\0';
    }

    normalizePath(queue[queueLen].path);

    queueLen++;
  }

  return queueLen > 0;
}

void startTrack(int index) {
  if (index < 0 || index >= queueLen) {
    trackStarted = false;
    playbackDone = true;
    Serial.println("Reached end of queue.");
    return;
  }

  const char* path = queue[index].path;

  Serial.printf("Starting track %d/%d: %s\n", index + 1, queueLen, path);

  if (!audio.connecttoFS(SD, path)) {
    Serial.printf("connecttoFS failed: %s\n", path);
    trackStarted = false;
    currentTrack++;
    return;
  }

  trackStarted = true;
  currentTrackStartMillis = millis();

  // TODO FEATURE 3:
  // Overlay capture is now button-controlled by RECORD_BUTTON_PIN in loop(),
  // rather than starting automatically at the beginning of every track.
}

// void startTrack(int index) {
//   if (index < 0 || index >= queueLen) {
//     trackStarted = false;
//     playbackDone = true;
//     Serial.println("Reached end of queue.");
//     return;
//   }

//   const char* path = queue[index].path;

//   if (!fileExists(path)) {
//     Serial.printf("Missing file: %s\n", path);
//     trackStarted = false;
//     currentTrack++;
//     return;
//   }

//   Serial.printf("Starting track %d/%d: %s\n", index + 1, queueLen, path);

//   if (!audio.connecttoFS(SD, path)) {
//     Serial.printf("connecttoFS failed: %s\n", path);
//     trackStarted = false;
//     currentTrack++;
//     return;
//   }

//   trackStarted = true;

//   // TODO FEATURE 3:
//   // Start a first-pass automatic overlay capture when the track begins.
//   // Later this should be button-controlled instead of automatic.
//   beginOverlayCaptureForCurrentTrack(index);
// }

void debugAudioOutput() {
  Serial.println("------------AUDIO OBJECT CONFIG BELOW------------");
  Serial.printf("bit rate: %" PRIu8 "\n", audio.getBitsPerSample());
  Serial.printf("sample rate: %" PRIu8 "\n", audio.getSampleRate());
  Serial.printf("stereo: %" PRIu8 "\n", audio.getChannels());
}

// Function to update the WAV header when the exact data byte count is known.
// This is safer than relying on File::size() immediately after writes, because
// some SD implementations do not report the new size until after flush/close.
bool updateWAVHeaderWithDataSize(File &file, uint32_t dataSize) {
    uint32_t chunkSize = 36 + dataSize;

    if (!file.seek(4)) {  // Move to the ChunkSize field
      Serial.println("WAV header update failed: seek ChunkSize");
      return false;
    }
    if (file.write((const uint8_t *)&chunkSize, 4) != 4) {
      Serial.println("WAV header update failed: write ChunkSize");
      return false;
    }

    if (!file.seek(40)) {  // Move to the Subchunk2Size field
      Serial.println("WAV header update failed: seek Subchunk2Size");
      return false;
    }
    if (file.write((const uint8_t *)&dataSize, 4) != 4) {
      Serial.println("WAV header update failed: write Subchunk2Size");
      return false;
    }

    file.flush();
    return true;
}

// Fallback updater for callers that do not track data size explicitly.
bool updateWAVHeader(File &file) {
    file.flush();
    uint32_t fileSize = file.size();
    if (fileSize < 44) {
      Serial.printf("WAV header update failed: file smaller than 44 bytes, size=%lu\n", (unsigned long)fileSize);
      return false;
    }
    return updateWAVHeaderWithDataSize(file, fileSize - 44);
}

void writeWAVHeader(File &file, uint32_t sampleRate, uint16_t channels, uint16_t bitsPerSample) {
    uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
    uint16_t blockAlign = channels * (bitsPerSample / 8);

    file.write((const uint8_t *)"RIFF", 4);  // ChunkID
    uint32_t chunkSize = 0; // Size will be updated later
    file.write((const uint8_t *)&chunkSize, 4);  // ChunkSize
    file.write((const uint8_t *)"WAVE", 4);  // Format
    file.write((const uint8_t *)"fmt ", 4);  // Subchunk1ID
    uint32_t subChunk1Size = 16;
    file.write((const uint8_t *)&subChunk1Size, 4);  // Subchunk1Size
    uint16_t audioFormat = 1;  // PCM
    file.write((const uint8_t *)&audioFormat, 2);  // AudioFormat
    file.write((const uint8_t *)&channels, 2);  // NumChannels
    file.write((const uint8_t *)&sampleRate, 4);  // SampleRate
    file.write((const uint8_t *)&byteRate, 4);  // ByteRate
    file.write((const uint8_t *)&blockAlign, 2);  // BlockAlign
    file.write((const uint8_t *)&bitsPerSample, 2);  // BitsPerSample
    file.write((const uint8_t *)"data", 4);  // Subchunk2ID
    uint32_t subChunk2Size = 0; // Size will be updated later
    file.write((const uint8_t *)&subChunk2Size, 4);  // Subchunk2Size
}

void setupRecording() {
  if (micRxChan != NULL) {
    Serial.println("I2S mic driver already initialized.");
    return;
  }

  i2s_chan_config_t rxChanCfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_PORT, I2S_ROLE_MASTER);

  esp_err_t err = i2s_new_channel(&rxChanCfg, NULL, &micRxChan);
  if (err != ESP_OK) {
    Serial.printf("Failed creating I2S RX channel: %d\n", err);
    while (true) delay(1000);
  }

  // New ESP-IDF std I2S API. Do not include/use legacy driver/i2s.h in this sketch,
  // because Audio.h may use the newer I2S driver internally for playback.
  // Explicit mono-left slot config is used for the SPH0645.
  i2s_std_config_t rxStdCfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
    .slot_cfg = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
      .slot_mode = I2S_SLOT_MODE_MONO,
      .slot_mask = I2S_STD_SLOT_LEFT,
      .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
      .ws_pol = false,
      .bit_shift = true,
      .left_align = true,
      .big_endian = false,
      .bit_order_lsb = false,
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)MIC_PIN_SCK,
      .ws   = (gpio_num_t)MIC_PIN_WS,
      .dout = I2S_GPIO_UNUSED,
      .din  = (gpio_num_t)MIC_PIN_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv   = false,
      },
    },
  };

  err = i2s_channel_init_std_mode(micRxChan, &rxStdCfg);
  if (err != ESP_OK) {
    Serial.printf("Failed initializing I2S std RX mode: %d\n", err);
    while (true) delay(1000);
  }

  err = i2s_channel_enable(micRxChan);
  if (err != ESP_OK) {
    Serial.printf("Failed enabling I2S RX channel: %d\n", err);
    while (true) delay(1000);
  }

  Serial.println("I2S mic std RX driver installed.");
}

static int16_t clamp16(int32_t s) {
  if (s > MIX_LIMIT) return (int16_t)MIX_LIMIT;
  if (s < -MIX_LIMIT) return (int16_t)(-MIX_LIMIT);
  return (int16_t)s;
}

// Convert one raw 32-bit SPH0645 sample into a filtered 16-bit PCM sample.
// Keeps the filter state in caller-owned references so file recording and overlay recording can reset independently.
static int16_t processMicSample(int32_t raw, float& dcPrev, float& outPrev) {
  const float R = 0.995f;   // ~50 Hz cutoff at 16 kHz
  float x = (float)(raw >> MIC_SAMPLE_SHIFT);
  float y = x - dcPrev + R * outPrev;

  dcPrev = x;
  outPrev = y;

  int32_t s = (int32_t)(y * VOLUME_SCALE);
  return clamp16(s);
}

void recordAudio(const char *path, uint32_t duration) {
    if (SD.exists(path)) SD.remove(path);
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }

    // WAV file header
    writeWAVHeader(file, MIC_SAMPLE_RATE, 1, WAV_BITS_PER_SAMPLE);

    uint32_t samples = MIC_SAMPLE_RATE * duration;  // Number of samples
    uint32_t skipSamples = (MIC_SAMPLE_RATE * MIC_STARTUP_SKIP_MS) / 1000;  // skip initial mic activation transient
    uint32_t totalDataBytes = 0;

    float dcPrev = 0.0f;
    float outPrev = 0.0f;

    while (samples > 0) {
      size_t bytesRead = 0;

      esp_err_t err = i2s_channel_read(micRxChan, buffer32, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);
      if (err != ESP_OK) {
        Serial.printf("I2S read failed: %d\n", err);
        break;
      }

      int samplesRead = bytesRead / sizeof(int32_t);
      if (samplesRead <= 0) continue;
      if ((uint32_t)samplesRead > samples) samplesRead = samples;

      int writeCount = 0;
      for (int i = 0; i < samplesRead; i++) {
          int16_t s = processMicSample(buffer32[i], dcPrev, outPrev);

          // Skip initial mic activation samples, but still run filter state
          if (skipSamples > 0) {
              skipSamples--;
              buffer16[writeCount++] = 0; // preserve timing while avoiding startup pop
          } else {
              buffer16[writeCount++] = s;
          }
      }

      if (writeCount > 0) {
          size_t bytesToWrite = writeCount * sizeof(int16_t);
          size_t written = file.write((uint8_t *)buffer16, bytesToWrite);
          if (written != bytesToWrite) {
              Serial.printf("recordAudio short write %u/%u\n", (unsigned)written, (unsigned)bytesToWrite);
              break;
          }
          totalDataBytes += written;
      }

      samples -= samplesRead;
  }

    // Update the WAV file header using the known PCM data byte count.
    updateWAVHeaderWithDataSize(file, totalDataBytes);
    file.flush();
    file.close();
}

// DEBUG ONLY: write the captured overlay samples as a standalone mono 16-bit PCM WAV.
// This verifies that overlayRecordTask is executing and that the mic capture/filter path
// creates a valid WAV before the later track-mixing step touches the audio.
bool writeDebugMicWav(const char* path, const int16_t* samples, uint32_t sampleCount) {
  if (!path || !samples || sampleCount == 0) {
    Serial.println("DEBUG WAV: no samples to write");
    return false;
  }

  if (SD.exists(path)) {
    SD.remove(path);
  }

  File file = SD.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("DEBUG WAV: failed to open %s\n", path);
    return false;
  }

  writeWAVHeader(file, MIC_SAMPLE_RATE, 1, WAV_BITS_PER_SAMPLE);

  uint32_t bytesToWrite = sampleCount * sizeof(int16_t);
  size_t written = file.write((const uint8_t*)samples, bytesToWrite);
  if (written != bytesToWrite) {
    Serial.printf("DEBUG WAV: short write %u/%u\n", (unsigned)written, (unsigned)bytesToWrite);
    updateWAVHeaderWithDataSize(file, written);
    file.flush();
    file.close();
    return false;
  }

  updateWAVHeaderWithDataSize(file, bytesToWrite);
  file.flush();
  file.close();

  Serial.printf("DEBUG WAV: wrote %s (%lu samples, %lu data bytes)\n",
                path, (unsigned long)sampleCount, (unsigned long)bytesToWrite);
  return true;
}

// DEBUG ONLY: summarize the captured overlay buffer so silent output can be diagnosed
// without needing to inspect the WAV on another device.
void printOverlayCaptureStats(const int16_t* samples, uint32_t sampleCount) {
  if (!samples || sampleCount == 0) {
    Serial.println("OVERLAY DEBUG: no captured samples");
    return;
  }

  int16_t minVal = 32767;
  int16_t maxVal = -32768;
  uint32_t nonzero = 0;
  uint32_t clipped = 0;
  int64_t absSum = 0;

  for (uint32_t i = 0; i < sampleCount; i++) {
    int16_t s = samples[i];
    if (s < minVal) minVal = s;
    if (s > maxVal) maxVal = s;
    if (s != 0) nonzero++;
    if (s == 32767 || s == -32768) clipped++;
    absSum += (s < 0) ? -(int32_t)s : s;
  }

  Serial.printf("OVERLAY DEBUG: samples=%lu nonzero=%lu min=%d max=%d avgAbs=%ld clipped=%lu\n",
                (unsigned long)sampleCount,
                (unsigned long)nonzero,
                minVal,
                maxVal,
                (long)(absSum / (int64_t)sampleCount),
                (unsigned long)clipped);

  if (nonzero == 0) {
    Serial.println("OVERLAY DEBUG WARNING: captured overlay is all zeros. Check mic SEL/channel slot and I2S RX config.");
  } else if ((absSum / (int64_t)sampleCount) < 20) {
    Serial.println("OVERLAY DEBUG WARNING: captured overlay is extremely quiet. Increase VOLUME_SCALE or speak closer to the mic port.");
  }
}

// Build a new path beside the source track, e.g. /MIXTAPE1/TRACKS/save.wav -> /MIXTAPE1/TRACKS/save_overlay_12345.wav
bool makeOverlayOutputPath(const char* basePath, char* out, size_t outSize) {
  if (!basePath || !out || outSize == 0) return false;

  const char* slash = strrchr(basePath, '/');
  const char* fileName = slash ? slash + 1 : basePath;
  size_t dirLen = slash ? (size_t)(slash - basePath) : 0;

  char stem[48];
  strncpy(stem, fileName, sizeof(stem) - 1);
  stem[sizeof(stem) - 1] = '\0';
  char* dot = strrchr(stem, '.');
  if (dot) *dot = '\0';

  if (slash) {
    return snprintf(out, outSize, "%.*s/%s_overlay_%lu.wav", (int)dirLen, basePath, stem, (unsigned long)random(10000, 99999)) < (int)outSize;
  }
  return snprintf(out, outSize, "/%s_overlay_%lu.wav", stem, (unsigned long)random(10000, 99999)) < (int)outSize;
}

bool beginOverlayCaptureForCurrentTrack(int index) {
  if (index < 0 || index >= queueLen) return false;
  if (overlayRecording) return false;

  const uint32_t maxSamples = MIC_SAMPLE_RATE * OVERLAY_RECORD_SECONDS;

  if (overlaySamples) {
    free(overlaySamples);
    overlaySamples = NULL;
  }

  overlaySamples = (int16_t*)heap_caps_malloc(maxSamples * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!overlaySamples) overlaySamples = (int16_t*)malloc(maxSamples * sizeof(int16_t));

  if (!overlaySamples) {
    Serial.println("Failed to allocate overlay buffer");
    return false;
  }

  strncpy(overlayBasePath, queue[index].path, sizeof(overlayBasePath) - 1);
  overlayBasePath[sizeof(overlayBasePath) - 1] = '\0';

  if (!makeOverlayOutputPath(queue[index].path, overlayOutputPath, sizeof(overlayOutputPath))) {
    Serial.println("Failed to make overlay output path");
    free(overlaySamples);
    overlaySamples = NULL;
    return false;
  }

  overlayTargetIndex = index;
  overlaySampleCount = 0;
  overlayReady = false;
  overlayRecording = true;

  BaseType_t ok = xTaskCreatePinnedToCore(overlayRecordTask, "overlayRecordTask", 8192, NULL, 2, NULL, 1);
  if (ok != pdPASS) {
    Serial.println("Failed to start overlay record task");
    overlayRecording = false;
    free(overlaySamples);
    overlaySamples = NULL;
    return false;
  }

  Serial.printf("TODO FEATURE 3: recording overlay for %s -> %s\n", overlayBasePath, overlayOutputPath);
  return true;
}

void overlayRecordTask(void* param) {
  Serial.println("TODO FEATURE 3: overlayRecordTask started");

  const uint32_t maxSamples = MIC_SAMPLE_RATE * OVERLAY_RECORD_SECONDS;
  uint32_t skipSamples = (MIC_SAMPLE_RATE * MIC_STARTUP_SKIP_MS) / 1000;
  uint32_t captured = 0;
  uint32_t readBlocks = 0;
  int32_t rawMin = INT32_MAX;
  int32_t rawMax = INT32_MIN;
  uint32_t rawNonzero = 0;
  float dcPrev = 0.0f;
  float outPrev = 0.0f;

  while (captured < maxSamples) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_channel_read(micRxChan, buffer32, BUFFER_SIZE * sizeof(int32_t), &bytesRead, pdMS_TO_TICKS(250));
    if (err != ESP_OK || bytesRead == 0) continue;

    readBlocks++;
    int samplesRead = bytesRead / sizeof(int32_t);
    for (int i = 0; i < samplesRead && captured < maxSamples; i++) {
      int32_t raw = buffer32[i];
      if (raw < rawMin) rawMin = raw;
      if (raw > rawMax) rawMax = raw;
      if (raw != 0) rawNonzero++;

      int16_t s = processMicSample(raw, dcPrev, outPrev);

      // Keep timing aligned with the track while suppressing startup clipping.
      if (skipSamples > 0) {
        skipSamples--;
        s = 0;
      }

      overlaySamples[captured++] = s;
    }
  }

  overlaySampleCount = captured;
  overlayReady = true;
  overlayRecording = false;

  Serial.printf("TODO FEATURE 3: overlay capture complete (%lu samples from %lu reads)\n",
                (unsigned long)captured, (unsigned long)readBlocks);
  Serial.printf("OVERLAY DEBUG RAW: nonzero=%lu min=%ld max=%ld shift=%d gain=%.2f\n",
                (unsigned long)rawNonzero, (long)rawMin, (long)rawMax, MIC_SAMPLE_SHIFT, (double)VOLUME_SCALE);
  printOverlayCaptureStats(overlaySamples, overlaySampleCount);

  // DEBUG ONLY: write the exact captured overlay buffer to a standalone WAV.
  // If this file is valid and audible, the mic capture task and WAV header path are working,
  // and any later problem is likely in renderOverlayMixToFile().
  writeDebugMicWav(OVERLAY_DEBUG_WAV_PATH, overlaySamples, overlaySampleCount);

  // DEBUG FORCE EOF: request loop() to stop the current track immediately after
  // recording finishes, instead of waiting for the song's natural EOF.
  // Comment out this assignment to restore normal full-track playback.
  // Button-controlled recording should not force the song to end.
  // Let the current track play out normally; Audio::evt_eof will schedule mixing.
  debugForceEndTrackAfterOverlay = false;

  vTaskDelete(NULL);
}

static uint16_t readLE16(File& f) {
  uint8_t b[2];
  f.read(b, 2);
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t readLE32(File& f) {
  uint8_t b[4];
  f.read(b, 4);
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}


static float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static float computeOverlayRMS16(const int16_t* samples, uint32_t count) {
  if (!samples || count == 0) return 0.0f;

  double sumSquares = 0.0;
  for (uint32_t i = 0; i < count; i++) {
    double s = (double)samples[i];
    sumSquares += s * s;
  }

  return (float)sqrt(sumSquares / (double)count);
}

bool computeBaseRMSForOverlayWindow(File& in, WavInfo& info, uint32_t overlayCount, float& baseRMS) {
  baseRMS = 0.0f;
  if (overlayCount == 0 || info.bitsPerSample != 16 || (info.channels != 1 && info.channels != 2)) return false;

  in.seek(info.dataOffset);

  const size_t FRAME_BLOCK = 128;
  int16_t frames[FRAME_BLOCK * 2];
  uint32_t frameCount = info.dataSize / (info.channels * sizeof(int16_t));
  uint32_t frameIndex = 0;
  uint32_t overlayStartFrame = ((uint64_t)overlayStartMillisIntoTrack * info.sampleRate) / 1000;
  double sumSquares = 0.0;
  uint32_t valueCount = 0;

  while (frameIndex < frameCount) {
    uint32_t framesThis = min((uint32_t)FRAME_BLOCK, frameCount - frameIndex);
    size_t bytesToRead = framesThis * info.channels * sizeof(int16_t);
    size_t actuallyRead = in.read((uint8_t*)frames, bytesToRead);
    if (actuallyRead != bytesToRead) break;

    for (uint32_t f = 0; f < framesThis; f++) {
      if ((frameIndex + f) < overlayStartFrame) {
        continue;
      }

      uint64_t baseFramesSinceOverlayStart = (frameIndex + f) - overlayStartFrame;
      uint64_t overlayIdx = (baseFramesSinceOverlayStart * MIC_SAMPLE_RATE) / info.sampleRate;
      if (overlayIdx >= overlayCount) {
        goto done_rms;
      }

      for (uint16_t c = 0; c < info.channels; c++) {
        int16_t baseSample = frames[f * info.channels + c];
        double s = (double)baseSample;
        sumSquares += s * s;
        valueCount++;
      }
    }

    frameIndex += framesThis;
  }

done_rms:
  if (valueCount == 0) return false;
  baseRMS = (float)sqrt(sumSquares / (double)valueCount);
  return true;
}

bool readWavInfo(File& f, WavInfo& info) {
  memset(&info, 0, sizeof(info));
  f.seek(0);

  char id[5] = {0};
  f.read((uint8_t*)id, 4);
  if (strcmp(id, "RIFF") != 0) return false;
  readLE32(f);
  f.read((uint8_t*)id, 4);
  if (strcmp(id, "WAVE") != 0) return false;

  bool gotFmt = false;
  bool gotData = false;

  while (f.available()) {
    f.read((uint8_t*)id, 4);
    uint32_t chunkSize = readLE32(f);
    uint32_t chunkStart = f.position();

    if (strncmp(id, "fmt ", 4) == 0) {
      uint16_t audioFormat = readLE16(f);
      info.channels = readLE16(f);
      info.sampleRate = readLE32(f);
      readLE32(f); // byteRate
      readLE16(f); // blockAlign
      info.bitsPerSample = readLE16(f);
      if (audioFormat != 1) return false; // PCM only for now
      gotFmt = true;
    } else if (strncmp(id, "data", 4) == 0) {
      info.dataOffset = f.position();
      info.dataSize = chunkSize;
      gotData = true;
      break;
    }

    f.seek(chunkStart + chunkSize + (chunkSize & 1));
  }

  return gotFmt && gotData && info.bitsPerSample == 16 && (info.channels == 1 || info.channels == 2);
}

bool renderOverlayMixToFile(const char* basePath, const char* outputPath, const int16_t* overlay, uint32_t overlayCount) {
  if (!basePath || !outputPath || !overlay || overlayCount == 0) return false;

  File in = SD.open(basePath, FILE_READ);
  if (!in) {
    Serial.printf("Could not open base track for mix: %s\n", basePath);
    return false;
  }

  WavInfo info;
  if (!readWavInfo(in, info)) {
    Serial.println("Base track is not supported 16-bit PCM WAV");
    in.close();
    return false;
  }

  // TODO FEATURE 3: automatic overlay loudness matching.
  // Measure the base track RMS over the same time window as the overlay, then
  // scale the overlay toward OVERLAY_TARGET_RELATIVE_RMS of that base RMS.
  // This keeps quiet recordings quieter than loud recordings, while still
  // boosting normal speech enough to sit on top of the song.
  float baseRMS = 0.0f;
  float overlayRMS = computeOverlayRMS16(overlay, overlayCount);
  bool haveBaseRMS = computeBaseRMSForOverlayWindow(in, info, overlayCount, baseRMS);

  float autoOverlayGain = 1.0f;
  if (haveBaseRMS && baseRMS > 0.0f && overlayRMS > OVERLAY_NOISE_FLOOR_RMS) {
    autoOverlayGain = (baseRMS * OVERLAY_TARGET_RELATIVE_RMS) / overlayRMS;
    autoOverlayGain = clampFloat(autoOverlayGain, MIN_AUTO_OVERLAY_GAIN, MAX_AUTO_OVERLAY_GAIN);
  }

  float finalOverlayGain = autoOverlayGain * OVERLAY_MANUAL_GAIN;

  Serial.printf("TODO FEATURE 3 MIX: baseRMS=%.2f overlayRMS=%.2f autoOverlayGain=%.2f manualGain=%.2f finalOverlayGain=%.2f duck=%.2f\n",
                baseRMS,
                overlayRMS,
                autoOverlayGain,
                OVERLAY_MANUAL_GAIN,
                finalOverlayGain,
                BASE_DUCK_GAIN_DURING_OVERLAY);

  if (SD.exists(outputPath)) SD.remove(outputPath);
  File out = SD.open(outputPath, FILE_WRITE);
  if (!out) {
    Serial.printf("Could not open mix output: %s\n", outputPath);
    in.close();
    return false;
  }

  writeWAVHeader(out, info.sampleRate, info.channels, 16);
  in.seek(info.dataOffset);

  const size_t FRAME_BLOCK = 128;
  int16_t frames[FRAME_BLOCK * 2]; // supports mono or stereo base track
  uint32_t frameCount = info.dataSize / (info.channels * sizeof(int16_t));
  uint32_t frameIndex = 0;
  uint32_t overlayStartFrame = ((uint64_t)overlayStartMillisIntoTrack * info.sampleRate) / 1000;
  uint32_t bytesWritten = 0;

  printOverlayCaptureStats(overlay, overlayCount);

  while (frameIndex < frameCount) {
    uint32_t framesThis = min((uint32_t)FRAME_BLOCK, frameCount - frameIndex);
    size_t bytesToRead = framesThis * info.channels * sizeof(int16_t);
    size_t actuallyRead = in.read((uint8_t*)frames, bytesToRead);
    if (actuallyRead != bytesToRead) break;

    for (uint32_t f = 0; f < framesThis; f++) {
      bool overlayActive = false;
      uint64_t overlayIdx = 0;
      int32_t overlaySample = 0;

      if ((frameIndex + f) >= overlayStartFrame) {
        uint64_t baseFramesSinceOverlayStart = (frameIndex + f) - overlayStartFrame;
        overlayIdx = (baseFramesSinceOverlayStart * MIC_SAMPLE_RATE) / info.sampleRate;
        overlayActive = overlayIdx < overlayCount;
      }

      if (overlayActive) {
        overlaySample = (int32_t)(overlay[overlayIdx] * finalOverlayGain);
      }

      for (uint16_t c = 0; c < info.channels; c++) {
        uint32_t idx = f * info.channels + c;
        float baseGain = overlayActive ? BASE_DUCK_GAIN_DURING_OVERLAY : 1.0f;
        int32_t mixed = (int32_t)(frames[idx] * baseGain) + overlaySample;
        frames[idx] = clamp16(mixed);
      }
    }

    size_t written = out.write((uint8_t*)frames, bytesToRead);
    if (written != bytesToRead) {
      Serial.println("Mix write failed");
      in.close();
      out.close();
      return false;
    }
    bytesWritten += written;
    frameIndex += framesThis;
  }

  updateWAVHeaderWithDataSize(out, bytesWritten);
  out.flush();
  out.close();
  in.close();
  Serial.printf("TODO FEATURE 3: wrote mixed track: %s\n", outputPath);
  return bytesWritten > 0;
}

void servicePendingOverlayMix() {
  if (!mixPending) return;

  if (overlayRecording) {
    // Track ended before the 5-second capture finished; wait until capture task completes.
    return;
  }

  if (!overlayReady || overlaySampleCount == 0 || mixPendingIndex < 0 || mixPendingIndex >= queueLen) {
    mixPending = false;
    return;
  }

  Serial.println("TODO FEATURE 3: rendering overlay mix after EOF...");
  bool ok = renderOverlayMixToFile(overlayBasePath, overlayOutputPath, overlaySamples, overlaySampleCount);
  if (ok) {
    // Make the mixed file the queue's new version of this track for later replay.
    strncpy(queue[mixPendingIndex].path, overlayOutputPath, MAX_PATH_LEN - 1);
    queue[mixPendingIndex].path[MAX_PATH_LEN - 1] = '\0';
  }

  free(overlaySamples);
  overlaySamples = NULL;
  overlayReady = false;
  overlaySampleCount = 0;
  overlayTargetIndex = -1;
  mixPendingIndex = -1;
  mixPending = false;
}

void setup() {
  Serial.begin(115200);
  while(!Serial) {yield();}

  Serial.println("Initializing SPI...");
  Audio::audio_info_callback = my_audio_info;

  setupSD();
  Serial.println("SD init done.");

  setupRecording();

  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RECORD_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Locating manifest:");
  MANIFEST_PATH = findPath("playlist-manifest.json", 5, true);

  if (MANIFEST_PATH == NULL || !fileExists(MANIFEST_PATH)) {
    Serial.println("Manifest not found");
    Serial.println("Check folder name/case and that it’s on the SD (not just your PC).");
    while (true) delay(1000);
  }

  if (!readFileToBuffer(MANIFEST_PATH, jsonBuf, sizeof(jsonBuf))) {
    Serial.println("Failed to read manifest");
    while (true) delay(1000);
  }

  Serial.println("Locating TRACKS:");
  TRACKS_DIR = findPath("TRACKS", 5);

  if (!parsePlaylistAndBuildQueue(jsonBuf)) {
    Serial.println("Failed to parse manifest");
    while (true) delay(1000);
  }

  if (!validateQueueFilesExist()) {
    Serial.println("No valid tracks after validation. Check /demo/TRACKS and filenames.");
    while (true) delay(1000);
  }

  Serial.print("Queue built. Tracks = ");
  Serial.println(queueLen);

  for (int i = 0; i < queueLen; i++) {
    Serial.printf("%d: %s -> %s\n", i, queue[i].name, queue[i].path);
  }

  // from https://dronebotworkshop.com/esp32-i2s/
  Serial.println("Ready for playback stage.");
  delay(1000);
  pinMode(SD_CS_PIN, OUTPUT);      
  digitalWrite(SD_CS_PIN, HIGH); 

  // if (!SD.begin(SD_CS_PIN, SPI)) {
  //   Serial.println("SD init failed");
  //   while (true) delay(1000);
  // }
  audio.setPinout(OUTPUT_BCLK, OUTPUT_LRCK, OUTPUT_DOUT);
  audio.setVolume(9);
  audio.setOutput48KHz(SAMPLE_RATE);
  // debugAudioOutput();
  startTrack(currentTrack);

}


void loop() {
  // =========================
  // PLAY / PAUSE BUTTON
  // =========================

  bool currentPlayButtonState = digitalRead(PLAY_BUTTON_PIN);

  // Detect button press (HIGH -> LOW)
  if (lastPlayButtonState == HIGH && currentPlayButtonState == LOW) {
    isPaused = !isPaused;

    if (isPaused) {
      Serial.println("pause");
      audio.pauseResume();
    } else {
      Serial.println("play");
      audio.pauseResume();
    }

    delay(200); // simple debounce
  }

  lastPlayButtonState = currentPlayButtonState;


  // =========================
  // RECORD BUTTON
  // =========================

  bool currentRecordButtonState = digitalRead(RECORD_BUTTON_PIN);

  // Detect button press (HIGH -> LOW)
  if (lastRecordButtonState == HIGH && currentRecordButtonState == LOW) {
    if (!trackStarted || playbackDone) {
      Serial.println("record ignored: no active track");
    } else if (isPaused) {
      Serial.println("record ignored: playback is paused");
    } else if (overlayRecording) {
      Serial.println("record ignored: already recording");
    } else if (overlayReady || mixPending) {
      Serial.println("record ignored: overlay already captured for this track");
    } else {
      overlayStartMillisIntoTrack = millis() - currentTrackStartMillis;
      Serial.printf("recording at %lu ms into track\n", (unsigned long)overlayStartMillisIntoTrack);
      beginOverlayCaptureForCurrentTrack(currentTrack);
    }

    delay(200); // simple debounce
  }

  lastRecordButtonState = currentRecordButtonState;


  // =========================
  // Audio loop
  // =========================

  if (!isPaused) {
    audio.loop();
  }

  // DEBUG FORCE EOF should now remain false for button-controlled recording.
  // This block can stay as a safety/debug hook, but normal behavior is to let
  // the current track play out and mix only after Audio::evt_eof.
  if (debugForceEndTrackAfterOverlay && trackStarted) {
    debugForceEndTrackAfterOverlay = false;
    Serial.println("DEBUG FORCE EOF: stopping current track after overlay capture");
    audio.stopSong();
    scheduleOverlayMixForTrackEnd();
    trackStarted = false;
    currentTrack++;
  }

  // TODO FEATURE 3:
  // - record external audio using the record button for exactly OVERLAY_RECORD_SECONDS
  // - make sure when we overlay recording over currently playing track, the recording's audio signals still go to both channels because the recording only happens on left channel
  // - track should still be playing while recording audio (if possible)
  // Current button-controlled pass:
  // - startTrack() only starts playback
  // - RECORD_BUTTON_PIN starts a 5/10-second overlay capture into RAM while playback continues
  // - Audio::evt_eof schedules mixing only if an overlay was captured
  // - servicePendingOverlayMix() renders a new mixed WAV after EOF, then updates the queue path
  servicePendingOverlayMix();

  if (!trackStarted && !playbackDone && !mixPending && !isPaused) {
    startTrack(currentTrack);
  }
}
