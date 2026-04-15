// TODO:
// - Less fragile manifest identifier function (not from abs path)
// - Audio output monitor
// - Audio issue:
//    - I2S config:
  //    - bits/sample
  //    - sample rate
  //    - # of channels
//    - mono vs. stereo

#include <ArduinoJson.h>
#include <SPI.h>
#include "SdFat.h"
// #include <driver/i2s.h>
#include <math.h>
#include "Audio.h"  // https://github.com/schreibfaul1/ESP32-audioI2S/wiki
#include <cstdio>  // for printing
#include <cinttypes>  // for printing
#include <driver/i2s.h>

// --- SD setup (unchanged) ---
#define SD_FAT_TYPE 3
const uint8_t SD_MOSI   = 42;
const uint8_t SD_MISO   = 21;
const uint8_t SD_SCK    = 39;
const uint8_t SD_CS_PIN = 45;

SdFs sd;

// --- Audio setup ---
#define I2S_BCLK 6
#define I2S_LRCK 7
#define I2S_DOUT 8

// todo: play around w/ audio settings (# of bits, sample rate, etc.)
#define SAMPLE_RATE 44100
#define TONE_FREQ   440.0
#define BITS_PER_SAMPLE 128
#define AMPLITUDE 2048  // ~ -12 dBFS (out of 32767)
#define BUFFER_SAMPLES 256  // stereo frames

Audio audio;
void my_audio_info(Audio::msg_t m) {
  switch(m.e){
    case Audio::evt_info:           Serial.printf("info: ....... %s\n", m.msg); break;
    case Audio::evt_eof:            Serial.printf("end of file:  %s\n", m.msg); break;
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

// TO-DO: CHANGE FROM HARD-CODED PATH TO MANIFEST IDENTIFIER
// --- Demo file structure ---
static const char* PLAYLIST_PATH = "/demo/playlist-manifest.json";
static const char* TRACKS_DIR    = "/demo/TRACKS";

// --- In-memory queue ---
static const int MAX_TRACKS   = 32;
static const int MAX_NAME_LEN = 48;
static const int MAX_PATH_LEN = 96;

struct TrackEntry {
  char name[MAX_NAME_LEN];
  char path[MAX_PATH_LEN];   // absolute path like "/demo/TRACKS/001.wav"
};

TrackEntry queue[MAX_TRACKS];
int queueLen = 0;

// ---- Diagnostics: list directory (SdFat flavor) ----
void listDir(const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  FsFile dir;
  if (!dir.open(dirname)) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!dir.isDir()) {
    Serial.println("Not a directory");
    dir.close();
    return;
  }

  FsFile entry;
  while (entry.openNext(&dir, O_RDONLY)) {
    char name[96];
    if (!entry.getName(name, sizeof(name))) {
      strcpy(name, "<noname>");
    }

    if (entry.isDir()) {
      Serial.print("  DIR : ");
      Serial.println(name);

      if (levels) {
        char subpath[160];
        // Avoid '//' when dirname is "/"
        if (strcmp(dirname, "/") == 0) {
          snprintf(subpath, sizeof(subpath), "/%s", name);
        } else {
          snprintf(subpath, sizeof(subpath), "%s/%s", dirname, name);
        }
        listDir(subpath, levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(name);
      Serial.print("  SIZE: ");
      Serial.println((uint32_t)entry.fileSize());
    }

    entry.close();
  }

  dir.close();
}


// --- Utility: open+exist check ---
bool fileExists(const char* path) {
  FsFile tmp;
  bool ok = tmp.open(path, O_RDONLY);
  if (ok) tmp.close();
  return ok;
}

// --- Utility: read entire file into a buffer (playlist-manifest.json should be small) ---
static const size_t JSON_BUF_SIZE = 2048;
char jsonBuf[JSON_BUF_SIZE];

bool readFileToBuffer(const char* path, char* out, size_t outSize) {
  FsFile f;
  if (!f.open(path, O_RDONLY)) return false;
  size_t n = f.read(out, outSize - 1);
  out[n] = '\0';
  f.close();
  return n > 0;
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

    if (path[0] == '\0') {
      Serial.println("Skipping track with missing \"path\"");
      continue;
    }

    // Use id as the display name (since your schema doesn't have "name")
    strncpy(queue[queueLen].name, id, MAX_NAME_LEN - 1);
    queue[queueLen].name[MAX_NAME_LEN - 1] = '\0';

    // Copy path as-is then normalize for SdFat paths
    strncpy(queue[queueLen].path, path, MAX_PATH_LEN - 1);
    queue[queueLen].path[MAX_PATH_LEN - 1] = '\0';

    normalizePath(queue[queueLen].path);

    queueLen++;
  }

  return queueLen > 0;
}

int16_t i2s_buffer[BUFFER_SAMPLES * 2]; // stereo (L,R)

// void setupI2S() {
//   i2s_config_t i2s_config = {
//     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
//     .sample_rate = SAMPLE_RATE,
//     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
//     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // stereo
//     .communication_format = I2S_COMM_FORMAT_I2S,  // standard I2S
//     .intr_alloc_flags = 0,
//     .dma_buf_count = 8,
//     .dma_buf_len = 128,
//     .use_apll = true,
//     .tx_desc_auto_clear = true,
//     .fixed_mclk = 0
//   };

//   i2s_pin_config_t pin_config = {
//     .bck_io_num = I2S_BCLK,
//     .ws_io_num = I2S_LRCK,
//     .data_out_num = I2S_DOUT,
//     .data_in_num = I2S_PIN_NO_CHANGE
//   };

//   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
//   i2s_set_pin(I2S_NUM_0, &pin_config);
// }

void setup() {
  Serial.begin(115200);
  // while (!Serial) { yield(); }
  // setupI2S();

  // Serial.println("Initializing SPI...");
  // Audio::audio_info_callback = my_audio_info;

  // SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI);

  // Serial.println("Initializing SD...");
  // if (!sd.begin(sdConfig)) {
  //   sd.initErrorHalt(&Serial);
  // }
  // Serial.println("SD init done.");

  // // Debug: list what the ESP32 actually sees on the SD
  // listDir("/", 2);

  // Serial.print("Locating manifest: ");
  // Serial.println(PLAYLIST_PATH);

  // if (!fileExists(PLAYLIST_PATH)) {
  //   Serial.println("Manifest not found at /demo/playlist-manifest.json");
  //   Serial.println("Check folder name/case and that it’s on the SD (not just your PC).");
  //   while (true) delay(1000);
  // }

  // if (!readFileToBuffer(PLAYLIST_PATH, jsonBuf, sizeof(jsonBuf))) {
  //   Serial.println("Failed to read manifest");
  //   while (true) delay(1000);
  // }

  // if (!parsePlaylistAndBuildQueue(jsonBuf)) {
  //   Serial.println("Failed to parse manifest");
  //   while (true) delay(1000);
  // }

  // if (!validateQueueFilesExist()) {
  //   Serial.println("No valid tracks after validation. Check /demo/TRACKS and filenames.");
  //   while (true) delay(1000);
  // }

  // Serial.print("Queue built. Tracks = ");
  // Serial.println(queueLen);

  // for (int i = 0; i < queueLen; i++) {
  //   Serial.printf("%d: %s -> %s\n", i, queue[i].name, queue[i].path);
  // }

  // from https://dronebotworkshop.com/esp32-i2s/
  Serial.println("Ready for playback stage.");
  delay(1000);
  // Serial.println("I2S audio test");
  // setupI2S();
  pinMode(SD_CS_PIN, OUTPUT);      
  digitalWrite(SD_CS_PIN, HIGH); 

  

  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("SD init failed");
    while (true) delay(1000);
  }
  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  audio.setVolume(9);
  audio.setOutput48KHz(SAMPLE_RATE);
  Serial.println("------------AUDIO OBJECT CONFIG BELOW------------");
  Serial.printf("bit rate: %" PRIu8 "\n", audio.getBitsPerSample());
  Serial.printf("sample rate: %" PRIu8 "\n", audio.getSampleRate());
  Serial.printf("stereo: %" PRIu8 "\n", audio.getChannels());

  // Put exactly one known-good file on the card root first.
  // if (!audio.connecttoFS(SD, "/MIXTAPE1/TRACKS/save.wav")) {
  if (!audio.connecttoFS(SD, "/MIXTAPE1/TRACKS/lost.wav")) {
    Serial.println("connecttoFS failed");
  } else {
    Serial.println("Playback started");
    // my_audio_info();
  }

}

void loop() {
  // approach 1: play from SD card
  audio.loop();

}
