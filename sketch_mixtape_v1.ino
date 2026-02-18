// TODO:
// - Less fragile manifest identifier function (not from abs path)
#include <ArduinoJson.h>
#include <SPI.h>
#include "SdFat.h"

// --- SD setup (unchanged) ---
#define SD_FAT_TYPE 3

const uint8_t SD_MOSI   = 42;
const uint8_t SD_MISO   = 21;
const uint8_t SD_SCK    = 39;
const uint8_t SD_CS_PIN = 45;

SdFs sd;

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

// --- Tiny JSON extractor (no extra libraries) ---
// Parses entries like: { "name": "...", "file": "..." } inside "tracks":[ ... ]
// This is intentionally minimal: good enough for your controlled manifest format.

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

void setup() {
  Serial.begin(115200);
  while (!Serial) { yield(); }

  Serial.println("Initializing SPI...");
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS_PIN);

  SdSpiConfig sdConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(4), &SPI);

  Serial.println("Initializing SD...");
  if (!sd.begin(sdConfig)) {
    sd.initErrorHalt(&Serial);
  }
  Serial.println("SD init done.");

  // 🔎 Debug: list what the ESP32 actually sees on the SD
  listDir("/", 2);

  Serial.print("Locating manifest: ");
  Serial.println(PLAYLIST_PATH);

  if (!fileExists(PLAYLIST_PATH)) {
    Serial.println("Manifest not found at /demo/playlist-manifest.json");
    Serial.println("Check folder name/case and that it’s on the SD (not just your PC).");
    while (true) delay(1000);
  }

  if (!readFileToBuffer(PLAYLIST_PATH, jsonBuf, sizeof(jsonBuf))) {
    Serial.println("Failed to read manifest");
    while (true) delay(1000);
  }

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

  Serial.println("Ready for playback stage.");
}

void loop() {}
