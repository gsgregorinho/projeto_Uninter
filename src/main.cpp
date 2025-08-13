// main.cpp - Versão completa e corrigida
// Mantive suas configurações de hardware e funcionalidades originais.
// Corrigi coexistência WiFi+Bluetooth, uso de SD e handlers HTTP.

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include "time.h"
#include <SD.h>
#include <FS.h>
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"
#include <cmath>
#include <WebServer.h>

WebServer server(80);

// ====== Configs / pinos ======
#define THERMISTOR_PIN 36
#define SERIES_RESISTOR 10000
#define NOMINAL_RES 10000
#define B_VALUE 3950
#define NOMINAL_TEMP 25

#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST   4
#define TFT_MOSI 23
#define TFT_SCLK 18

#define SD_CS    5

#define ENCODER_CLK 34
#define ENCODER_DT  35
#define ENCODER_SW  32

#define BUZZER_PIN  25
#define PLAY_BUTTON 33

#ifndef TFT_BL
#define TFT_BL -1
#endif

// WiFi / NTP
const char* ssid = "Gabriel";
const char* password = "13092014";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600;
const int daylightOffset_sec = 0;

// ===== Globals =====
TFT_eSPI tft = TFT_eSPI();
ESP32Encoder encoder;
BluetoothSerial SerialBT;
struct tm timeinfo;

float currentTemp = 0.0;
int lastEncoderValue = 0;

String noteText = "Anotacao inicial";
const int noteMaxLen = 80;
bool noteNeedsUpdate = true;
const char* noteFileName = "/Anotacao.txt";

bool sdOk = false; // flag global para SD montado

// State machine UI
enum AppState { HOME_SCREEN, MAIN_MENU, TEXT_MENU, MUSIC_MENU, VIEWING_FILE, PLAYING_MUSIC };
AppState currentState = HOME_SCREEN;

int menuIndex = 0;
String menuFiles[50];
String musicFiles[50];
int menuFileCount = 0;
int musicFileCount = 0;
const int itemsPerPage = 4;

unsigned long lastSecondUpdate = 0;
unsigned long lastMinuteUpdate = 0;
unsigned long lastTempUpdate = 0;
const long encoderDebounce = 50;

int lastMenuIndex = -1;
bool refreshScreen = true;
int currentMusicIndex = -1;
bool isPlaying = false;
bool musicPaused = false;
String currentMelody = "";

// music machine variables
enum MusicState { IDLE, PLAYING_NOTE, BETWEEN_NOTES };
MusicState musicState = IDLE;

String currentNotes = "";
unsigned long noteStartTime = 0;
unsigned long noteDuration = 0;
int currentFrequency = 0;
int defaultDuration = 4;
int defaultOctave = 6;
int bpm = 120;
int wholenote = 0;
bool newNoteReady = true;

// ====== Prototypes ======
String getContentType(String filename);
bool tryServeFromSD(const String& path);
String resolveSDPath(const String& path);

void setupWebServer(); // configura rotas
void handleRoot();
void handleStyle();
void handleScript();
void handleReadFile();
void handleUpdateNote();
void handleListFiles();
void handleUpload();
void handleDelete();
void handleSystemStatus();
void handleFileUploadInternal(); // upload callback

// UI & hardware prototypes
void setupDisplayBlank();
void enableBacklightIfAny();
void setupEncoderPins();
void handleEncoder();
void handlePlayButton();
float readThermistor();
bool initSDCard();
bool connectToWiFi(unsigned long timeoutMs);

// music prototypes
void playTone(int frequency, int duration);
void stopTone();
void parseRTTTL(const String &melody);
void startMelody();
void updateMusicPlayer();
void stopMelody();

// helper filesystem
void listFiles(const char* dirPath, const char* ext, String files[], int &count);

// ====== Implementations ======

// MIME helper
String getContentType(String filename) {
  String s = filename;
  s.toLowerCase();
  if (s.endsWith(".html") || s.endsWith(".htm")) return "text/html";
  if (s.endsWith(".css")) return "text/css";
  if (s.endsWith(".js")) return "application/javascript";
  if (s.endsWith(".json")) return "application/json";
  if (s.endsWith(".png")) return "image/png";
  if (s.endsWith(".jpg") || s.endsWith(".jpeg")) return "image/jpeg";
  if (s.endsWith(".ico")) return "image/x-icon";
  if (s.endsWith(".svg")) return "image/svg+xml";
  return "text/plain";
}

// Resolve possible SD paths (root, /sd, /sdcard)
String resolveSDPath(const String& rawPath) {
  if (!sdOk) return "";
  String path = rawPath;
  if (!path.startsWith("/")) path = "/" + path;

  // Try exact
  if (SD.exists(path)) return path;

  // /sd + path
  String p1 = String("/sd") + path;
  if (SD.exists(p1)) return p1;

  // /sdcard + path
  String p2 = String("/sdcard") + path;
  if (SD.exists(p2)) return p2;

  // try removing leading slash
  String p3 = path.substring(1);
  if (SD.exists(p3)) return p3;

  return "";
}

// try serve from SD (path absolute or relative)
bool tryServeFromSD(const String &path) {
  if (!sdOk) return false;
  String rp = resolveSDPath(path);
  if (rp.length() == 0) return false;
  File f = SD.open(rp, FILE_READ);
  if (!f) return false;
  server.sendHeader("Connection", "close");
  server.streamFile(f, getContentType(rp));
  f.close();
  server.client().stop();
  return true;
}

// ===== Web handlers =====

void handleRoot() {
  if (tryServeFromSD("/www/index.html")) return;
  server.sendHeader("Connection", "close");
  String html = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>ESP32 Notas</title><link rel='stylesheet' href='/style.css'></head><body>"
                "<h1>ESP32 Notas (fallback)</h1><p>Coloque /www/index.html no SD para página customizada.</p>"
                "<script src='/script.js'></script></body></html>";
  server.send(200, "text/html", html);
}

void handleStyle() {
  if (tryServeFromSD("/www/style.css")) return;
  server.sendHeader("Connection", "close");
  server.send(200, "text/css", "body{font-family:sans-serif;padding:12px;}textarea{width:100%;height:120px;}");
}

void handleScript() {
  if (tryServeFromSD("/www/script.js")) return;
  server.sendHeader("Connection", "close");
  server.send(200, "application/javascript", "console.log('fallback script');");
}

void handleReadFile() {
  String path = server.hasArg("path") ? server.arg("path") : "";
  if (path.length() == 0) { server.sendHeader("Connection", "close"); server.send(400, "text/plain", "missing path"); return; }
  if (!sdOk) { server.sendHeader("Connection", "close"); server.send(500, "text/plain", "SD not mounted"); return; }
  String rp = resolveSDPath(path);
  if (rp.length() == 0) { server.sendHeader("Connection", "close"); server.send(404, "text/plain", "not found"); return; }
  File f = SD.open(rp, FILE_READ);
  if (!f) { server.sendHeader("Connection", "close"); server.send(500, "text/plain", "fail open"); return; }
  server.sendHeader("Connection", "close");
  server.streamFile(f, "text/plain");
  f.close();
  server.client().stop();
}

void handleUpdateNote() {
  if (server.hasArg("note")) {
    String n = server.arg("note");
    if (n.length() > noteMaxLen) n = n.substring(0, noteMaxLen);
    noteText = n;
    noteNeedsUpdate = true;
    if (sdOk) { File f = SD.open(noteFileName, FILE_WRITE); if (f) { f.print(noteText); f.close(); } }
    server.sendHeader("Connection", "close"); server.send(200, "text/plain", "Nota salva"); return;
  }
  if (server.hasArg("plain")) {
    String n = server.arg("plain");
    if (n.length() > noteMaxLen) n = n.substring(0, noteMaxLen);
    noteText = n;
    noteNeedsUpdate = true;
    if (sdOk) { File f = SD.open(noteFileName, FILE_WRITE); if (f) { f.print(noteText); f.close(); } }
    server.sendHeader("Connection", "close"); server.send(200, "text/plain", "Nota salva"); return;
  }
  server.sendHeader("Connection", "close"); server.send(400, "text/plain", "missing note");
}

void handleListFiles() {
  // Recebe ?path=/Alguma/Pasta  (aceita "/" ou "/Arquivos" ou "/Arquivos/")
  String path = server.hasArg("path") ? server.arg("path") : "/";
  if (!path.startsWith("/")) path = "/" + path;
  // Remove trailing slash exceto quando for root "/"
  if (path.length() > 1 && path.charAt(path.length() - 1) == '/') path.remove(path.length() - 1);

  if (!sdOk) {
    server.sendHeader("Connection", "close");
    server.send(500, "text/plain", "SD not mounted");
    return;
  }

  // Usa resolveSDPath para aceitar /sd, /sdcard, etc.
  String rp = resolveSDPath(path);
  if (rp.length() == 0) {
    server.sendHeader("Connection", "close");
    server.send(404, "text/plain", "not found");
    return;
  }

  File dir = SD.open(rp);
  if (!dir || !dir.isDirectory()) {
    server.sendHeader("Connection", "close");
    server.send(404, "text/plain", "not dir");
    if (dir) dir.close();
    return;
  }

  String out = "[";
  File entry = dir.openNextFile();
  bool first = true;
  while (entry) {
    if (!first) out += ",";
    String full = String(entry.name());
    String name = full;
    int lastSlash = name.lastIndexOf('/');
    if (lastSlash >= 0) name = name.substring(lastSlash + 1);
    bool isDir = entry.isDirectory();
    unsigned long size = entry.size();
    out += "{\"name\":\"" + name + "\",";
    out += String("\"isDir\":") + (isDir ? "true" : "false") + ",";
    out += String("\"size\":") + String(size);
    out += "}";
    first = false;
    entry = dir.openNextFile();
  }
  out += "]";
  dir.close();
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", out);
}



void handleUpload() {
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain", "Upload handler finished");
}

void handleFileUploadInternal() {
  HTTPUpload& upload = server.upload();
  static File uploadFile;
  if (upload.status == UPLOAD_FILE_START) {
    String filename = upload.filename;
    if (!filename.startsWith("/")) filename = "/" + filename;
    if (!sdOk) { Serial.println("[WEB] Upload but SD not mounted"); return; }
    // try to create in existing path or root
    String rp = resolveSDPath(filename);
    String outPath = (rp.length() > 0) ? rp : filename;
    if (SD.exists(outPath)) SD.remove(outPath);
    uploadFile = SD.open(outPath, FILE_WRITE);
    if (!uploadFile) Serial.println("[WEB] Failed to open upload file: " + outPath);
    else Serial.println("[WEB] Upload start: " + outPath);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (uploadFile) { uploadFile.close(); Serial.println("[WEB] Upload finished: " + upload.filename + " size=" + String(upload.totalSize)); }
  }
}

void handleDelete() {
  if (!server.hasArg("path")) { server.sendHeader("Connection", "close"); server.send(400, "text/plain", "missing path"); return; }
  String path = server.arg("path");
  if (!path.startsWith("/")) path = "/" + path;
  if (!sdOk) { server.sendHeader("Connection", "close"); server.send(500, "text/plain", "SD not mounted"); return; }
  String rp = resolveSDPath(path);
  String target = (rp.length() > 0) ? rp : path;
  if (!SD.exists(target)) { server.sendHeader("Connection", "close"); server.send(404, "text/plain", "not found"); return; }
  bool ok = SD.remove(target);
  server.sendHeader("Connection", "close");
  if (ok) server.send(200, "text/plain", "deleted"); else server.send(500, "text/plain", "fail delete");
}

void handleSystemStatus() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += ",\"temp\":" + String(currentTemp, 1);
  json += "}";
  server.sendHeader("Connection", "close");
  server.send(200, "application/json", json);
}

// ========= Setup web server =========
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/style.css", HTTP_GET, handleStyle);
  server.on("/script.js", HTTP_GET, handleScript);
  server.on("/read-file", HTTP_GET, handleReadFile);
  server.on("/update-note", HTTP_POST, handleUpdateNote);
  server.on("/list-files", HTTP_GET, handleListFiles);
  server.on("/upload", HTTP_POST, handleUpload, handleFileUploadInternal);
  server.on("/delete", HTTP_POST, handleDelete);
  server.on("/system-status", HTTP_GET, handleSystemStatus);
  server.begin();
  Serial.println("HTTP server started");
}

// ====== Note SD helpers ======
void saveNoteToSD(const String& text) {
  if (!sdOk) return;
  File f = SD.open(noteFileName, FILE_WRITE);
  if (f) { f.print(text); f.close(); }
}

void loadNoteFromSD() {
  if (!sdOk) return;
  if (SD.exists(noteFileName)) {
    File f = SD.open(noteFileName, FILE_READ);
    if (f) {
      noteText = f.readString();
      noteText.trim();
      if (noteText.length() > noteMaxLen) noteText = noteText.substring(0, noteMaxLen);
      f.close();
    }
  }
}

// ====== Music functions ======
void playTone(int frequency, int duration) {
  if (frequency <= 0) return;
  ledcSetup(0, frequency, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWrite(0, 128);
}

void stopTone() {
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

void parseRTTTL(const String &melody) {
  defaultDuration = 4;
  defaultOctave = 6;
  bpm = 120;

  int startIndex = melody.indexOf(':');
  if (startIndex == -1) return;

  String settings = melody.substring(0, startIndex);
  settings.toLowerCase();

  int bpmIndex = settings.indexOf("b=");
  if (bpmIndex != -1) {
    int nextComma = settings.indexOf(',', bpmIndex);
    if (nextComma == -1) nextComma = settings.length();
    bpm = settings.substring(bpmIndex + 2, nextComma).toInt();
  }

  int durIndex = settings.indexOf("d=");
  if (durIndex != -1) {
    int nextComma = settings.indexOf(',', durIndex);
    if (nextComma == -1) nextComma = settings.length();
    defaultDuration = settings.substring(durIndex + 2, nextComma).toInt();
  }

  int octIndex = settings.indexOf("o=");
  if (octIndex != -1) {
    int nextComma = settings.indexOf(',', octIndex);
    if (nextComma == -1) nextComma = settings.length();
    defaultOctave = settings.substring(octIndex + 2, nextComma).toInt();
  }

  if (bpm <= 0) bpm = 120;
  wholenote = (60000 * 4) / bpm;

  currentNotes = melody.substring(startIndex + 1);
  currentNotes.replace(" ", "");
  currentNotes.replace("\r", "");
  currentNotes.replace("\n", "");
}

void startMelody() {
  if (currentMelody.length() == 0) {
    if (currentMusicIndex < 0 || currentMusicIndex >= musicFileCount) {
      isPlaying = false;
      return;
    }
    File file = SD.open(String("/Musicas/") + musicFiles[currentMusicIndex]);
    if (file) {
      currentMelody = file.readString();
      file.close();
      parseRTTTL(currentMelody);
      musicState = IDLE;
      newNoteReady = true;
    } else {
      isPlaying = false;
    }
  }
}

void updateMusicPlayer() {
  unsigned long currentTime = millis();
  switch (musicState) {
    case IDLE:
      if (newNoteReady) {
        if (currentNotes.length() == 0) { stopMelody(); return; }

        int duration = 0;
        int note = 0;
        int octave = defaultOctave;
        bool hasDot = false;

        int idx = 0;
        if (idx < (int)currentNotes.length() && isDigit(currentNotes.charAt(idx))) {
          duration = currentNotes.substring(idx, idx+1).toInt();
          idx++;
          if (idx < (int)currentNotes.length() && isDigit(currentNotes.charAt(idx))) {
            duration = duration * 10 + currentNotes.substring(idx, idx+1).toInt();
            idx++;
          }
        } else {
          duration = defaultDuration;
        }

        if (idx < (int)currentNotes.length() && currentNotes.charAt(idx) == '.') { hasDot = true; idx++; }
        if (idx >= (int)currentNotes.length()) { stopMelody(); return; }

        char noteChar = currentNotes.charAt(idx++);
        switch (noteChar) {
          case 'c': note = 0; break;
          case 'd': note = 2; break;
          case 'e': note = 4; break;
          case 'f': note = 5; break;
          case 'g': note = 7; break;
          case 'a': note = 9; break;
          case 'b': note = 11; break;
          case 'p': note = -1; break;
          default: note = -1; break;
        }

        if (idx < (int)currentNotes.length() && currentNotes.charAt(idx) == '#') { note++; idx++; }
        if (idx < (int)currentNotes.length() && isDigit(currentNotes.charAt(idx))) { octave = currentNotes.substring(idx, idx+1).toInt(); idx++; }

        currentNotes = currentNotes.substring(idx);

        noteDuration = wholenote / max(1, duration);
        if (hasDot) noteDuration += noteDuration / 2;

        if (note >= 0) {
          currentFrequency = (int) round(440.0 * pow(1.0594630943592953, note + (octave - 4) * 12));
        } else {
          currentFrequency = 0;
        }

        newNoteReady = false;
        musicState = PLAYING_NOTE;
        noteStartTime = currentTime;

        if (currentFrequency > 0) playTone(currentFrequency, noteDuration);
        else stopTone();
      }
      break;

    case PLAYING_NOTE:
      if (currentTime - noteStartTime >= noteDuration) {
        stopTone();
        noteStartTime = currentTime;
        musicState = BETWEEN_NOTES;
      }
      break;

    case BETWEEN_NOTES:
      if (currentTime - noteStartTime >= (noteDuration / 10)) {
        newNoteReady = true;
        musicState = IDLE;
      }
      break;
  }
}

void stopMelody() {
  isPlaying = false;
  musicPaused = false;
  currentMelody = "";
  currentNotes = "";
  musicState = IDLE;
  newNoteReady = true;
  stopTone();
  currentState = MUSIC_MENU;
  menuIndex = 0;
  refreshScreen = true;
}

// ====== UI / periféricos ======

void setupDisplayBlank() {
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  delay(10);
  tft.init();
  tft.setRotation(1);
  tft.setTextWrap(false);
  tft.fillScreen(TFT_BLACK); // tela preta

  if (TFT_BL >= 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);
  }

  int squareSize = 80;
  int x = (tft.width() - squareSize) / 2;
  int y = (tft.height() - squareSize) / 2;
  tft.fillRect(x, y, squareSize, squareSize, TFT_WHITE);

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(6);
  tft.drawString("G", tft.width() / 2, tft.height() / 2);
}

void enableBacklightIfAny() {
  if (TFT_BL >= 0) digitalWrite(TFT_BL, HIGH);
}

void setupEncoderPins() {
  encoder.attachHalfQuad(ENCODER_CLK, ENCODER_DT);
  encoder.clearCount();
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
}

float readThermistor() {
  int adc = analogRead(THERMISTOR_PIN);
  if (adc == 0) return currentTemp;
  float resistance = SERIES_RESISTOR / ((4095.0 / adc) - 1.0);
  float steinhart = log(resistance / NOMINAL_RES) / B_VALUE;
  steinhart += 1.0 / (NOMINAL_TEMP + 273.15);
  return 1.0 / steinhart - 273.15;
}

bool initSDCard() {
  if (!SD.begin(SD_CS)) return false;
  return SD.cardType() != CARD_NONE;
}

bool connectToWiFi(unsigned long timeoutMs) {
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  // IMPORTANT: when using Bluetooth Classic (BluetoothSerial), WiFi modem sleep must be enabled
  WiFi.setSleep(true);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  Serial.printf("[WIFI] Conectando a %s\n", ssid);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.println("\n[WIFI] Timeout");
      return false;
    }
    Serial.print(".");
    delay(500);
    server.handleClient(); // keep webserver responsive while waiting
  }

  Serial.println("\n[WIFI] Conectado com sucesso");
  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

// UI helpers
void drawStaticScreen() {
  tft.fillScreen(TFT_NAVY);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_CYAN);
  uint16_t grayBlue = tft.color565(80, 100, 110);
  tft.fillRoundRect(0, 0, tft.width(), 30, 4, grayBlue);
  tft.fillRoundRect(60, 180, 120, 40, 8, TFT_DARKCYAN);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(90, 190);
  tft.print("MENU");
}

void updateDateText() {
  if (!getLocalTime(&timeinfo)) return;
  char dateStr[20];
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", &timeinfo);
  uint16_t grayBlue = tft.color565(80, 100, 110);
  tft.fillRoundRect(0, 0, 240, 30, 4, grayBlue);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, grayBlue);
  tft.setTextSize(1);
  tft.drawString(dateStr, 5, 15);
}

void updateTimeText() {
  if (!getLocalTime(&timeinfo)) return;
  int h = timeinfo.tm_hour;
  int m = timeinfo.tm_min;
  int s = timeinfo.tm_sec;
  uint16_t grayBlue = tft.color565(80, 100, 110);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, grayBlue);
  tft.setTextSize(1);
  char timeStr[6];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d", h, m);
  tft.fillRoundRect(100, 0, 50, 30, 4, grayBlue);
  tft.drawString(timeStr, 105, 15);
  char secStr[4];
  snprintf(secStr, sizeof(secStr), ":%02d", s);
  tft.fillRoundRect(150, 0, 30, 30, 4, grayBlue);
  tft.drawString(secStr, 135, 15);
}

void updateTempText() {
  currentTemp = readThermistor();
  uint16_t grayBlue = tft.color565(80, 100, 110);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, grayBlue);
  tft.setTextSize(1);
  tft.fillRoundRect(180, 0, 60, 30, 4, grayBlue);
  char tempStr[10];
  snprintf(tempStr, sizeof(tempStr), "%.1fC", currentTemp);
  tft.drawString(tempStr, 210, 15);
}

void updateNoteText() {
  uint16_t bgColor = TFT_NAVY;
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, bgColor);
  tft.setTextSize(1);
  tft.fillRect(10, 140, 220, 30, bgColor);
  tft.drawString(noteText, tft.width() / 2, 155);
  noteNeedsUpdate = false;
}

void listFiles(const char* dirPath, const char* ext, String files[], int &count) {
  if (!sdOk) { count = 0; return; }
  String rp = resolveSDPath(dirPath);
  if (rp.length() == 0) { count = 0; return; }
  File root = SD.open(rp);
  count = 0;
  if (!root) return;
  File file = root.openNextFile();
  while (file && count < 50) {
    if (!file.isDirectory()) {
      String name = file.name();
      if (ext == nullptr || ext[0] == '\0' || name.endsWith(ext)) files[count++] = name.substring(name.lastIndexOf('/')+1);
    }
    file = root.openNextFile();
  }
  root.close();
}

// menu generic
void drawPagedMenu(const char* title, String items[], int totalItems, int selectedIndex, bool hasBack) {
  if (!refreshScreen && selectedIndex == lastMenuIndex) return;
  tft.fillScreen(TFT_NAVY);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(10, 10);
  tft.println(title);

  int effectiveTotal = totalItems + (hasBack ? 1 : 0);
  int page = selectedIndex / itemsPerPage;
  int start = page * itemsPerPage;
  int end = min(start + itemsPerPage, effectiveTotal);

  for (int i = start; i < end; i++) {
    int y = 40 + (i - start) * 40;
    uint16_t bgColor = (i == selectedIndex) ? TFT_DARKCYAN : TFT_BLUE;
    tft.fillRoundRect(10, y, 220, 30, 4, bgColor);
    tft.setTextColor(TFT_WHITE);
    String textToPrint = (hasBack && i == 0) ? "<< Voltar" : items[i - (hasBack ? 1 : 0)];
    tft.setCursor(20, y + 7);
    tft.print(textToPrint);
  }

  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, 210);
  tft.printf("Pagina %d/%d", page + 1, (effectiveTotal + itemsPerPage - 1) / itemsPerPage);

  lastMenuIndex = selectedIndex;
  refreshScreen = false;
}

void showFileContent(const String& filename) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(1);

  File file = SD.open(String("/Arquivos/") + filename);
  if (!file) {
    tft.println("Erro ao abrir arquivo.");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    tft.println(line);
  }
  file.close();

  tft.setCursor(0, 230);
  tft.setTextColor(TFT_CYAN);
  tft.print(">Pressione para voltar");
}

void showMusicPlayer() {
  if (!refreshScreen) return;
  tft.fillScreen(TFT_DARKGREEN);
  tft.drawRect(0, 0, tft.width(), tft.height(), TFT_CYAN);

  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(10, 10);
  tft.print("Tocando:");

  tft.setTextColor(TFT_WHITE);
  tft.setCursor(10, 40);
  if (currentMusicIndex >=0 && currentMusicIndex < musicFileCount) tft.print(musicFiles[currentMusicIndex]);

  for (int i = 0; i < 2; i++) {
    int y = 80 + i * 60;
    uint16_t bgColor = (i == menuIndex) ? TFT_DARKCYAN : (i == 0 ? TFT_BLUE : TFT_RED);
    tft.fillRoundRect(20, y, 200, 40, 8, bgColor);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    if (i == 0) {
      tft.setCursor(musicPaused ? 70 : 80, y + 13);
      tft.print(musicPaused ? "CONTINUAR" : "PAUSAR");
    } else {
      tft.setCursor(80, y + 13);
      tft.print("VOLTAR");
    }
  }

  refreshScreen = false;
}

void handleEncoder() {
  static unsigned long lastMillis = 0;
  int currentValue = encoder.getCount();
  int delta = currentValue - lastEncoderValue;

  if (millis() - lastMillis >= encoderDebounce) {
    if (abs(delta) >= 2) {
      int totalItems = 0;
      switch (currentState) {
        case MAIN_MENU: totalItems = 3; break;
        case TEXT_MENU: totalItems = menuFileCount + 1; break;
        case MUSIC_MENU: totalItems = musicFileCount + 1; break;
        case PLAYING_MUSIC: totalItems = 2; break;
        default: totalItems = 0; break;
      }

      if (currentState != HOME_SCREEN && totalItems > 0) {
        int step = (delta > 0) ? 1 : -1;
        menuIndex = (menuIndex + step + totalItems) % totalItems;
        refreshScreen = true;
      }

      lastEncoderValue = currentValue;
      lastMillis = millis();
    }
  }

  if (digitalRead(ENCODER_SW) == LOW) {
    delay(50);
    if (digitalRead(ENCODER_SW) == LOW) {
      while (digitalRead(ENCODER_SW) == LOW) { delay(1); }
      switch (currentState) {
        case HOME_SCREEN:
          currentState = MAIN_MENU;
          menuIndex = 0;
          refreshScreen = true;
          break;
        case MAIN_MENU:
          if (menuIndex == 0) {
            listFiles("/Arquivos", ".txt", menuFiles, menuFileCount);
            currentState = TEXT_MENU;
          } else if (menuIndex == 1) {
            listFiles("/Musicas", ".txt", musicFiles, musicFileCount);
            currentState = MUSIC_MENU;
          } else {
            currentState = HOME_SCREEN;
            drawStaticScreen();
          }
          menuIndex = 0;
          refreshScreen = true;
          break;
        case TEXT_MENU:
          if (menuIndex == 0) {
            currentState = MAIN_MENU;
          } else {
            showFileContent(menuFiles[menuIndex - 1]);
            currentState = VIEWING_FILE;
          }
          refreshScreen = true;
          break;
        case MUSIC_MENU:
          if (menuIndex == 0) {
            currentState = MAIN_MENU;
          } else {
            currentMusicIndex = menuIndex - 1;
            isPlaying = true;
            musicPaused = false;
            currentState = PLAYING_MUSIC;
            menuIndex = 0;
            startMelody();
          }
          refreshScreen = true;
          break;
        case VIEWING_FILE:
          currentState = TEXT_MENU;
          refreshScreen = true;
          break;
        case PLAYING_MUSIC:
          if (menuIndex == 0) {
            musicPaused = !musicPaused;
            if (musicPaused) stopTone();
            else { musicState = IDLE; newNoteReady = true; }
            refreshScreen = true;
          } else if (menuIndex == 1) {
            stopMelody();
          }
          break;
      }
    }
  }

  // Update UI screen based on state
  switch (currentState) {
    case MAIN_MENU:
      if (refreshScreen) {
        String main[] = {"Arquivos de Texto", "Lista de Musicas", "Sair"};
        drawPagedMenu("Menu Principal:", main, 3, menuIndex, false);
      }
      break;
    case TEXT_MENU:
      if (refreshScreen) drawPagedMenu("Arquivos .txt:", menuFiles, menuFileCount, menuIndex, true);
      break;
    case MUSIC_MENU:
      if (refreshScreen) drawPagedMenu("Lista de Musicas:", musicFiles, musicFileCount, menuIndex, true);
      break;
    case PLAYING_MUSIC:
      if (refreshScreen) showMusicPlayer();
      break;
    default: break;
  }
}

void handlePlayButton() {
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;

  int buttonState = digitalRead(PLAY_BUTTON);
  if (buttonState != lastButtonState) lastDebounceTime = millis();
  if ((millis() - lastDebounceTime) > 50) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      if (currentState == PLAYING_MUSIC) {
        musicPaused = !musicPaused;
        if (musicPaused) stopTone();
        else { musicState = IDLE; newNoteReady = true; }
        refreshScreen = true;
      }
    }
  }
  lastButtonState = buttonState;
}

// ===== setup / loop =====
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Notas");

  setupDisplayBlank();
  setupEncoderPins();
  lastEncoderValue = encoder.getCount();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PLAY_BUTTON, INPUT_PULLUP);

  sdOk = initSDCard();
  if (sdOk) {
    loadNoteFromSD();
    if (!SD.exists("/www")) SD.mkdir("/www");
    if (!SD.exists("/Arquivos")) SD.mkdir("/Arquivos");
    if (!SD.exists("/Musicas")) SD.mkdir("/Musicas");
    Serial.println("[SD] inicializado");
  } else {
    Serial.println("[SD] não inicializado");
  }

  bool wifiOk = connectToWiFi(60000); // 60s timeout
  if (wifiOk) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  } else {
    Serial.println("[WIFI] falha conectar");
  }

  setupWebServer();

  enableBacklightIfAny();
  drawStaticScreen();
  updateDateText();
  updateTimeText();
  updateTempText();
  updateNoteText();

  lastSecondUpdate = millis();
  lastMinuteUpdate = millis();
  lastTempUpdate = millis();
}

void loop() {
  unsigned long now = millis();

  // serve clients promptly to avoid socket pileup
  server.handleClient();

  if (SerialBT.available()) {
    String incoming = SerialBT.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > noteMaxLen) incoming = incoming.substring(0, noteMaxLen);
    noteText = incoming;
    noteNeedsUpdate = true;
    saveNoteToSD(noteText);
  }

  handleEncoder();
  handlePlayButton();

  if (currentState == PLAYING_MUSIC && isPlaying && !musicPaused) updateMusicPlayer();

  if (currentState == HOME_SCREEN) {
    if (now - lastSecondUpdate >= 1000) { updateTimeText(); lastSecondUpdate = now; }
    if (now - lastMinuteUpdate >= 60000) { updateDateText(); lastMinuteUpdate = now; }
    if (now - lastTempUpdate >= 60000) { updateTempText(); lastTempUpdate = now; }
    if (noteNeedsUpdate) updateNoteText();
  }

  delay(1);
}
