#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include <Audio.h>
#include <esp32-hal-psram.h>
#include <driver/i2s.h>
#include <math.h>
#include <mutex>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "kiss_fft.h"
#include <Preferences.h> 

#define SD_CS 10
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

#define DIN 38
#define BCLK 39
#define LRC 40

#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_16
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_15
#define I2S_MIC_SERIAL_DATA GPIO_NUM_17
#define I2S_PORT I2S_NUM_1

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 1024
#define DURATION_SEC 12
#define READ_SIZE 1024
#define FFT_SIZE 1024

// Định nghĩa chân I2C cho LCD
#define I2C_SDA 4
#define I2C_SCL 5

// Định nghĩa chân nút nhấn
#define BUTTON_PLAY_PIN 42     // Nút phát âm thanh
#define BUTTON_DELETE_PIN 7    // Nút xóa file
#define BUTTON_RESET_PIN 9     // Nút reset hệ thống

// Biến debounce
bool isPlaying = false;

unsigned long lastDebounceTime[3] = {0, 0, 0};
uint8_t lastButtonState[3] = {HIGH, HIGH, HIGH};
const unsigned long debounceDelay = 50;
enum PlayState {
    IDLE,
    WAITING_TO_PLAY,
    PLAYING
};

PlayState playState = IDLE;
unsigned long playStartTime = 0;
// Khởi tạo LCD I2C (địa chỉ 0x27, 20 ký tự x 4 dòng)
LiquidCrystal_I2C lcd(0x27, 20, 4);
Preferences preferences; // Khai báo đối tượng Preferences
std::mutex fileMutex;
Audio audio;
String fileList[100];
int fileCount = 0;
int currentFileIndex = 0;

String customFolder = "ghiam";
uint32_t customRecordDuration = 0;
String lastRecordedFile = "";
int32_t *buffer32 = NULL;
int16_t *buffer16 = NULL;
bool isRecording = false;
unsigned long recordTriggerTime = 0;
bool hasRecordedForCurrentPlayback = false;
uint32_t fileCounter = 0;

void loadFileList() {
    File root = SD.open("/");
    fileCount = 0;
    Serial.println("[SD] Danh sách file:");
    
    while (File entry = root.openNextFile()) {
        if (!entry.isDirectory() && fileCount < 100) {
            String fileName = entry.name();
            if (fileName.endsWith(".mp3") || fileName.endsWith(".wav")) {
                fileList[fileCount] = fileName;
                Serial.printf("[%d] %s\n", fileCount + 1, fileName.c_str());
                fileCount++;
            }
        }
        entry.close();
    }
    root.close();
}

uint32_t getNextFileCounter() {
    uint32_t maxCounter = 0;
    uint32_t prefCounter = preferences.getUInt("fileCounter", 0);
    File root = SD.open("/" + customFolder);
    if (!root || !root.isDirectory()) {
        Serial.println("[SD] ❌ Không tìm thấy thư mục /ghiam");
        return (prefCounter > 0) ? prefCounter : 1;
    }
    
    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        if (filename.startsWith("audio_") && filename.endsWith(".wav")) {
            String numStr = filename.substring(6, filename.length() - 4);
            uint32_t num = numStr.toInt();
            if (num > maxCounter) maxCounter = num;
        }
        file = root.openNextFile();
    }
    root.close();
    
    return (maxCounter > prefCounter) ? maxCounter + 1 : prefCounter;
}

void playFile(int index) {
    if (index >= fileCount || index < 0) {
        Serial.println("[Audio] ⚠️ Index file không hợp lệ!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Invalid index");
        return;
    }

    currentFileIndex = index;
    String path = "/" + fileList[index];
    Serial.println("[Audio] 🔍 Đang chuẩn bị phát file: " + path);
    
    if (!SD.exists(path)) {
        Serial.println("[SD] ❌ File không tồn tại trên thẻ SD: " + path);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("File not found");
        return;
    }
    
    Serial.println("[Audio] ✅ File tồn tại, bắt đầu kết nối...");
    Serial.printf("Kết nối audio với: %s\n", path.c_str());
    Serial.printf("Chân DIN: %d, BCLK: %d, LRC: %d\n", DIN, BCLK, LRC);
    
    audio.stopSong();
    
    if (!audio.connecttoFS(SD, path.c_str())) {
        Serial.println("[Audio] ❌ LỖI: Không thể phát file!");
        Serial.println("Nguyên nhân có thể do:");
        Serial.println("1. Sai định dạng file (chỉ hỗ trợ MP3/WAV)");
        Serial.println("2. Lỗi thư viện Audio");
        Serial.println("3. Kết nối loa không chính xác");
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Playback error");
        return;
    }
    
    Serial.println("[Audio] 🎵 Đã kết nối file thành công! Đang phát...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Dang Phat File :");
    lcd.setCursor(0, 1);
    lcd.print(fileList[index].substring(0, 15));
    
    playState = WAITING_TO_PLAY;
    playStartTime = millis();
    Serial.println("[Audio] ⏳ Lên lịch phát sau 1 giây...");
    recordTriggerTime = millis() + 900;
    hasRecordedForCurrentPlayback = false;
    Serial.println("[Audio] ⏳ Lên lịch bắt đầu ghi âm sau 1 giây...");
}

void TaskAudio(void * parameter) {
    Serial.println("Audio Task bắt đầu");
    while (1) {
        audio.loop();
        vTaskDelay(1);
    }
}

void recordAudio(const char *filename, uint32_t duration) {
    Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());

    String filenameStr = String(filename);
    if (filenameStr.startsWith("/")) {
        filenameStr = filenameStr.substring(1);
    }
    String fullPath = "/" + customFolder + "/" + filenameStr;
    File file = SD.open(fullPath, FILE_WRITE);
    if (!file) {
        Serial.println("[SD] ❌ Không thể mở file để ghi: " + fullPath);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(fileMutex);
        lastRecordedFile = fullPath;
    }
    
    Serial.println("[Record] 📂 Đường dẫn ghi âm: " + fullPath);
    writeWAVHeader(file, SAMPLE_RATE, 1, 16);

    const uint32_t targetSamples = SAMPLE_RATE * duration;
    uint32_t samplesWritten = 0;
    const size_t WRITE_BUFFER_SIZE = 8192;
    static uint8_t writeBuffer[WRITE_BUFFER_SIZE];
    static size_t writeBufferPos = 0;

    float b[] = {0.0660287f, 0.0f, -0.0660287f};
    float a[] = {1.0f, -1.8310493f, 0.8679426f};
    float state[2] = {0.0f, 0.0f};

    while (samplesWritten < targetSamples && isRecording) {
        size_t bytesRead = 0;
        ESP_ERROR_CHECK(i2s_read(I2S_NUM_1, buffer32, BUFFER_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY));

        uint16_t samplesRead = bytesRead / sizeof(int32_t);
        for (int i = 0; i < samplesRead; i++) {
            float sample = (buffer32[i] >> 16) / 16384.0f;
            sample = fmax(fmin(sample, 1.0f), -1.0f);
            
            float v = sample - a[1] * state[0] - a[2] * state[1];
            float filteredSample = b[0] * v + b[1] * state[0] + b[2] * state[1];
            state[1] = state[0];
            state[0] = v;
            filteredSample = fmaxf(fminf(filteredSample, 1.0f), -1.0f);
            buffer16[i] = filteredSample * 32767.0f;
        }

        file.write((uint8_t*)buffer16, samplesRead * sizeof(int16_t));
        samplesWritten += samplesRead;

        Serial.printf("Progress: %.1f%%\r", (samplesWritten * 100.0) / targetSamples);
    }
    updateWAVHeader(file);
    file.close();
    delay(100);
    isRecording = false;
    Serial.println("\n[Audio] ✅ Ghi âm hoàn tất!");
    processFFT(fullPath.c_str());
}

void TaskRecord(void * parameter) {
    while (1) {
        if (recordTriggerTime != 0 && audio.isRunning() && millis() >= recordTriggerTime && !isRecording && !hasRecordedForCurrentPlayback) {
            isRecording = true;
            hasRecordedForCurrentPlayback = true;
            recordTriggerTime = 0;
            Serial.println("[Audio] 🎤 Bắt đầu ghi âm (kích hoạt sau 1 giây phát)...");
        }

        if (isRecording) {
            String filename = "audio_" + String(fileCounter) + ".wav";
            Serial.println("Recording to: " + filename);
            uint32_t duration = (customRecordDuration > 0) ? customRecordDuration : DURATION_SEC;
            Serial.println("[Audio] ⏱ Thời gian ghi âm: " + String(duration) + " giây");
            recordAudio(filename.c_str(), duration);
            preferences.putUInt("fileCounter", fileCounter + 1);
            fileCounter++;
            customRecordDuration = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void writeWAVHeader(File &file, uint32_t sampleRate, uint16_t channels, uint16_t bitsPerSample) {
    uint32_t byteRate = sampleRate * channels * (bitsPerSample / 8);
    uint16_t blockAlign = channels * (bitsPerSample / 8);
    file.write((const uint8_t *)"RIFF", 4);
    uint32_t chunkSize = 0;
    file.write((const uint8_t *)&chunkSize, 4);
    file.write((const uint8_t *)"WAVE", 4);
    file.write((const uint8_t *)"fmt ", 4);
    uint32_t subChunk1Size = 16;
    file.write((const uint8_t *)&subChunk1Size, 4);
    uint16_t audioFormat = 1;
    file.write((const uint8_t *)&audioFormat, 2);
    file.write((const uint8_t *)&channels, 2);
    file.write((const uint8_t *)&sampleRate, 4);
    file.write((const uint8_t *)&byteRate, 4);
    file.write((const uint8_t *)&blockAlign, 2);
    file.write((const uint8_t *)&bitsPerSample, 2);
    file.write((const uint8_t *)"data", 4);
    uint32_t subChunk2Size = 0;
    file.write((const uint8_t *)&subChunk2Size, 4);
}

void updateWAVHeader(File &file) {
    uint32_t fileSize = file.size();
    uint32_t dataSize = fileSize - 44;

    file.seek(4);
    uint32_t chunkSize = fileSize - 8;
    file.write((const uint8_t *)&chunkSize, 4);

    file.seek(40);
    file.write((const uint8_t *)&dataSize, 4);
}

void processFFT(const char* filename) {
    File file = SD.open(filename, FILE_READ);
    if (!file) {
        Serial.println("[SD] ❌ Không thể mở file: " + String(filename));
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD Error");
        return;
    }

    uint8_t header[44];
    if (file.read(header, 44) != 44) {
        Serial.println("[SD] ❌ File WAV không hợp lệ (header quá ngắn)");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Invalid WAV");
        file.close();
        return;
    }

    uint32_t sampleRate = *(uint32_t*)&header[24];
    uint16_t bitsPerSample = *(uint16_t*)&header[34];
    uint16_t channels = *(uint16_t*)&header[22];
    if (sampleRate != SAMPLE_RATE || bitsPerSample != 16 || channels != 1) {
        Serial.println("[SD] ❌ File WAV không được hỗ trợ (yêu cầu mono, 16-bit, " + String(SAMPLE_RATE) + " Hz)");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Unsupported WAV");
        file.close();
        return;
    }

    int16_t* buffer = (int16_t*)malloc(READ_SIZE * sizeof(int16_t));
    kiss_fft_cpx* fftInput = (kiss_fft_cpx*)malloc(FFT_SIZE * sizeof(kiss_fft_cpx));
    kiss_fft_cpx* fftOutput = (kiss_fft_cpx*)malloc(FFT_SIZE * sizeof(kiss_fft_cpx));
    kiss_fft_cfg cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    
    if (!buffer || !fftInput || !fftOutput || !cfg) {
        Serial.println("[FFT] ❌ Lỗi cấp phát bộ nhớ");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Memory Error");
        if (buffer) free(buffer);
        if (fftInput) free(fftInput);
        if (fftOutput) free(fftOutput);
        if (cfg) free(cfg);
        file.close();
        return;
    }

    float* hannWindow = (float*)malloc(READ_SIZE * sizeof(float));
    if (!hannWindow) {
        Serial.println("[FFT] ❌ Lỗi cấp phát bộ nhớ cho hannWindow");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Memory Error");
        free(buffer);
        free(fftInput);
        free(fftOutput);
        free(cfg);
        file.close();
        return;
    }
    float windowSum = 0.0f;
    for (int i = 0; i < READ_SIZE; i++) {
        hannWindow[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (READ_SIZE - 1)));
        windowSum += hannWindow[i];
    }
    const float windowFactor = 1.0f / windowSum;

    Serial.println("[FFT] Đang xử lý file: " + String(filename));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Phan Tich FFT...");

    float maxMagnitudeDB = -200.0f;
    float maxFrequency = 0.0f;
    int maxBin = 0;
    float leftBinMag = 0, centerBinMag = 0, rightBinMag = 0;

    while (file.available()) {
        uint16_t samplesRead = file.read((uint8_t*)buffer, READ_SIZE * sizeof(int16_t)) / sizeof(int16_t);
        if (samplesRead < READ_SIZE) break;

        for (int i = 0; i < samplesRead; i++) {
            fftInput[i].r = (buffer[i] / 32768.0f) * hannWindow[i] * windowFactor;
            fftInput[i].i = 0.0f;
        }

        for (int i = samplesRead; i < FFT_SIZE; i++) {
            fftInput[i].r = 0.0f;
            fftInput[i].i = 0.0f;
        }

        kiss_fft(cfg, fftInput, fftOutput);

        for (int i = 1; i < FFT_SIZE / 2 - 1; i++) {
            float real = fftOutput[i].r;
            float imag = fftOutput[i].i;
            float mag = sqrtf(real * real + imag * imag) * 2.0f;
            float magDB = 20.0f * log10f(mag);
            
            if (magDB > maxMagnitudeDB) {
                maxMagnitudeDB = magDB;
                maxBin = i;
                real = fftOutput[i-1].r;
                imag = fftOutput[i-1].i;
                leftBinMag = sqrtf(real * real + imag * imag) * 2.0f;
                centerBinMag = mag;
                real = fftOutput[i+1].r;
                imag = fftOutput[i+1].i;
                rightBinMag = sqrtf(real * real + imag * imag) * 2.0f;
            }
        }
    }

    float peakBin = (float)maxBin;
    if (maxBin > 0 && maxBin < FFT_SIZE / 2 - 1) {
        float delta = 0.5f * (leftBinMag - rightBinMag) / (leftBinMag - 2.0f * centerBinMag + rightBinMag);
        peakBin += delta;

        float a = (leftBinMag - 2.0f * centerBinMag + rightBinMag) / 2.0f;
        float b = (rightBinMag - leftBinMag) / 2.0f;
        float y_max;

        if (a < 0) {
            y_max = centerBinMag - (b * b) / (4.0f * a);
        } else {
            y_max = centerBinMag;
        }
        maxMagnitudeDB = 20.0f * log10f(y_max);
    }
    
    maxFrequency = peakBin * (SAMPLE_RATE / (float)FFT_SIZE);

    String coconutType;
    if (maxMagnitudeDB <= -15.0f && maxMagnitudeDB > -31.65f) {
        coconutType = "Dua Khong Sap";
    } 
    else if (maxMagnitudeDB <= -31.65f && maxMagnitudeDB > -55.0f) {
        coconutType = "Dua Sap";
    }
    else {
        coconutType = "Khong Nhan Biet";
    }
    
    Serial.printf("[FFT] Magnitude lớn nhất: %.1f Hz, %.2f dB\n", maxFrequency, maxMagnitudeDB);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TanSo:");
    lcd.setCursor(7, 0);
    lcd.print(maxFrequency, 1);
    lcd.print("Hz");
    
    lcd.setCursor(0, 1);
    lcd.print("BienDo:");
    lcd.setCursor(7, 1);
    lcd.print(maxMagnitudeDB, 1);
    lcd.print("dB");
    
    lcd.setCursor(0, 2);
    lcd.print("");

    free(buffer);
    free(fftInput);
    free(fftOutput);
    free(cfg);
    free(hannWindow);
    file.close();
    Serial.println("[FFT] Hoàn tất xử lý file!");
}

void setupI2SMic() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 10,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK,
        .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SERIAL_DATA
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1, &pin_config));
}

void deleteAllAudioFiles() {
    File root = SD.open("/" + customFolder);
    if (!root || !root.isDirectory()) {
        Serial.println("[SD] ❌ Không tìm thấy thư mục " + customFolder + "!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Folder");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        String filename = file.name();
        Serial.print("[SD] 🗑️ Xóa: ");
        Serial.println(filename);
        SD.remove("/" + customFolder + "/" + filename);
        file = root.openNextFile();
    }
    root.close();

    Serial.println("[SD] ✅ Tất cả các file trong thư mục " + customFolder + " đã bị xóa!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Files Deleted");
}

void deleteLastRecordedFile() {
    std::lock_guard<std::mutex> lock(fileMutex);
    
    if (lastRecordedFile.length() == 0) {
        Serial.println("[BUTTON] ⚠️ Chưa có file nào được ghi!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Khong Co File");
        return;
    }

    Serial.println("[BUTTON] 🔍 Kiểm tra file: " + lastRecordedFile);
    
    if (SD.exists(lastRecordedFile)) {
        if (SD.remove(lastRecordedFile)) {
            Serial.println("[BUTTON] ✅ Xóa thành công: " + lastRecordedFile);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Xoa File Vua Ghi Am:");
            lcd.setCursor(0, 1);
            lcd.print(lastRecordedFile.substring(0, 15));
            lastRecordedFile = "";
        } else {
            Serial.println("[BUTTON] ❌ Lỗi khi xóa!");
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("Xoa That Bai");
        }
    } else {
        Serial.println("[BUTTON] ⚠️ File không tồn tại!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("File not found");
        lastRecordedFile = "";
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(BUTTON_PLAY_PIN, INPUT_PULLUP);
    pinMode(BUTTON_DELETE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RESET_PIN, INPUT_PULLUP);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("LVTN 2025 De Tai:");
    lcd.setCursor(0, 1);
    lcd.print("Phan Loai Dua Sap");
    lcd.setCursor(0, 2);
    lcd.print("Phuong Phap Am Thanh");
    delay(2000);
    
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
    
    preferences.begin("storage", false);
    fileCounter = getNextFileCounter();
    Serial.printf("File counter khởi tạo: %u\n", fileCounter);
    size_t psramSize = ESP.getPsramSize();
    if (psramSize == 0) {
        Serial.println("[ERROR] PSRAM không khả dụng!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Dang Khoi Tao PSRAM...");
        buffer32 = (int32_t*) malloc(BUFFER_SIZE * sizeof(int32_t));
        buffer16 = (int16_t*) malloc(BUFFER_SIZE * sizeof(int16_t));
        if (!buffer32 || !buffer16) {
            Serial.println("[CRITICAL] Khong du bo nho!");
            while(1);
        }
        Serial.println("[WARNING] Su dung RAM chinh thay PSRAM");
    } 
    else {
        Serial.printf("PSRAM size: %d bytes\n", psramSize);
        buffer32 = (int32_t*) ps_malloc(BUFFER_SIZE * sizeof(int32_t));
        buffer16 = (int16_t*) ps_malloc(BUFFER_SIZE * sizeof(int16_t));
        if (!buffer32 || !buffer16) {
            Serial.println("[ERROR] Cap phat PSRAM that bai!");
            buffer32 = (int32_t*) malloc(BUFFER_SIZE * sizeof(int32_t));
            buffer16 = (int16_t*) malloc(BUFFER_SIZE * sizeof(int16_t));
            if (!buffer32 || !buffer16) {
                lcd.print("LOI BO NHO NGHIEM");
                while(1);
            }
        }
    }
    setupI2SMic();

    if (!SD.begin(SD_CS)) {
        Serial.println("[SD] Lỗi khởi động thẻ!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD Card Chua San Sang");
        while (1);
    }
    Serial.println("[SD] Đã sẵn sàng!");

    if (!SD.exists("/ghiam")) {
        SD.mkdir("/ghiam");
        Serial.println("[SD] 📁 Đã tạo thư mục mặc định: /ghiam");
    }

    loadFileList();

    audio.setPinout(BCLK, LRC, DIN);
    audio.setVolume(10);

    xTaskCreatePinnedToCore(TaskAudio, "Audio Task", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskRecord, "Record Task", 8192, NULL, 5, NULL, 1);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CAC NUT DIEU KHIEN");
    lcd.setCursor(0, 1);
    lcd.print("XANH. NHAN NUT PLAY");
    lcd.setCursor(0, 2);
    lcd.print("DO. NHAN NUT RESET");
    lcd.setCursor(0, 3);
    lcd.print("");
}

void loop() {
    static uint8_t debouncedState[3] = {HIGH, HIGH, HIGH};
    static uint8_t previousDebouncedState[3] = {HIGH, HIGH, HIGH};
    static unsigned long lastDebounceTime[3] = {0, 0, 0};
    static uint8_t lastButtonState[3] = {HIGH, HIGH, HIGH};
    static unsigned long pressStartTime = 0;
    static bool isPressingPlay = false;

    unsigned long currentMillis = millis();
    
    const int buttonPins[3] = {BUTTON_PLAY_PIN, BUTTON_DELETE_PIN, BUTTON_RESET_PIN};
    
    for (int i = 0; i < 3; i++) {
        uint8_t reading = digitalRead(buttonPins[i]);
        
        if (reading != lastButtonState[i]) {
            lastDebounceTime[i] = currentMillis;
            lastButtonState[i] = reading;
        }
        
        if ((currentMillis - lastDebounceTime[i]) > debounceDelay) {
            if (reading != debouncedState[i]) {
                previousDebouncedState[i] = debouncedState[i];
                debouncedState[i] = reading;
                
                if (i == 1) {
                    if (debouncedState[i] == LOW && previousDebouncedState[i] == HIGH) {
                        deleteLastRecordedFile();
                    }
                } else if (i == 2) {
                    if (debouncedState[i] == LOW && previousDebouncedState[i] == HIGH) {
                        Serial.println("[BUTTON] Resetting system...");
                        lcd.clear();
                        lcd.setCursor(0, 0);
                        lcd.print("Khoi Dong Lai...");
                        delay(1000);
                        ESP.restart();
                    }
                } else if (i == 0) {
                    if (debouncedState[i] == LOW && previousDebouncedState[i] == HIGH) {
                        pressStartTime = currentMillis;
                        isPressingPlay = true;
                    } else if (debouncedState[i] == HIGH && previousDebouncedState[i] == LOW) {
                        unsigned long pressDuration = currentMillis - pressStartTime;
                        if (pressDuration > 1000) {
                            deleteLastRecordedFile();
                        } else if (pressDuration > 50 && fileCount > 0) {
                            currentFileIndex = (currentFileIndex + 1) % fileCount;
                            
                            if (playState == PLAYING) {
                                playState = IDLE;
                                isPlaying = false;
                                Serial.println("[AUDIO] Đã dừng phát hiện tại");
                            }
                            
                            isRecording = false;
                            hasRecordedForCurrentPlayback = false;
                            recordTriggerTime = 0;
                            
                            currentFileIndex = (currentFileIndex + 1) % fileCount;
                            playState = WAITING_TO_PLAY;
                            playStartTime = currentMillis;
                            
                            lcd.clear();
                            lcd.setCursor(0, 0);
                            lcd.print("Phat File Sau 5s....");
                        }
                        isPressingPlay = false;
                    }
                }
            }
        }
    }
    
    if (playState == WAITING_TO_PLAY && (currentMillis - playStartTime >= 5000)) {
        playFile(currentFileIndex);
        playState = PLAYING;
        isPlaying = true;
        Serial.println("[AUDIO] Bắt đầu phát âm thanh sau 5 giây");
    } else if (debouncedState[0] == HIGH && isPlaying) {
        isPlaying = false;
        playState = IDLE;
        Serial.println("[AUDIO] Dừng phát file");
        Serial.println("[AUDIO] Dừng phát âm thanh");
    }
    
    delay(10);
}