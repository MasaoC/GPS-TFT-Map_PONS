#include "settings.h"
#include "sound.h"
#include "gps.h"
#include "mysd.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "SdFat.h"

#define PWM_PIN 9           // PWM output pin
#define AMP_SHUTDOWN_PIN 10 // PAM8302 shutdown pin

//makesure file does not contain meta data.  8 bit unsigned PCM , 16kHz.

extern SdFat32 SD;
const int sampleRate = 16000;  // 16 kHz sample rate
// Audio parameters
const int WAV_HEADER_SIZE = 45;  // Standard WAV header size

// Double buffering setup
const int CHUNK_SIZE = 16 * 1024;  // 8KB chunks for 1.0 seconds of audio
uint8_t audioBuffer[2][CHUNK_SIZE];  // Double buffer
volatile int activeBuffer = 0;  // Currently playing buffer (0 or 1)
volatile int loadBuffer = 1;    // Buffer to load data into (0 or 1, opposite of activeBuffer)
volatile uint32_t bufferPos = 0;  // Position in active buffer
volatile bool playing = false;  // Playback state
volatile int playing_priority = 0;

volatile bool endOfFile = false;  // Flag to indicate end of file reached
volatile bool bufferReady[2] = {false, false};  // Flags to indicate if buffers are loaded
volatile bool bufferSwapRequest = false;  // Flag to request buffer swap


// File object and size tracking
File32 audioFile;
uint32_t totalAudioSize = 0;    // Total size of audio data
uint32_t audioDataRead = 0;     // How much data we've read so far
uint32_t chunksLoaded = 0;      // Counter for tracking how many chunks were loaded

// Timing variables
const unsigned long playInterval = 10000;  // 2 seconds between plays
extern int sound_volume;

//------
const int tableSize = 256;     // Sine wave table size
uint8_t sineTable[tableSize];

volatile uint32_t phaseAcc = 0;
volatile uint32_t phaseInc = 0;

bool wavmode = true;
bool sinmode = true;



// Timer interrupt callback
bool timerCallback(struct repeating_timer *t) {
  if(wavmode){
    if (!playing || !bufferReady[activeBuffer]) {
        return true;  // Nothing to play
    }
    // Output PCM value directly from active buffer
    pwm_set_gpio_level(PWM_PIN, audioBuffer[activeBuffer][bufferPos]*sound_volume/100.0);
    bufferPos++;
    
    // Check if we've reached the end of the current buffer
    if (bufferPos >= CHUNK_SIZE) {
        bufferPos = 0;  // Reset position
        
        // Request buffer swap - we'll handle it in the main loop
        bufferSwapRequest = true;
        DEBUG_P(20250424,"bufferSwapRequest");
    }
  }
  if(sinmode){
    phaseAcc += phaseInc;
    pwm_set_gpio_level(PIN_TONE, sineTable[(phaseAcc >> 24) % tableSize]*sound_volume/100.0); // Output sine wave
  }
  //keep running
  return true;
}


// Controls the PAM8302 amplifier
void setAmplifierState(bool enable) {
    digitalWrite(AMP_SHUTDOWN_PIN, enable ? HIGH : LOW);
    DEBUG_P(20250424,"Amplifier ");
    DEBUG_P(20250424,enable ? "enabled" : "disabled");
}

// Function to load the next chunk of audio data
bool loadNextChunk() {
    if (audioFile && !bufferReady[loadBuffer] && audioDataRead < totalAudioSize) {
        uint32_t remaining = totalAudioSize - audioDataRead;
        uint32_t toRead = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
        
        uint32_t bytesRead = audioFile.read(audioBuffer[loadBuffer], toRead);
        
        if (bytesRead > 0) {
            audioDataRead += bytesRead;
            chunksLoaded++;  // Increment the chunks loaded counter
            // If we couldn't fill the buffer completely, pad with silence
            if (bytesRead < CHUNK_SIZE) {
                // Use 128 as silence for unsigned 8-bit audio. (remove last 1 byte due to value 0 at the end of wav file makes noise.)
                memset(audioBuffer[loadBuffer] + bytesRead, 128, CHUNK_SIZE - bytesRead);
                endOfFile = true;
                DEBUG_PLN(20250424,"endOfFile");
            }
            
            bufferReady[loadBuffer] = true;
            return true;
        } else {
            endOfFile = true;
            return false;
        }
    }
    return false;
}

extern bool sdError;
void startPlayWav(const char* filename, int priority) {
    if(playing && playing_priority > priority){
        DEBUGW_P(20250427,"Start play wav canceled due to priority.:");
        DEBUGW_PLN(20250427,filename);
        return;
    }
    // Stop current playback if any
    wavmode = true;
    sinmode = false;
    playing = false;
    delay(50);  // Give time for interrupt to stop
    
    // Reset state
    endOfFile = false;
    audioDataRead = 0;
    chunksLoaded = 0;  // Reset chunks loaded counter
    bufferPos = 0;
    bufferReady[0] = false;
    bufferReady[1] = false;
    activeBuffer = 0;  // Start with buffer 0 as active
    loadBuffer = 1;    // Start loading into buffer 1
    bufferSwapRequest = false;
    
    // Close previous file if open
    if (audioFile) {
        audioFile.close();
    }
    
    // Open WAV file
    audioFile = SD.open(filename, FILE_READ);
    if (!audioFile) {
        sdError = true;
        DEBUG_P(20250424,"Error opening file: ");
        DEBUG_PLN(20250424,filename);
        enqueueTask(createPlayMultiToneTask(500, 200, 2));//error sound.
        return;
    }
    
    // Get file size and calculate audio data size
    uint32_t fileSize = audioFile.size();
    totalAudioSize = fileSize - WAV_HEADER_SIZE;
    
    DEBUG_P(20250424,"File: ");
    DEBUG_P(20250424,filename);
    DEBUG_P(20250424,", Size: ");
    DEBUG_P(20250424,fileSize);
    DEBUG_P(20250424," bytes, Audio data: ");
    DEBUG_P(20250424,totalAudioSize);
    DEBUG_P(20250424," bytes (");
    DEBUG_P(20250424,(float)totalAudioSize / sampleRate);
    DEBUG_PLN(20250424," seconds)");
    
    // Skip WAV header
    audioFile.seek(WAV_HEADER_SIZE);
    
    // Load initial buffer
    loadNextChunk();  // Load first buffer into loadBuffer (buffer 1)
    
    // Start playback only after we have data
    if (bufferReady[loadBuffer]) {
        // Swap buffers so the first loaded buffer becomes active
        int temp = activeBuffer;
        activeBuffer = loadBuffer;
        loadBuffer = temp;
        
        // Enable the amplifier before starting playback
        setAmplifierState(true);
        
        // Start playback
        bufferPos = 0;
        playing = true;
        playing_priority = priority;
        DEBUG_P(20250424,"Playback started");
    } else {
        DEBUG_P(20250424,"Failed to load initial audio data");
    }
}

void stopPlayback() {
    playing = false;
    delay(50);  // Short delay to ensure the interrupt handler stops
    
    // Disable the amplifier when playback stops
    setAmplifierState(false);
    
    DEBUG_P(20250424,"Playback stopped");
}




void loop_sound(){
    unsigned long currentTime = millis();
    
    // Handle buffer swap request outside of interrupt
    if (bufferSwapRequest && playing) {
        bufferSwapRequest = false;
        bufferReady[activeBuffer] = false;  // Mark current buffer as processed
        
        // Swap buffers if the next one is ready
        if (bufferReady[loadBuffer]) {
            int temp = activeBuffer;
            activeBuffer = loadBuffer;
            loadBuffer = temp;
            DEBUG_P(20240424,"Swapped to buffer ");
            DEBUG_P(20240424,activeBuffer);
            DEBUG_P(20240424,", chunks loaded so far: ");
            DEBUG_PLN(20240424,chunksLoaded);
        } else if (endOfFile) {
            // If next buffer isn't ready and we've reached EOF, stop playing
            DEBUG_PLN(20240424,"End of file reached");
            stopPlayback();  // This will disable the amplifier
        } else {
            // Buffer underrun - this is bad, we couldn't load the next buffer in time
            Serial.println("WARNING: Buffer underrun detected!");
        }
    }
    
    // If we're playing, try to load the next chunk into the load buffer
    if (playing && !bufferReady[loadBuffer] && !endOfFile) {
        if (loadNextChunk()) {
            DEBUG_P(20240424,"Loaded chunk #");
            DEBUG_P(20240424,chunksLoaded);
            DEBUG_P(20240424," into buffer ");
            DEBUG_PLN(20240424,loadBuffer);
        }
    }
    
    
    // Simple playback status indicator - but not too often
    static unsigned long lastStatusTime = 0;
    if (playing && (currentTime - lastStatusTime >= 1000)) {  // Update every second
        lastStatusTime = currentTime;
        float percentComplete = (float)audioDataRead / totalAudioSize * 100.0;
        DEBUG_P(20250424,"Playing: ");
        DEBUG_P(20250424,percentComplete);
        DEBUG_P(20250424,"% complete (");
        DEBUG_P(20250424,chunksLoaded);
        DEBUG_PLN(20250424," chunks loaded)");
    }
}



float last_tone_tt = 0;

float note_frequencies[] = {
    130.81,  // C3 (Do)
    146.83,  // D3 (Re)
    164.81,  // E3 (Mi)
    174.61,  // F3 (Fa)
    196.00,  // G3 (So)
    220.00,  // A3 (La)
    246.94,  // B3 (Si)
    261.63,  // C4 (Do)
    293.66,  // D4 (Re)
    329.63,  // E4 (Mi)
    349.23,  // F4 (Fa)
    392.00,  // G4 (So)
    440.00,  // A4 (La)
    493.88,  // B4 (Si)
    523.25,  // C5 (Do)
    587.33,  // D5 (Re)
    659.25,  // E5 (Mi)
    698.46,  // F5 (Fa)
    783.99,  // G5 (So)
    880.00,  // A5 (La)
    987.77,  // B5 (Si)
    1046.50, // C6 (Do)
    1174.66, // D6 (Re)
    1318.51, // E6 (Mi)
    1396.91, // F6 (Fa)
    1567.98, // G6 (So)
    1760.00, // A6 (La)
    1975.53, // B6 (Si)
    2093.00, // C7 (Do)
    2349.32, // D7 (Re)
    2637.02, // E7 (Mi)
    2793.83, // F7 (Fa)
    3135.96, // G7 (So)
    3520.00, // A7 (La)
    3951.07, // B7 (Si)
    4186.01  // C8 (Do)
};

// Function to find the nearest frequency using binary search
float nearest_note_frequency(float input_freq) {
    int left = 0;
    int right = sizeof(note_frequencies) / sizeof(note_frequencies[0]) - 1;

    // Handle cases outside the known note frequencies
    if (input_freq <= note_frequencies[left]) {
        return note_frequencies[left];
    }
    if (input_freq >= note_frequencies[right]) {
        return note_frequencies[right];
    }

    // Binary search for the closest frequency
    while (left <= right) {
        int mid = (left + right) / 2;

        if (note_frequencies[mid] == input_freq) {
            return note_frequencies[mid];
        } else if (note_frequencies[mid] < input_freq) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    // After binary search, 'left' will be the smallest number greater than 'input_freq'
    // and 'right' will be the largest number smaller than 'input_freq'.
    // Find the nearest one.
    if (fabs(input_freq - note_frequencies[left]) < fabs(input_freq - note_frequencies[right])) {
        return note_frequencies[left];
    } else {
        return note_frequencies[right];
    }
}

// CORE1
void setup_sound(){
    // Initialize amplifier control pin
  pinMode(AMP_SHUTDOWN_PIN, OUTPUT);
  setAmplifierState(false);  // Start with amplifier disabled

  // Initialize PWM with proper settings for audio
  pinMode(PWM_PIN, OUTPUT);
  uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
  pwm_config config = pwm_get_default_config();
  
  pwm_config_set_clkdiv(&config, 8);      //Less than 32 required for audio quality.
  pwm_config_set_wrap(&config, 255);     // 8-bit resolution
  
  pwm_init(slice_num, &config, true);
  gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
  
  // Set PWM to mid-point (silence) initially
  pwm_set_gpio_level(PWM_PIN, 128);

  // Setup timer for 16kHz playback
  static struct repeating_timer timer;
  add_repeating_timer_us(-1000000 / sampleRate, timerCallback, NULL, &timer);

  
    for (int i = 0; i < tableSize; i++) {
        sineTable[i] = 128 + 127 *SIN_VOLUME* sin(2 * M_PI * i / tableSize);
    }

}

unsigned long trackwarning_until;
// CORE0
unsigned long last_update_tone = 0;
void update_tone(float degpersecond){
    if(millis() - last_update_tone < 900){
        Serial.println("errrr");
        return;
    }
    last_update_tone - millis();
  int relativedif = get_gps_truetrack()-last_tone_tt;
  if(relativedif > 180)
    relativedif -= 180;
  if(relativedif < -180)
    relativedif += 180;
  int angle_diff = abs(relativedif);

  //角度変化が大きい時。
  if(angle_diff > 15){
    last_tone_tt = get_gps_truetrack();
    if(get_gps_mps() > 2.0){
        trackwarning_until = millis()+8000;
        enqueueTask(createPlayMultiToneTask(3136,120,angle_diff>30?5:3));// スピーカー性能都合 ソの音
        if(good_sd()){
            enqueueTask(createPlayWavTask("wav/track.wav"));
        }
    }
  }
  //音を出す閾値は、2.0deg/s かつ 2.0 m/s 以上 かつ 音再生設定がある時。
  //また。angle_diffが15より大きい時は上記警告音+wavファイル再生を優先し、deg/sの警告音は流さない。
  else if(abs(degpersecond) > 2.0 && get_gps_mps() > 2.0){
    int freq = abs(degpersecond)*600-680;// 1200-680=520Hz以上 (speaker性能都合)
    if(freq > 3136){
      freq = 3136;// スピーカー性能都合 ソの音
    }
    //精度が低い状況や角度変化少ない状況では、音は短めにする。
    int newduration = (abs(degpersecond)<3 || get_gps_mps() < 4.0)?80:150;
    enqueueTask(createPlayMultiToneTask(nearest_note_frequency(freq), newduration,1));
  }
}


// Change the note
void playNextNote(int frequency) {
    phaseAcc = 0;
    phaseInc = (frequency * tableSize * (1ULL << 24)) / 16150; // Use 1ULL for 64-bit
    DEBUG_P(20250412,"FRQ: ");
    DEBUG_PLN(20250412,frequency);
}


void myTone(int freq,int duration){
  digitalWrite(AMP_SHUTDOWN_PIN,HIGH);
  playNextNote(freq);
  delay(duration);
  digitalWrite(AMP_SHUTDOWN_PIN,LOW);
}



void playTone(int freq, int duration, int counter){
    if(playing){//wait maximum of 3 seconds.
        DEBUG_P(20250424,"failed playTone due to Wav file playing.");
        //caution.  Do not wait wav file finish playing here, because that will stop loop1 and cause wav file unable finish playing.
       return;
    }
    DEBUG_P(20250412,"playTone:counter");
    DEBUG_PLN(20250412,counter);
  wavmode = false;
  sinmode = true;
  myTone(freq, duration);
  if(counter <= 1){
    return;
  }
  for(int i = counter; i > 1; i--){
    delay(duration);
    myTone(freq, duration);
  }
}