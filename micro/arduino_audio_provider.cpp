/*

I simplified the audio provider from Adafruit sample code - Kumana

==============================================================================*/

#include "audio_provider.h"
#include "micro_features_micro_model_settings.h"
#include <Adafruit_Arcada.h>

#define USE_EDGEBADGE_PDMMIC
#define AUDIO_OUT A0         // uncomment to 'echo' audio to A0 for debugging
#define DEFAULT_BUFFER_SIZE 512

#include <Adafruit_ZeroPDMSPI.h>
#define PDM_SPI            SPI2    // PDM mic SPI peripheral
#define TIMER_CALLBACK     SERCOM3_0_Handler
Adafruit_ZeroPDMSPI pdmspi(&PDM_SPI);

extern Adafruit_Arcada arcada;
void CaptureSamples();

namespace {
bool g_is_audio_initialized = false;
// An internal buffer able to fit 16x our sample size
constexpr int kAudioCaptureBufferSize = DEFAULT_BUFFER_SIZE * 16;
int16_t g_audio_capture_buffer[kAudioCaptureBufferSize];
// A buffer that holds our output
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
// Mark as volatile so we can check in a while loop to see if
// any samples have arrived yet.
volatile int32_t g_latest_audio_timestamp = 0;
// Our callback buffer for collecting a chunk of data
volatile int16_t recording_buffer[DEFAULT_BUFFER_SIZE];

volatile int max_audio = -32768, min_audio = 32768;

void (*audio_capture_cb)() = nullptr;
}  // namespace

void TIMER_CALLBACK() {
  static bool ledtoggle = false;
  static uint32_t audio_idx = 0;
  int32_t sample = 0;

  uint16_t read_pdm;
  if (!pdmspi.decimateFilterWord(&read_pdm)) {
    return; // not ready for data yet!
  }
  sample = read_pdm;
  
  if (audio_idx >= DEFAULT_BUFFER_SIZE) {
    CaptureSamples();
    max_audio = -32768, min_audio = 32768;
    audio_idx = 0;

    if (audio_capture_cb != nullptr) {
      (*audio_capture_cb) ();
    }
  }

  sample -= 32676; // from 0-65535 to -32768 to 32768
#if defined(AUDIO_OUT)
  analogWrite(AUDIO_OUT, sample+2048); 
#endif

  recording_buffer[audio_idx] = sample;
  max_audio = max(max_audio, sample);
  min_audio = min(min_audio, sample);
  audio_idx++;
}

TfLiteStatus InitAudioRecording(tflite::ErrorReporter* error_reporter) {
  Serial.begin(115200);
  
  Serial.println("init audio"); delay(10);
  
  // Hook up the callback that will be called with each sample
  Serial.println("USE_EDGEBADGE_PDMMIC = True!!");
  delay(1000);
  pdmspi.begin(kAudioSampleFrequency);
  Serial.print("Final PDM frequency: "); Serial.println(pdmspi.sampleRate);
#if defined(AUDIO_OUT)
  analogWriteResolution(12);
#endif

  // Block until we have our first audio sample
  while (!g_latest_audio_timestamp) {
    Serial.println("Got first audio timestamp!!");
    delay(100);
  }

  return kTfLiteOk;
}

void SetAudioCaptureCallback(void (*cb)()) {
  audio_capture_cb = cb;
}

void CaptureSamples() {
  // This is how many bytes of new data we have each time this is called
  const int number_of_samples = DEFAULT_BUFFER_SIZE;
  // Calculate what timestamp the last audio sample represents
  const int32_t time_in_ms =
      g_latest_audio_timestamp +
      (number_of_samples / (kAudioSampleFrequency / 1000));
  // Determine the index, in the history of all samples, of the last sample
  const int32_t start_sample_offset =
      g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);
  // Determine the index of this sample in our ring buffer
  const int capture_index = start_sample_offset % kAudioCaptureBufferSize;
  // Read the data to the correct place in our buffer, note 2 bytes per buffer entry
  memcpy(g_audio_capture_buffer + capture_index, (void *)recording_buffer, DEFAULT_BUFFER_SIZE*2);
  // This is how we let the outside world know that new audio data has arrived.
  g_latest_audio_timestamp = time_in_ms;
}

TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  // Set everything up to start receiving audio
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording(error_reporter);
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }
  // This next part should only be called when the main thread notices that the
  // latest audio sample data timestamp has changed, so that there's new data
  // in the capture ring buffer. The ring buffer will eventually wrap around and
  // overwrite the data, but the assumption is that the main thread is checking
  // often enough and the buffer is large enough that this call will be made
  // before that happens.

  // Determine the index, in the history of all samples, of the first
  // sample we want
  const int start_offset = start_ms * (kAudioSampleFrequency / 1000);
  // Determine how many samples we want in total
  const int duration_sample_count =
      duration_ms * (kAudioSampleFrequency / 1000);
  for (int i = 0; i < duration_sample_count; ++i) {
    // For each sample, transform its index in the history of all samples into
    // its index in g_audio_capture_buffer
    const int capture_index = (start_offset + i) % kAudioCaptureBufferSize;
    // Write the sample to the output buffer
    g_audio_output_buffer[i] = g_audio_capture_buffer[capture_index];
  }

  // Set pointers to provide access to the audio
  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;

  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }
