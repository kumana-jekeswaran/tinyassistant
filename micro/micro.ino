
/* Copyright 2018 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include <TensorFlowLite.h>
#include "Adafruit_Arcada.h"
Adafruit_Arcada arcada;

#include "main_functions.h"

#include "audio_provider.h"
#include "command_responder.h"
#include "feature_provider.h"
#include "micro_features_micro_model_settings.h"
#include "micro_features_tiny_conv_micro_features_model_data.h"
#include "recognize_commands.h"
#include "tensorflow/lite/experimental/micro/kernels/micro_ops.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "wifi_provider.h"

#define STATE_SLEEP 0
#define STATE_LISTEN 1
#define LISTEN_DURATION_MS 10 * 1000

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* model_input = nullptr;
FeatureProvider* feature_provider = nullptr;
RecognizeCommands* recognizer = nullptr;
int32_t previous_time = 0;

WifiProvider* wifi_provider;

int current_state = STATE_SLEEP;
long listen_start_time = 0L;

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 10 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

void setup() {
  if (!arcada.arcadaBegin()) {
    while (1);
  }
  // If we are using TinyUSB we will have the filesystem show up!
  arcada.filesysBeginMSD();
  // Start TFT and fill blue
  arcada.displayBegin();
  arcada.display->fillScreen(ARCADA_BLUE);
  arcada.setBacklight(255);

  Serial.begin(115200);
  //while (!Serial) delay(100);
  Serial.println("******************"); delay(10);
  
  if (arcada.filesysBegin()) {
    Serial.println("Found filesystem!");
    delay(10);
  } else {
    arcada.haltBox("No filesystem found! For QSPI flash, load CircuitPython. For SD cards, format with FAT");
  }
  arcada.filesysListFiles();
  // draw intro
  ImageReturnCode stat = arcada.drawBMP((char *)"sahana.bmp", 0, 0);
  if (stat != IMAGE_SUCCESS) {
    arcada.display->fillScreen(ARCADA_BLACK);
    arcada.display->setCursor(0, 0);
    arcada.display->setTextColor(ARCADA_WHITE);
    arcada.display->setTextSize(ceil(arcada.display->width() / 180.0));
    arcada.display->println("Hold microphone");
    arcada.display->println("approx. 6-8\" away ");
    arcada.display->println("from mouth and say");
    arcada.display->println("either YES or NO");
  }
  
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_tiny_conv_micro_features_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  //
  // tflite::ops::micro::AllOpsResolver resolver;
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver micro_mutable_op_resolver;
  micro_mutable_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_mutable_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_FULLY_CONNECTED,
      tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_mutable_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                                       tflite::ops::micro::Register_SOFTMAX());

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, micro_mutable_op_resolver, tensor_arena, kTensorArenaSize,
      error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    error_reporter->Report("AllocateTensors() failed");
    return;
  }

  // Get information about the memory area to use for the model's input.
  model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != kFeatureSliceCount) ||
      (model_input->dims->data[2] != kFeatureSliceSize) ||
      (model_input->type != kTfLiteUInt8)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return;
  }

  // Prepare to access the audio spectrograms from a microphone or other source
  // that will provide the inputs to the neural network.
  // NOLINTNEXTLINE(runtime-global-variables)
  static FeatureProvider static_feature_provider(kFeatureElementCount,
                                                 model_input->data.uint8,
                                                 &audioCaptureCallback);
  feature_provider = &static_feature_provider;

  static RecognizeCommands static_recognizer(error_reporter);
  recognizer = &static_recognizer;

  previous_time = 0;

  static WifiProvider static_wifiprovider;
  wifi_provider = &static_wifiprovider;
  wifi_provider->Load();
  wifi_provider->ConnectToWifi();
}

bool processForWakeupWord() {
  // Fetch the spectrogram for the current time.
  const int32_t current_time = LatestAudioTimestamp();
  int how_many_new_slices = 0;
  TfLiteStatus feature_status = feature_provider->PopulateFeatureData(
          error_reporter, previous_time, current_time, &how_many_new_slices);
  if (feature_status != kTfLiteOk) {
    error_reporter->Report("Feature generation failed");
    return false;
  }
  previous_time = current_time;
  // If no new audio samples have been received since last time, don't bother
  // running the network model.
  if (how_many_new_slices == 0) {
    return false;
  }

  // Run the model on the spectrogram input and make sure it succeeds.
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    error_reporter->Report("Invoke failed");
    return false;
  }

  // Obtain a pointer to the output tensor
  TfLiteTensor* output = interpreter->output(0);
  // Determine whether a command was recognized based on the output of inference
  const char* found_command = nullptr;
  uint8_t score = 0;
  bool is_new_command = false;
  TfLiteStatus process_status = recognizer->ProcessLatestResults(
          output, current_time, &found_command, &score, &is_new_command);
  if (process_status != kTfLiteOk) {
    error_reporter->Report("RecognizeCommands::ProcessLatestResults() failed");
    return false;
  }
  // Do something based on the recognized command. The default implementation
  // just prints to the error console, but you should replace this with your
  // own function for a real application.
  RespondToCommand(error_reporter, current_time, found_command, score,
                   is_new_command);

  return is_new_command && found_command[0] == 'y';
}

void audioCaptureCallback() {
  if (current_state == STATE_LISTEN) {
    Serial.println("TODO: Send to server!!");
  }
}

void loop() {
  switch (current_state) {
    case STATE_SLEEP: {
      bool woke_up = processForWakeupWord();
      if (woke_up) {
        current_state = STATE_LISTEN;
        listen_start_time = millis();
        Serial.println("current_status: STATE_LISTEN");
      }
      break;
    }
    case STATE_LISTEN: {
      long current_time = millis();
      if (current_time - listen_start_time > LISTEN_DURATION_MS) {
        Serial.println("listen timeout reached. time to switch state!");
        current_state = STATE_SLEEP;
        Serial.println("current_status: STATE_SLEEP");
      }
      break;
    }
  }
}
