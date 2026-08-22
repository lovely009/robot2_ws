#include "bmi088_transport_status.h"

static volatile bool g_transport_error = false;

void BMI088_transport_error_clear(void) {
  g_transport_error = false;
}

bool BMI088_transport_error_latched(void) {
  return g_transport_error;
}

void BMI088_transport_error_latch(void) {
  g_transport_error = true;
}
