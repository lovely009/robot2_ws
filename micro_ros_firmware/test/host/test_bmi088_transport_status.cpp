#include "bmi088_transport_status.h"

#include <cassert>

static void test_transport_error_latches_until_next_attempt() {
  BMI088_transport_error_clear();
  assert(!BMI088_transport_error_latched());

  BMI088_transport_error_latch();
  assert(BMI088_transport_error_latched());

  BMI088_transport_error_clear();
  assert(!BMI088_transport_error_latched());
}

int main() {
  test_transport_error_latches_until_next_attempt();
  return 0;
}
