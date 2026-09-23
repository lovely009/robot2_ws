#include <cassert>

#include "led_task.h"

int main() {
  const led_color_t red = led_status_select(
      MICRO_ROS_LINK_WAIT_AGENT, false, false, {9U, 8U, 7U});
  assert(red.r == 1U && red.g == 0U && red.b == 0U);
  const led_color_t held = led_status_select(
      MICRO_ROS_LINK_CONNECT, false, false, {9U, 8U, 7U});
  assert(held.r == 9U && held.g == 8U && held.b == 7U);
  const led_color_t green = led_status_select(
      MICRO_ROS_LINK_RUN, true, true, {0U, 0U, 0U});
  assert(green.r == 0U && green.g == 1U && green.b == 0U);
  const led_color_t off = led_status_select(
      MICRO_ROS_LINK_RUN, true, false, {0U, 1U, 0U});
  assert(off.r == 0U && off.g == 0U && off.b == 0U);
  const led_color_t lost = led_status_select(
      MICRO_ROS_LINK_LOST, true, false, {0U, 1U, 0U});
  assert(lost.r == 0U && lost.g == 0U && lost.b == 1U);
  const led_color_t reconnect_wait = led_status_select(
      MICRO_ROS_LINK_WAIT_AGENT, true, false, {1U, 0U, 0U});
  assert(reconnect_wait.r == 0U && reconnect_wait.g == 0U &&
      reconnect_wait.b == 1U);
}