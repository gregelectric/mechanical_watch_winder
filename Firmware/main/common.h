#ifndef COMMON_H
#define COMMON_H

enum
{
  PASS,
  FAIL
};

enum
{
  g_state_idle,
  g_state_error,
  g_state_active,
  g_state_estop,
  g_test_estop,
  g_state_ready,
  g_state_alarm_1,
  g_state_alarm_2
};

#endif
