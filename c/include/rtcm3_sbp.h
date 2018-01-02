/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
#define GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H

#include <libsbp/observation.h>
#include <libsbp/gnss.h>

#define MAX_OBS_PER_EPOCH 56
/* MAX valid value (ms) for GPS is 604799999 and GLO is 86401999 */
#define INVALID_TIME 0xFFFF

/* Multiplier for glonass bias resolution scaling */
#define GLO_BIAS_RESOLUTION 50.0

/* How long to wait after receiving a 1230 message before accepting the 1033 message again */
#define MSG_1230_TIMEOUT_SEC 45

/* Third party receiver bias value - these have been sourced from RTCM1230 message, the
 * data can be found with the unit tests*/
#define TRIMBLE_BIAS_M 19.06
#define LEICA_BIAS_M -71.94
#define SEPTENTRIO_BIAS_M 0.0
#define JAVAD_BIAS_M 19.06
#define TOPCON_BIAS_L1CA_M -2.56
#define TOPCON_BIAS_L2P_M 3.74
#define HEMISPHERE_BIAS_L1CA_M -0.2
#define HEMISPHERE_BIAS_L2P_M 3.4
#define JAVAD_BIAS_L1CA_M -1.5
#define JAVAD_BIAS_L2P_M -8.1

struct rtcm3_sbp_state {
  gps_time_sec_t time_from_rover_obs;
  bool gps_time_updated;
  s8 leap_seconds;
  bool leap_second_known;
  u16 sender_id;
  gps_time_sec_t last_gps_time;
  gps_time_sec_t last_glo_time;
  gps_time_sec_t last_1230_received;
  void (*cb_rtcm_to_sbp)(u8 msg_id, u8 buff, u8 *len, u16 sender_id);
  void (*cb_base_obs_invalid)(double time_diff);
  u8 obs_buffer[sizeof(observation_header_t) + MAX_OBS_PER_EPOCH * sizeof(packed_obs_content_t)];
};

void rtcm2sbp_decode_frame(const uint8_t *frame, uint32_t frame_length, struct rtcm3_sbp_state *state);

void rtcm2sbp_set_gps_time(gps_time_sec_t *current_time, struct rtcm3_sbp_state* state);

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   void (*cb_rtcm_to_sbp)(u8 msg_id, u8 length, u8 *buffer, u16 sender_id),
                   void (*cb_base_obs_invalid)(double time_diff));

#endif //GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
