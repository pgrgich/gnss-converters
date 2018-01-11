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

#include <libsbp/gnss.h>
#include <libsbp/observation.h>
#include <libsbp/logging.h>

#define MAX_OBS_PER_EPOCH 56
/* MAX valid value (ms) for GPS is 604799999 and GLO is 86401999 */
#define INVALID_TIME 0xFFFF

struct rtcm3_sbp_state {
  gps_time_sec_t time_from_rover_obs;
  bool gps_time_updated;
  s8 leap_seconds;
  bool leap_second_known;
  u16 sender_id;
  gps_time_sec_t last_gps_time;
  gps_time_sec_t last_glo_time;
  gps_time_sec_t last_1230_received;
  void (*cb_rtcm_to_sbp)(u16 msg_id, u8 buff, u8 *len, u16 sender_id);
  void (*cb_base_obs_invalid)(double time_diff);
  u8 obs_buffer[sizeof(observation_header_t) +
                MAX_OBS_PER_EPOCH * sizeof(packed_obs_content_t)];
};

void rtcm2sbp_decode_frame(const uint8_t *frame, uint32_t frame_length,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_set_gps_time(gps_time_sec_t *current_time,
                           struct rtcm3_sbp_state *state);

void rtcm2sbp_set_leap_second(s8 leap_seconds, struct rtcm3_sbp_state *state);

void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   void (*cb_rtcm_to_sbp)(u16 msg_id, u8 length, u8 *buffer,
                                          u16 sender_id),
                   void (*cb_base_obs_invalid)(double time_diff));

#endif // GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
