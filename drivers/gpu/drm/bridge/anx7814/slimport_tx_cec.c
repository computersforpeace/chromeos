/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "slimport.h"
#include "slimport_tx_drv.h"
#include "slimport_tx_cec.h"


#ifdef CEC_ENABLE

static unchar DP_CECSend_Frame[CEC_FRAME_SIZE];
static unchar DP_CECRecv_Buf[CEC_BUF_SIZE];
static unchar DP_SendFrame_len;
static unchar *DP_CECRecv_cur;

static unchar HDMI_CECSend_Frame[CEC_FRAME_SIZE];
static unchar HDMI_CECRecv_Buf[CEC_BUF_SIZE];
static unchar HDMI_SendFrame_len;
static unchar *HDMI_CECRecv_cur;

struct {
	uint hdmi_cec_receive_done:1;
	uint hdmi_cec_transmit_done:1;
	uint dp_cec_receive_done:1;
	uint dp_cec_transmit_done:1;
	uint dp_resend_flag:1;
	uint hdmi_resend_flag:1;
	uint hdmi_frame_ready:1;
	uint dp_frame_ready:1;
	uint dp_frametx_error:1;
	uint dp_resend_count:3;
	uint hdmi_resend_count:3;
} cec;

static unchar dp_timeout_count;
static unchar hdmi_timeout_count;

void cec_init(void)
{
	unchar c, i;

	sp_write_reg(RX_P0, RX_CEC_CTRL, 0x09);


	c = CEC_DEVICE_TYPE << 4 | 0x09;
	for (i = 0; i < 2; i++)	{
		if (AUX_OK ==
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c))
			break;
	}

	DP_CECRecv_cur = DP_CECRecv_Buf;
	HDMI_CECRecv_cur = HDMI_CECRecv_Buf;
	dp_timeout_count = 0;
	hdmi_timeout_count = 0;
	cec.hdmi_cec_receive_done = 0;
	cec.hdmi_cec_transmit_done = 0;
	cec.dp_cec_receive_done = 0;
	cec.dp_cec_transmit_done = 0;
	cec.dp_resend_flag = 0;
	cec.hdmi_resend_flag = 0;
	cec.dp_frame_ready = 0;
	cec.hdmi_frame_ready = 0;
	cec.dp_frametx_error = 0;
	cec.dp_resend_count = 0;
	cec.hdmi_resend_count = 0;
	pr_info("%s %s :cec initialed!\n", LOG_TAG, __func__);
}

unchar dp_buf_length_get(void)
{
	return (unchar)(DP_CECRecv_cur - DP_CECRecv_Buf);
}

unchar hdmi_buf_length_get(void)
{
	return (unchar)(HDMI_CECRecv_cur - HDMI_CECRecv_Buf);
}

void FIFO_read(unchar direction, unchar len)
{
	unchar c, i;

	for (i = 0; i < len; i++) {
		if (direction == DP) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x80, 1, &c);
			*DP_CECRecv_cur = c;
			DP_CECRecv_cur++;
		} else {
			sp_read_reg(RX_P0, HDMI_RX_CEC_FIFO_REG, &c);
			*HDMI_CECRecv_cur = c;
			HDMI_CECRecv_cur++;
		}
	}

}

unchar cec_frame_parse(unchar direction)
{
	unchar i, BufLen, FrameLen = 0;
	unchar *buf, *cur, *frame;

	if (direction == DP) {
		sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &i);
		if ((i & 0x10) == 0x0)
			return TASK_NG;

		buf = DP_CECRecv_Buf;
		frame = DP_CECSend_Frame;
		cur = DP_CECRecv_cur;
	 } else {
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x72, 1, &i);
		if ((i & 0x20) == 0x0)
			return TASK_NG;

		buf = HDMI_CECRecv_Buf;
		frame = HDMI_CECSend_Frame;
		cur = HDMI_CECRecv_cur;
	 }

	if (cur > buf)
		BufLen = cur - buf;
	else
		return TASK_NG;

	for (i = 0; i < BufLen; i += 2) {
		if (i > 0 && *(buf + i) == 0x01)
			break;
		else
			*frame = *(buf + i + 1);
		frame++;
	}
	FrameLen = i / 2;
	cur -=  i;

	if (cur > buf) {
		for (i = 0; i < BufLen - FrameLen * 2; i++)
			*(buf + i) = *(buf + FrameLen * 2 + i);
	}
	if (direction == DP) {
		DP_CECRecv_cur = cur;
		DP_SendFrame_len = FrameLen;
	} else {
		HDMI_CECRecv_cur = cur;
		HDMI_SendFrame_len = FrameLen;
	}

	 return TASK_OK;
}

unchar cec_frame_check(unchar direction)
{
	if (direction == DP) {
		if (DP_CECSend_Frame[0] == (CEC_DEVICE_TYPE << 4 | 0x0f))
			return TASK_NG;
	} else {
		if (HDMI_CECSend_Frame[0] == 0x0f)
			return TASK_NG;
	}
	return TASK_OK;
}
unchar dp_cec_recv_process(void)
{
	unchar status, FIFOLen;

	if (AUX_OK != sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &status))
		return TASK_NG;

	FIFOLen = status & 0x1F;

	if (FIFOLen == 0)
		return TASK_NG;
	if (status & 0x40) {
		FIFOLen = 0x10;
		pr_info("%s %s :DP cec FIFO full!\n", LOG_TAG, __func__);
	}
	FIFO_read(DP, FIFOLen << 1);
	return TASK_OK;
}


unchar hdmi_cec_send_process(void)
{
	unchar status, c, i;

	sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &c);
	if (status & HDMI_RX_CEC_RX_BUSY)
		return TASK_NG;
	else {
		sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &status);
		if ((status & 0x90) == 0x10) {
			for (i = 0; i < DP_SendFrame_len; i++)
				sp_write_reg(RX_P0, HDMI_RX_CEC_FIFO_REG,
					DP_CECSend_Frame[i]);

		} else
			return TASK_NG;
		sp_write_reg(RX_P0, RX_CEC_CTRL, 0x0c);
		return TASK_OK;
	}

}


unchar hdmi_cec_recv_process(void)
{
	unchar status, FIFOLen;

	sp_read_reg(RX_P0, HDMI_RX_CEC_RX_STATUS_REG, &status);

	FIFOLen = status & 0x0F;

	if (FIFOLen == 0)
		return TASK_NG;
	if (status & HDMI_RX_CEC_RX_FULL) {
		FIFOLen = 0x10;
		pr_info("%s %s :hdmi cec FIFO full!\n", LOG_TAG, __func__);
	}

	FIFO_read(HDMI, FIFOLen << 1);
	return TASK_OK;
}

unchar dp_cec_send_process(void)
{
	unchar status, c, i;

	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x71, 1, &c);
	if (status & 0x80)
		return TASK_NG;
	else {
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x72, 1, &status);
		if ((status & 0xa0) == 0x20) {
			for (i = 0; i < HDMI_SendFrame_len; i++)
				sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x80,
						1, HDMI_CECSend_Frame+i);

		} else
			return TASK_NG;

		c = CEC_DEVICE_TYPE << 4 | 0x0c;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c);

		return TASK_OK;
	}

}


void CEC_handler(void)
{
	unchar result, c;

	if (cec.dp_cec_receive_done) {
		if (cec.hdmi_resend_flag == 1 || cec.hdmi_frame_ready == 1)
			result = TASK_OK;
		else {
			result = cec_frame_parse(DP);
			if (result == TASK_OK) {
				if (TASK_OK == cec_frame_check(DP))
					cec.hdmi_frame_ready = 1;
				else {
					result = TASK_NG;
					cec.dp_cec_receive_done = 0;
				}
			}
		}

		if (result == TASK_OK) {
			result = hdmi_cec_send_process();
			if (result == TASK_OK) {
				cec.dp_cec_receive_done = 0;
				cec.hdmi_frame_ready = 0;
				hdmi_timeout_count = CEC_TIMEOUT_COUNT;
			}
		}
	}

	if (cec.hdmi_cec_transmit_done) {
		hdmi_timeout_count = 0;
		sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &c);

		if ((c & 0x40) == 0) {
			cec.hdmi_resend_flag = 0;
			cec.hdmi_resend_count = 0;
			if ((c & 0x90) == 0x10) {
				if (dp_buf_length_get()) {
					sp_tx_aux_dpcdread_bytes(0x00, 0x05,
								0x71, 1, &c);
					if ((c & 0xa0) == 0x20
						&& cec.hdmi_frame_ready == 0) {
						cec.dp_cec_receive_done = 1;
						cec.hdmi_cec_transmit_done = 0;
					}
				} else {
					cec.dp_cec_receive_done = 0;
					cec.hdmi_cec_transmit_done = 0;

				}

			}

		} else {
			pr_info("%s %s : DP -> HDMI CEC data transmit fail!\n",
					LOG_TAG, __func__);
			cec.hdmi_cec_transmit_done = 0;
			sp_write_reg(RX_P0, RX_CEC_CTRL, 0x09);

			if (cec.hdmi_resend_flag == 0) {
				cec.hdmi_resend_flag = 1;
				cec.hdmi_resend_count = 2;
			}

			if (cec.hdmi_resend_count > 0) {
				cec.hdmi_resend_count--;
				cec.dp_cec_receive_done = 1;
			} else
				cec.hdmi_resend_flag = 0;


		}
	} else if (hdmi_timeout_count > 0) {
		hdmi_timeout_count--;
		if (hdmi_timeout_count == 1) {
			sp_read_reg(RX_P0, HDMI_RX_CEC_TX_STATUS_REG, &c);
			if (c & 0x80)
				sp_write_reg(RX_P0, RX_CEC_CTRL, 0x09);
		}
	}

	if (cec.hdmi_cec_receive_done) {
		if (cec.dp_resend_flag == 1 || cec.dp_frame_ready == 1)
			result = TASK_OK;
		else {
			result = cec_frame_parse(HDMI);
			if (result == TASK_OK) {
				if (TASK_OK == cec_frame_check(HDMI))
					cec.dp_frame_ready = 1;
				else {
					result = TASK_NG;
					cec.hdmi_cec_receive_done = 0;
				}
			}
		}

		if (result == TASK_OK) {
			result = dp_cec_send_process();
			if (result == TASK_OK) {
				cec.hdmi_cec_receive_done = 0;
				cec.dp_frame_ready = 0;
				dp_timeout_count = CEC_TIMEOUT_COUNT;
			}
		}
	}

	if (cec.dp_cec_transmit_done) {
		dp_timeout_count = 0;
		if (cec.dp_frametx_error == 0) {
			cec.dp_resend_flag = 0;
			cec.dp_resend_count = 0;
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x72, 1, &c);

			if ((c & 0xa0) == 0x20) {
				if (hdmi_buf_length_get()) {
					sp_read_reg(RX_P0,
						HDMI_RX_CEC_RX_STATUS_REG, &c);
					if ((c & 0x90) == 0x10
					&& cec.dp_frame_ready == 0) {
						cec.hdmi_cec_receive_done = 1;
						cec.dp_cec_transmit_done = 0;
					}
				} else {
					cec.hdmi_cec_receive_done = 0;
					cec.dp_cec_transmit_done = 0;
				}
			}

		} else {
			pr_info("%s %s :HDMI -> DP CEC data transmit fail!\n",
					LOG_TAG, __func__);
			cec.dp_cec_transmit_done = 0;
			cec.dp_frametx_error = 0;
			c = CEC_DEVICE_TYPE << 4 | 0x09;
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x05, 0x70, 1, &c);

			if (cec.dp_resend_flag == 0) {
				cec.dp_resend_flag = 1;
				cec.dp_resend_count = 2;
			}

			if (cec.dp_resend_count > 0) {
				cec.dp_resend_count--;
				cec.hdmi_cec_receive_done = 1;
			} else
				cec.dp_resend_flag = 0;


		}

	} else if (dp_timeout_count > 0) {
		dp_timeout_count--;
		if (dp_timeout_count == 1) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x72, 1, &c);
			if (c & 0x80) {
				c = CEC_DEVICE_TYPE << 4 | 0x09;
				sp_tx_aux_dpcdwrite_bytes(0x00,
						0x05, 0x70, 1, &c);
			}

		}
	}
}

#endif

