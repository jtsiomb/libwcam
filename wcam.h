/*
This file is a small video input library for video4linux2 devices
Copyright (C) 2008 John Tsiombikas <nuclear@member.fsf.org>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _WCAM_H_
#define _WCAM_H_

enum {
	WCAM_CTL_BRIGHTNESS,
	WCAM_CTL_CONTRAST,
	WCAM_CTL_SATURATION,
	WCAM_CTL_HUE,
	WCAM_CTL_AUTO_WHITE_BALANCE,
	WCAM_CTL_AUTO_WHITE_BALANCE_TEMP,
	WCAM_CTL_GAMMA,
	WCAM_CTL_EXPOSURE,
	WCAM_CTL_EXPOSURE_ABS,
	WCAM_CTL_AUTO_EXPOSURE,
	WCAM_CTL_GAIN,
	WCAM_CTL_AUTOGAIN,

	WCAM_CTL_COUNT
};

#define WCAM_AUTOEXP_ON		8
#define WCAM_AUTOEXP_OFF	1

#ifdef __cplusplus
extern "C" {
#endif

int wcam_open(const char *devfile, int xsz, int ysz, int fps);
void wcam_close(int fd);

void wcam_set_opt(int fd, int opt, int val);
int wcam_get_opt(int fd, int opt);

int wcam_read_frame_yuv(int fd, void *fbuf);
int wcam_read_frame_rgb(int fd, void *fbuf);

/* waits until a frame is ready to be read */
int wcam_wait(int fd);

/* start/stop video capture */
#define wcam_start(fd)	wcam_streamctl(fd, 1)
#define wcam_stop(fd)	wcam_streamctl(fd, 0)

int wcam_streamctl(int fd, int state);

int wcam_check_ctl(int fd, int ctl_id, int *low, int *high, int *step);
const char *wcam_get_ctl_name(int fd, int ctl_id);
int wcam_set_ctl(int fd, int ctl_id, int val);
int wcam_get_ctl(int fd, int ctl_id);

#ifdef __cplusplus
}
#endif

#endif	/* _WCAM_H_ */
