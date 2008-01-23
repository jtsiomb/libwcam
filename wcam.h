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

/* reads a frame and converts the result to RGBA */
int wcam_read_frame(int fd, void *fbuf);

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
