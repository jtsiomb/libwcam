#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "wcam.h"


#define PACK_RGB(r, g, b)	\
	((((r) & 0xff) << 16) | \
	 (((g) & 0xff) << 8) |  \
	 ((b) & 0xff))


/* I/O modes */
enum {
	IO_READWR,		/* read(2) frames */
	IO_MMAP,		/* streaming on mmapped kernel buffers */
	IO_UPTR			/* streaming on user buffers */
};

#define NUM_BUF		4

struct webcam {
	int active;		/* streaming currently? - N/A for read/write mode */
	int iomode;

	/* pointer to active buffer
	 * - read/write: malloced on wcam_open
	 * - streaming: updated on dequeue_buffer
	 */
	unsigned int *buf;
	unsigned int buf_size;

	/* used for streaming I/O only */
	struct {
		void *ptr;
		size_t len;
	} buffers[NUM_BUF];
};

static int request_buffers(int fd, int count);
static int query_buffer(int fd, int idx, size_t *buf_sz, off_t *offset);
static int enqueue_buffer(int fd, int idx);
static int dequeue_buffer(int fd);
static void init_yuv_lut(void);
static int get_v4l_ctlid(int cid);

#define MAX_CAM_FD		512
static struct webcam wcam[MAX_CAM_FD];

/* YUV -> RGB conversion LUT */
static int lut_valid;
static int lut_rv[256];
static int lut_gu[256], lut_gv[256];
static int lut_bu[256];

#define YUV2RED(y, u, v)	CLAMP(((y) + lut_rv[v]), 0, 255)
#define YUV2GREEN(y, u, v)	CLAMP(((y) + lut_gu[u] + lut_gv[v]), 0, 255)
#define YUV2BLUE(y, u, v)	CLAMP(((y) + lut_bu[u]), 0, 255)

#define CLAMP(x, a, b)		((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

int wcam_open(const char *devfile, int xsz, int ysz, int fps)
{
	int i, fd;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_streamparm sparm;

	if(!lut_valid) {
		init_yuv_lut();
	}

	if(!devfile) devfile = "/dev/video0";

	if((fd = open(devfile, O_RDWR)) == -1) {
		perror("failed to open device");
		return -1;
	}

	if(ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
		perror("device does not support V4L2");
		goto err;
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "device does not support video capture\n");
		goto err;
	}

	/* set format */
	memset(&fmt, 0, sizeof fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = xsz;
	fmt.fmt.pix.height = ysz;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

	if(ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
		perror("failed to set video format");
		goto err;
	}
	if(fmt.fmt.pix.width != xsz || fmt.fmt.pix.height != ysz) {
		fprintf(stderr, "got format: %dx%d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
		goto err;
	}
	/* record the size of each frame (in bytes) */
	wcam[fd].buf_size = fmt.fmt.pix.sizeimage;

	/* set framerate */
	memset(&sparm, 0, sizeof sparm);
	sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sparm.parm.capture.timeperframe.numerator = 1;
	sparm.parm.capture.timeperframe.denominator = fps;

	if(ioctl(fd, VIDIOC_S_PARM, &sparm) == -1) {
		perror("failed to set framerate");
		goto err;
	}
	if(sparm.parm.capture.timeperframe.denominator != fps) {
		fprintf(stderr, "got fps: %d\n", sparm.parm.capture.timeperframe.denominator);
	}

	if(cap.capabilities & V4L2_CAP_STREAMING) {
		wcam[fd].iomode = IO_MMAP;

		if(request_buffers(fd, NUM_BUF) == -1) {
			perror("buffer allocation failed");
			goto err;
		}
		
		for(i=0; i<NUM_BUF; i++) {
			size_t blen;
			off_t map_offset;

			if(query_buffer(fd, i, &blen, &map_offset) == -1) {
				perror("failed to query buffer");
				goto err;
			}
			
			wcam[fd].buffers[i].len = blen;
			if((wcam[fd].buffers[i].ptr = mmap(0, blen, PROT_READ | PROT_WRITE, MAP_SHARED,
							fd, map_offset)) == (void*)-1) {
				perror("failed to mmap buffer");
				goto err;
			}

			if(enqueue_buffer(fd, i) == -1) {
				perror("failed to queue buffer");
				goto err;
			}
		}

	} else if(cap.capabilities & V4L2_CAP_READWRITE) {
		wcam[fd].iomode = IO_READWR;

		if(!(wcam[fd].buf = malloc(wcam[fd].buf_size))) {
			perror("failed to allocate user buffer");
			goto err;
		}

	} else {
		fprintf(stderr, "device does not support mmap or read/write\n");
		goto err;
	}

	return fd;

err:
	wcam_close(fd);
	return -1;
}


void wcam_close(int fd)
{
	int i;

	switch(wcam[fd].iomode) {
	case IO_READWR:
		free(wcam[fd].buf);
		break;

	case IO_MMAP:
		if(wcam[fd].active) {
			wcam_stop(fd);
		}
		for(i=0; i<NUM_BUF; i++) {
			munmap(wcam[fd].buffers[i].ptr, wcam[fd].buffers[i].len);
		}
		request_buffers(fd, 0);	/* free all kernel buffers */
		break;

	default:
		assert(0);
	}

	memset(wcam + fd, 0, sizeof *wcam);
	close(fd);
}


int wcam_read_frame(int fd, void *fbuf)
{
	int i, buf_idx;
	unsigned int *buf = fbuf;

	switch(wcam[fd].iomode) {
	case IO_READWR:
		if(read(fd, wcam[fd].buf, wcam[fd].buf_size) == -1) {
			return -1;
		}
		break;

	case IO_MMAP:
		if(!wcam[fd].active) {
			if(wcam_start(fd) == -1) {
				fprintf(stderr, "FOO\n");
				return -1;
			}
		}
		if((buf_idx = dequeue_buffer(fd)) == -1) {
			perror("buffer dequeue failed");
			return -1;
		}
		break;

	case IO_UPTR:
		/* TODO */
	default:
		assert(0);
	}

	for(i=0; i < wcam[fd].buf_size / 4; i++) {
		unsigned int yuyv = wcam[fd].buf[i];
		int r, g, b, y, u, v;

		y = yuyv & 0xff;
		u = (yuyv >> 8) & 0xff;
		v = (yuyv >> 24) & 0xff;

		r = YUV2RED(y, u, v);
		g = YUV2GREEN(y, u, v);
		b = YUV2BLUE(y, u, v);
		*buf++ = PACK_RGB(r, g, b);

		y = (yuyv >> 16) & 0xff;

		r = YUV2RED(y, u, v);
		g = YUV2GREEN(y, u, v);
		b = YUV2BLUE(y, u, v);
		*buf++ = PACK_RGB(r, g, b);
	}

	if(wcam[fd].iomode == IO_MMAP) {
		if(enqueue_buffer(fd, buf_idx) == -1) {
			perror("failed to re-enqueue the buffer after reading");
			return -1;
		}
	}

	return 0;
}

int wcam_wait(int fd)
{
	int res;
	fd_set rdset;

	FD_ZERO(&rdset);
	FD_SET(fd, &rdset);

	while((res = select(fd + 1, &rdset, 0, 0, 0)) < 0 && errno == EINTR);
	return res;
}

/* starts or stops streaming */
int wcam_streamctl(int fd, int state)
{
	int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	/* no effect on read/write mode, only applicable for IO_MMAP/IO_UPTR */
	if(wcam[fd].iomode == IO_READWR) {
		wcam[fd].active = state;
		return 0;
	}

	if(ioctl(fd, state ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type) == -1) {
		return -1;
	}

	wcam[fd].active = state;
	return 0;
}


int wcam_check_ctl(int fd, int ctl_id, int *low, int *high, int *step)
{
	struct v4l2_queryctrl qctl;

	memset(&qctl, 0, sizeof qctl);
	if((qctl.id = get_v4l_ctlid(ctl_id)) == -1) {
		return -1;
	}

	if(ioctl(fd, VIDIOC_QUERYCTRL, &qctl) == -1) {
		return -1;
	}

	switch(qctl.type) {
	case V4L2_CTRL_TYPE_INTEGER:
		if(low) *low = qctl.minimum;
		if(high) *high = qctl.maximum;
		if(step) *step = qctl.step;
		break;

	case V4L2_CTRL_TYPE_BOOLEAN:
		if(low) *low = 0;
		if(high) *high = 1;
		if(step) *step = 1;
		break;

	case V4L2_CTRL_TYPE_MENU:
		if(ctl_id == WCAM_CTL_AUTOEXPOSURE) {
			if(low) *low = 1;
			if(high) *high = 8;
			if(step) *step = 7;
			break;
		}
		
	default:
		return -1;
	}

	return 0;
}

const char *wcam_get_ctl_name(int fd, int ctl_id)
{
	static struct v4l2_queryctrl qctl;

	memset(&qctl, 0, sizeof qctl);
	if((qctl.id = get_v4l_ctlid(ctl_id)) == -1) {
		return 0;
	}

	if(ioctl(fd, VIDIOC_QUERYCTRL, &qctl) == -1) {
		return 0;
	}
	return (const char*)qctl.name;
}

int wcam_set_ctl(int fd, int ctl_id, int val)
{
	struct v4l2_control ctl;

	memset(&ctl, 0, sizeof ctl);
	if((ctl.id = get_v4l_ctlid(ctl_id)) == -1) {
		return -1;
	}
	ctl.value = val;

	return ioctl(fd, VIDIOC_S_CTRL, &ctl);
}

int wcam_get_ctl(int fd, int ctl_id)
{
	struct v4l2_control ctl;

	memset(&ctl, 0, sizeof ctl);
	if((ctl.id = get_v4l_ctlid(ctl_id)) == -1) {
		return -1;
	}

	if(ioctl(fd, VIDIOC_G_CTRL, &ctl) == -1) {
		return -1;
	}
	return ctl.value;
}

/* count of 0 means free all buffers, abort streaming, fuck off */
static int request_buffers(int fd, int count)
{
	struct v4l2_requestbuffers reqbuf;

	memset(&reqbuf, 0, sizeof reqbuf);
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = count;

	if(ioctl(fd, VIDIOC_REQBUFS, &reqbuf) == -1 || reqbuf.count < NUM_BUF) {
		return -1;
	}
	return 0;
}

static int query_buffer(int fd, int idx, size_t *buf_sz, off_t *offset)
{
	struct v4l2_buffer buffer;

	memset(&buffer, 0, sizeof buffer);
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	buffer.index = idx;

	if(ioctl(fd, VIDIOC_QUERYBUF, &buffer) == -1) {
		return -1;
	}

	*buf_sz = buffer.length;
	*offset = buffer.m.offset;

	return buffer.flags & (V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_DONE);
}

static int enqueue_buffer(int fd, int idx)
{
	struct v4l2_buffer buf;

	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = idx;

	return ioctl(fd, VIDIOC_QBUF, &buf);
}

static int dequeue_buffer(int fd)
{
	struct v4l2_buffer buf;

	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if(ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
		return -1;
	}

	wcam[fd].buf = wcam[fd].buffers[buf.index].ptr;
	return buf.index;
}

static void init_yuv_lut(void)
{
	int i;

	for(i=0; i<256; i++) {
		lut_rv[i] = (i - 128) * 1402 / 1000;
		lut_bu[i] = (i - 128) * 1772 / 1000;
		lut_gu[i] = (128 - i) * 714 / 1000;
		lut_gv[i] = (128 - i) * 344 / 1000;
	}
}


#ifndef V4L2_CID_EXPOSURE_AUTO
#define V4L2_CID_EXPOSURE_AUTO	(V4L2_CID_PRIVATE_BASE+10)
#endif

#ifndef V4L2_CID_EXPOSURE_ABSOLUTE
#define V4L2_CID_EXPOSURE_ABSOLUTE	(V4L2_CID_PRIVATE_BASE+11)
#endif

static int get_v4l_ctlid(int cid)
{
	switch(cid) {
	case WCAM_CTL_BRIGHTNESS:
		return V4L2_CID_BRIGHTNESS;

	case WCAM_CTL_CONTRAST:
		return V4L2_CID_CONTRAST;

	case WCAM_CTL_SATURATION:
		return V4L2_CID_SATURATION;

	case WCAM_CTL_HUE:
		return V4L2_CID_HUE;

	case WCAM_CTL_GAMMA:
		return V4L2_CID_GAMMA;

	case WCAM_CTL_EXPOSURE:
		return V4L2_CID_EXPOSURE;

	case WCAM_CTL_EXPOSURE_ABS:
		return V4L2_CID_EXPOSURE_ABSOLUTE;

	case WCAM_CTL_AUTOEXPOSURE:
		return V4L2_CID_EXPOSURE_AUTO;

	case WCAM_CTL_GAIN:
		return V4L2_CID_GAIN;

	case WCAM_CTL_AUTOGAIN:
		return V4L2_CID_AUTOGAIN;

	default:
		break;
	}

	return -1;
}
