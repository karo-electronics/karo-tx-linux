/*
 * Copyright 2005-2009 Freescale Semiconductor, Inc.
 *
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#ifndef __ASM_ARCH_IPU_H__
#define __ASM_ARCH_IPU_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/bitmap.h>
#include <linux/fb.h>
#include <mach/hardware.h>

struct ipu_soc;

/*
 * IPU Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/* Generic or Raw Data Formats */
#define IPU_PIX_FMT_GENERIC v4l2_fourcc('I', 'P', 'U', '0')	/* IPU Generic Data */
#define IPU_PIX_FMT_GENERIC_32 v4l2_fourcc('I', 'P', 'U', '1')	/* IPU Generic Data */
#define IPU_PIX_FMT_LVDS666 v4l2_fourcc('L', 'V', 'D', '6')	/* IPU Generic Data */
#define IPU_PIX_FMT_LVDS888 v4l2_fourcc('L', 'V', 'D', '8')	/* IPU Generic Data */
/* RGB Formats */
#define IPU_PIX_FMT_RGB332  V4L2_PIX_FMT_RGB332			/*  8  RGB-3-3-2    */
#define IPU_PIX_FMT_RGB555  V4L2_PIX_FMT_RGB555			/* 16  RGB-5-5-5    */
#define IPU_PIX_FMT_RGB565  V4L2_PIX_FMT_RGB565			/* 1 6  RGB-5-6-5   */
#define IPU_PIX_FMT_RGB666  v4l2_fourcc('R', 'G', 'B', '6')	/* 18  RGB-6-6-6    */
#define IPU_PIX_FMT_BGR666  v4l2_fourcc('B', 'G', 'R', '6')	/* 18  BGR-6-6-6    */
#define IPU_PIX_FMT_BGR24   V4L2_PIX_FMT_BGR24			/* 24  BGR-8-8-8    */
#define IPU_PIX_FMT_RGB24   V4L2_PIX_FMT_RGB24			/* 24  RGB-8-8-8    */
#define IPU_PIX_FMT_GBR24   v4l2_fourcc('G', 'B', 'R', '3')	/* 24  GBR-8-8-8    */
#define IPU_PIX_FMT_BGR32   V4L2_PIX_FMT_BGR32			/* 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_BGRA32  v4l2_fourcc('B', 'G', 'R', 'A')	/* 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_RGB32   V4L2_PIX_FMT_RGB32			/* 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_RGBA32  v4l2_fourcc('R', 'G', 'B', 'A')	/* 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_ABGR32  v4l2_fourcc('A', 'B', 'G', 'R')	/* 32  ABGR-8-8-8-8 */
/* YUV Interleaved Formats */
#define IPU_PIX_FMT_YUYV    V4L2_PIX_FMT_YUYV			/* 16 YUV 4:2:2 */
#define IPU_PIX_FMT_UYVY    V4L2_PIX_FMT_UYVY			/* 16 YUV 4:2:2 */
#define IPU_PIX_FMT_Y41P    V4L2_PIX_FMT_Y41P			/* 12 YUV 4:1:1 */
#define IPU_PIX_FMT_YUV444  V4L2_PIX_FMT_YUV444			/* 24 YUV 4:4:4 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define IPU_PIX_FMT_NV12    V4L2_PIX_FMT_NV12			/* 12  Y/CbCr 4:2:0  */
/* YUV Planar Formats */
#define IPU_PIX_FMT_GREY    V4L2_PIX_FMT_GREY			/* 8  Greyscale */
#define IPU_PIX_FMT_YVU410P V4L2_PIX_FMT_YVU410P		/* 9  YVU 4:1:0 */
#define IPU_PIX_FMT_YUV410P V4L2_PIX_FMT_YUV410P		/* 9  YUV 4:1:0 */
#define IPU_PIX_FMT_YVU420P v4l2_fourcc('Y', 'V', '1', '2')	/* 12 YVU 4:2:0 */
#define IPU_PIX_FMT_YUV420P v4l2_fourcc('I', '4', '2', '0')	/* 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YUV420P2 v4l2_fourcc('Y', 'U', '1', '2')	/* 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YVU422P v4l2_fourcc('Y', 'V', '1', '6')	/* 16 YVU 4:2:2 */
#define IPU_PIX_FMT_YUV422P V4L2_PIX_FMT_YUV422P		/* 16 YUV 4:2:2 */

/*
 * Bitfield of Display Interface signal polarities.
 */
struct ipu_di_signal_cfg {
	unsigned datamask_en:1;
	unsigned ext_clk:1;
	unsigned interlaced:1;
	unsigned odd_field_first:1;
	unsigned clksel_en:1;
	unsigned clkidle_en:1;
	unsigned data_pol:1;	/* true = inverted */
	unsigned clk_pol:1;	/* true = rising edge */
	unsigned enable_pol:1;
	unsigned Hsync_pol:1;	/* true = active high */
	unsigned Vsync_pol:1;

	u16 width;
	u16 height;
	u32 pixel_fmt;
	u16 h_start_width;
	u16 h_sync_width;
	u16 h_end_width;
	u16 v_start_width;
	u16 v_sync_width;
	u16 v_end_width;
	u32 v_to_h_sync;
};

typedef enum {
	IPU_COLORSPACE_RGB,
	IPU_COLORSPACE_YCBCR,
	IPU_COLORSPACE_YUV,
	IPU_COLORSPACE_UNKNOWN,
} ipu_color_space_t;

#define IPU_IRQ_EOF(channel)		(channel)		/* 0 .. 63 */
#define IPU_IRQ_NFACK(channel)		((channel) + 64)	/* 64 .. 127 */
#define IPU_IRQ_NFB4EOF(channel)	((channel) + 128)	/* 128 .. 191 */
#define IPU_IRQ_EOS(channel)		((channel) + 192)	/* 192 .. 255 */

#define IPU_IRQ_DP_SF_START		(448 + 2)
#define IPU_IRQ_DP_SF_END		(448 + 3)
#define IPU_IRQ_BG_SF_END		IPU_IRQ_DP_SF_END,
#define IPU_IRQ_DC_FC_0			(448 + 8)
#define IPU_IRQ_DC_FC_1			(448 + 9)
#define IPU_IRQ_DC_FC_2			(448 + 10)
#define IPU_IRQ_DC_FC_3			(448 + 11)
#define IPU_IRQ_DC_FC_4			(448 + 12)
#define IPU_IRQ_DC_FC_6			(448 + 13)
#define IPU_IRQ_VSYNC_PRE_0		(448 + 14)
#define IPU_IRQ_VSYNC_PRE_1		(448 + 15)

#define IPU_IRQ_COUNT	(15 * 32)

struct ipu_channel;

/*
 * IPU Image DMA Controller (idmac) functions
 */
struct ipu_channel *ipu_idmac_get(struct ipu_soc *ipu, unsigned channel);
void ipu_idmac_put(struct ipu_channel *);

int ipu_idmac_enable_channel(struct ipu_channel *channel);
int ipu_idmac_disable_channel(struct ipu_channel *channel);

void ipu_idmac_set_double_buffer(struct ipu_channel *channel, bool doublebuffer);
void ipu_idmac_select_buffer(struct ipu_channel *channel, u32 buf_num);

/*
 * IPU Display Controller (dc) functions
 */
struct ipu_dc;
struct ipu_dc *ipu_dc_get(struct ipu_soc *ipu, int channel);
void ipu_dc_put(struct ipu_dc *dc);
int ipu_dc_init_sync(struct ipu_dc *dc, int di, bool interlaced,
		u32 pixel_fmt, u32 width);
void ipu_dc_init_async(struct ipu_dc *dc, int di, bool interlaced);
void ipu_dc_enable_channel(struct ipu_dc *dc);
void ipu_dc_disable_channel(struct ipu_dc *dc);

/*
 * IPU Display Interface (di) functions
 */
struct ipu_di;
struct ipu_di *ipu_di_get(struct ipu_soc *ipu, int disp);
void ipu_di_put(struct ipu_di *);
int ipu_di_disable(struct ipu_di *);
int ipu_di_enable(struct ipu_di *);
int ipu_di_init_sync_panel(struct ipu_di *, struct ipu_di_signal_cfg *sig);

/*
 * IPU Display Multi FIFO Controller (dmfc) functions
 */
struct dmfc_channel;
int ipu_dmfc_enable_channel(struct dmfc_channel *dmfc);
void ipu_dmfc_disable_channel(struct dmfc_channel *dmfc);
int ipu_dmfc_alloc_bandwidth(struct dmfc_channel *dmfc, unsigned long bandwidth_mbs,
		int burstsize);
void ipu_dmfc_free_bandwidth(struct dmfc_channel *dmfc);
int ipu_dmfc_init_channel(struct dmfc_channel *dmfc, int width);
struct dmfc_channel *ipu_dmfc_get(struct ipu_soc *ipu, int ipu_channel);
void ipu_dmfc_put(struct dmfc_channel *dmfc);

/*
 * IPU Display Processor (dp) functions
 */
#define IPU_DP_FLOW_SYNC_BG	0
#define IPU_DP_FLOW_SYNC_FG	1
#define IPU_DP_FLOW_ASYNC0_BG	2
#define IPU_DP_FLOW_ASYNC0_FG	3
#define IPU_DP_FLOW_ASYNC1_BG	4
#define IPU_DP_FLOW_ASYNC1_FG	5

struct ipu_dp *ipu_dp_get(struct ipu_soc *ipu, unsigned int flow);
void ipu_dp_put(struct ipu_dp *);
int ipu_dp_enable_channel(struct ipu_dp *dp);
void ipu_dp_disable_channel(struct ipu_dp *dp);
int ipu_dp_setup_channel(struct ipu_dp *dp,
		ipu_color_space_t in, ipu_color_space_t out);
int ipu_dp_set_window_pos(struct ipu_dp *, u16 x_pos, u16 y_pos);
int ipu_dp_set_global_alpha(struct ipu_dp *dp, bool enable, u8 alpha,
		bool bg_chan);

#define IPU_CPMEM_WORD(word, ofs, size) ((((word) * 160 + (ofs)) << 8) | (size))

#define IPU_FIELD_UBO		IPU_CPMEM_WORD(0, 46, 22)
#define IPU_FIELD_VBO		IPU_CPMEM_WORD(0, 68, 22)
#define IPU_FIELD_IOX		IPU_CPMEM_WORD(0, 90, 4)
#define IPU_FIELD_RDRW		IPU_CPMEM_WORD(0, 94, 1)
#define IPU_FIELD_SO		IPU_CPMEM_WORD(0, 113, 1)
#define IPU_FIELD_SLY		IPU_CPMEM_WORD(1, 102, 14)
#define IPU_FIELD_SLUV		IPU_CPMEM_WORD(1, 128, 14)

#define IPU_FIELD_XV		IPU_CPMEM_WORD(0, 0, 10)
#define IPU_FIELD_YV		IPU_CPMEM_WORD(0, 10, 9)
#define IPU_FIELD_XB		IPU_CPMEM_WORD(0, 19, 13)
#define IPU_FIELD_YB		IPU_CPMEM_WORD(0, 32, 12)
#define IPU_FIELD_NSB_B		IPU_CPMEM_WORD(0, 44, 1)
#define IPU_FIELD_CF		IPU_CPMEM_WORD(0, 45, 1)
#define IPU_FIELD_SX		IPU_CPMEM_WORD(0, 46, 12)
#define IPU_FIELD_SY		IPU_CPMEM_WORD(0, 58, 11)
#define IPU_FIELD_NS		IPU_CPMEM_WORD(0, 69, 10)
#define IPU_FIELD_SDX		IPU_CPMEM_WORD(0, 79, 7)
#define IPU_FIELD_SM		IPU_CPMEM_WORD(0, 86, 10)
#define IPU_FIELD_SCC		IPU_CPMEM_WORD(0, 96, 1)
#define IPU_FIELD_SCE		IPU_CPMEM_WORD(0, 97, 1)
#define IPU_FIELD_SDY		IPU_CPMEM_WORD(0, 98, 7)
#define IPU_FIELD_SDRX		IPU_CPMEM_WORD(0, 105, 1)
#define IPU_FIELD_SDRY		IPU_CPMEM_WORD(0, 106, 1)
#define IPU_FIELD_BPP		IPU_CPMEM_WORD(0, 107, 3)
#define IPU_FIELD_DEC_SEL	IPU_CPMEM_WORD(0, 110, 2)
#define IPU_FIELD_DIM		IPU_CPMEM_WORD(0, 112, 1)
#define IPU_FIELD_BNDM		IPU_CPMEM_WORD(0, 114, 3)
#define IPU_FIELD_BM		IPU_CPMEM_WORD(0, 117, 2)
#define IPU_FIELD_ROT		IPU_CPMEM_WORD(0, 119, 1)
#define IPU_FIELD_HF		IPU_CPMEM_WORD(0, 120, 1)
#define IPU_FIELD_VF		IPU_CPMEM_WORD(0, 121, 1)
#define IPU_FIELD_THE		IPU_CPMEM_WORD(0, 122, 1)
#define IPU_FIELD_CAP		IPU_CPMEM_WORD(0, 123, 1)
#define IPU_FIELD_CAE		IPU_CPMEM_WORD(0, 124, 1)
#define IPU_FIELD_FW		IPU_CPMEM_WORD(0, 125, 13)
#define IPU_FIELD_FH		IPU_CPMEM_WORD(0, 138, 12)
#define IPU_FIELD_EBA0		IPU_CPMEM_WORD(1, 0, 29)
#define IPU_FIELD_EBA1		IPU_CPMEM_WORD(1, 29, 29)
#define IPU_FIELD_ILO		IPU_CPMEM_WORD(1, 58, 20)
#define IPU_FIELD_NPB		IPU_CPMEM_WORD(1, 78, 7)
#define IPU_FIELD_PFS		IPU_CPMEM_WORD(1, 85, 4)
#define IPU_FIELD_ALU		IPU_CPMEM_WORD(1, 89, 1)
#define IPU_FIELD_ALBM		IPU_CPMEM_WORD(1, 90, 3)
#define IPU_FIELD_ID		IPU_CPMEM_WORD(1, 93, 2)
#define IPU_FIELD_TH		IPU_CPMEM_WORD(1, 95, 7)
#define IPU_FIELD_SL		IPU_CPMEM_WORD(1, 102, 14)
#define IPU_FIELD_WID0		IPU_CPMEM_WORD(1, 116, 3)
#define IPU_FIELD_WID1		IPU_CPMEM_WORD(1, 119, 3)
#define IPU_FIELD_WID2		IPU_CPMEM_WORD(1, 122, 3)
#define IPU_FIELD_WID3		IPU_CPMEM_WORD(1, 125, 3)
#define IPU_FIELD_OFS0		IPU_CPMEM_WORD(1, 128, 5)
#define IPU_FIELD_OFS1		IPU_CPMEM_WORD(1, 133, 5)
#define IPU_FIELD_OFS2		IPU_CPMEM_WORD(1, 138, 5)
#define IPU_FIELD_OFS3		IPU_CPMEM_WORD(1, 143, 5)
#define IPU_FIELD_SXYS		IPU_CPMEM_WORD(1, 148, 1)
#define IPU_FIELD_CRE		IPU_CPMEM_WORD(1, 149, 1)
#define IPU_FIELD_DEC_SEL2	IPU_CPMEM_WORD(1, 150, 1)

struct ipu_cpmem_word {
	u32 data[5];
	u32 res[3];
};

struct ipu_ch_param {
	struct ipu_cpmem_word word[2];
};

void ipu_ch_param_set_field(struct ipu_ch_param *base, u32 wbs, u32 v);

struct ipu_ch_param *ipu_get_cpmem(struct ipu_channel *channel);

extern void ipu_get(struct ipu_soc *ipu);
extern void ipu_put(struct ipu_soc *ipu);

void ipu_ch_param_dump(struct ipu_ch_param *p);

static inline void ipu_cpmem_set_buffer(struct ipu_ch_param *p, int bufnum,
		dma_addr_t buf)
{
	if (bufnum)
		ipu_ch_param_set_field(p, IPU_FIELD_EBA1, buf >> 3);
	else
		ipu_ch_param_set_field(p, IPU_FIELD_EBA0, buf >> 3);
}

static inline void ipu_cpmem_set_resolution(struct ipu_ch_param *p, int xres,
		int yres)
{
	ipu_ch_param_set_field(p, IPU_FIELD_FW, xres - 1);
	ipu_ch_param_set_field(p, IPU_FIELD_FH, yres - 1);
}

static inline void ipu_cpmem_set_stride(struct ipu_ch_param *p, int stride)
{
	ipu_ch_param_set_field(p, IPU_FIELD_SLY, stride - 1);
}

static inline void ipu_cpmem_set_high_priority(struct ipu_ch_param *p)
{
	if (!cpu_is_mx53())
		ipu_ch_param_set_field(p, IPU_FIELD_ID, 1);
};

struct ipu_rgb {
	struct fb_bitfield	red;
	struct fb_bitfield	green;
	struct fb_bitfield	blue;
	struct fb_bitfield	transp;
	int			bits_per_pixel;
};

void ipu_cpmem_set_format_rgb(struct ipu_ch_param *, struct ipu_rgb *rgb);

static inline void ipu_cpmem_interlaced_scan(struct ipu_ch_param *p, int stride)
{
	ipu_ch_param_set_field(p, IPU_FIELD_SO, 1);
	ipu_ch_param_set_field(p, IPU_FIELD_ILO, stride / 8);
	ipu_ch_param_set_field(p, IPU_FIELD_SLY, (stride * 2) - 1);
};

void ipu_cpmem_set_yuv_interleaved(struct ipu_ch_param *p, u32 pixel_format);
void ipu_cpmem_set_yuv_planar(struct ipu_ch_param *p, u32 pixel_format,
			int stride, int width, int height);

static inline void ipu_cpmem_set_burstsize(struct ipu_ch_param *p, int burstsize)
{
	ipu_ch_param_set_field(p, IPU_FIELD_NPB, burstsize - 1);
};

#endif
