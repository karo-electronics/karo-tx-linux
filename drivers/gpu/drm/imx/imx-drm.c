/*
 * i.MX IPUv3 Graphics driver
 *
 * Copyright (C) 2011 Sascha Hauer, Pengutronix
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <drm/imx-ipu-v3.h>
#include <asm/fb.h>
#include <drm/drm_encon.h>

#define DRIVER_NAME		"i.MX"
#define DRIVER_DESC		"i.MX IPUv3 Graphics"
#define DRIVER_DATE		"20110604"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

struct ipu_resource {
	int ipu_channel_bg;
	int dc_channel;
	int dp_channel;
	int display;
	u32 interface_pix_fmt; /* FIXME: move to platform data */
};

static struct ipu_resource ipu_resources[] = {
	{
		.ipu_channel_bg = 23, /* IPUV3_CHANNEL_MEM_BG_SYNC */
		.dc_channel = 5,
		.dp_channel = IPU_DP_FLOW_SYNC_BG,
		.display = 0,
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	} , {
		.ipu_channel_bg = 28, /* IPUV3_CHANNEL_MEM_DC_SYNC */
		.dc_channel = 1,
		.dp_channel = -1,
		.display = 1,
		.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	},
};

struct ipu_crtc {
	struct drm_crtc		base;
	int			pipe;
	struct ipu_resource	*ipu_res;
	struct ipu_channel	*ipu_ch;
	struct ipu_dc		*dc;
	struct ipu_dp		*dp;
	struct dmfc_channel	*dmfc;
	struct ipu_di		*di;
	int			di_no;
	struct clk		*pixclk;
	int			enabled;
};

struct ipu_framebuffer {
	struct drm_framebuffer	base;
	void			*virt;
	dma_addr_t		phys;
	size_t			len;
};

struct ipu_drm_private {
	struct ipu_crtc		crtc[2];
	struct drm_encoder_connector *encon[2];
	struct drm_fb_helper	fb_helper;
	struct ipu_framebuffer	ifb;
	int			num_crtcs;
};

static struct fb_ops ipu_ipufb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_pan_display = drm_fb_helper_pan_display,
	.fb_blank = drm_fb_helper_blank,
	.fb_setcmap = drm_fb_helper_setcmap,
	.fb_debug_enter = drm_fb_helper_debug_enter,
	.fb_debug_leave = drm_fb_helper_debug_leave,
};

#define to_ipu_framebuffer(x) container_of(x, struct ipu_framebuffer, base)
#define to_ipu_crtc(x) container_of(x, struct ipu_crtc, base)

static void ipu_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct ipu_framebuffer *ipu_fb = to_ipu_framebuffer(fb);
	struct drm_device *drm = fb->dev;

	dev_info(drm->dev, "%s: %p freeing fb %p [%08x]\n", __func__,
		ipu_fb, ipu_fb->virt, ipu_fb->phys);
	dma_free_writecombine(drm->dev, ipu_fb->len, ipu_fb->virt, ipu_fb->phys);

	drm_framebuffer_cleanup(fb);

	kfree(ipu_fb);
}

static int ipu_user_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	/* We do not use the handle atm */
	*handle = 77;

	return 0;
}

static const struct drm_framebuffer_funcs ipu_fb_funcs = {
	.destroy = ipu_user_framebuffer_destroy,
	.create_handle = ipu_user_framebuffer_create_handle,
};

static struct ipu_rgb def_rgb_32 = {
	.red	= { .offset = 16, .length = 8, },
	.green	= { .offset =  8, .length = 8, },
	.blue	= { .offset =  0, .length = 8, },
	.transp = { .offset = 24, .length = 8, },
	.bits_per_pixel = 32,
};

static int calc_vref(struct drm_display_mode *mode)
{
	unsigned long htotal, vtotal;

	htotal = mode->htotal;
	vtotal = mode->vtotal;

	if (!htotal || !vtotal)
		return 60;

	return mode->clock * 1000 / vtotal / htotal;
}

static int calc_bandwidth(struct drm_display_mode *mode, unsigned int vref)
{
	return mode->hdisplay * mode->vdisplay * vref;
}

static void ipu_fb_enable(struct ipu_crtc *ipu_crtc)
{
	if (ipu_crtc->enabled)
		return;

	ipu_di_enable(ipu_crtc->di);
	ipu_dmfc_enable_channel(ipu_crtc->dmfc);
	ipu_idmac_enable_channel(ipu_crtc->ipu_ch);
	ipu_dc_enable_channel(ipu_crtc->dc);
	if (ipu_crtc->dp)
		ipu_dp_enable_channel(ipu_crtc->dp);

	ipu_crtc->enabled = 1;
}

static void ipu_fb_disable(struct ipu_crtc *ipu_crtc)
{
	if (!ipu_crtc->enabled)
		return;

	if (ipu_crtc->dp)
		ipu_dp_disable_channel(ipu_crtc->dp);
	ipu_dc_disable_channel(ipu_crtc->dc);
	ipu_idmac_disable_channel(ipu_crtc->ipu_ch);
	ipu_dmfc_disable_channel(ipu_crtc->dmfc);
	ipu_di_disable(ipu_crtc->di);

	ipu_crtc->enabled = 0;
}

static int ipu_fb_set_par(struct drm_crtc *crtc,
		struct drm_display_mode *mode,
		unsigned long phys)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	struct drm_device *drm = crtc->dev;
	struct ipu_soc *ipu = dev_get_drvdata(drm->dev->parent);
	struct drm_framebuffer *fb = crtc->fb;
	int ret;
	struct ipu_di_signal_cfg sig_cfg;
	u32 out_pixel_fmt;
	struct ipu_ch_param *cpmem = ipu_get_cpmem(ipu_crtc->ipu_ch);

	ipu_fb_disable(ipu_crtc);

	memset(cpmem, 0, sizeof(*cpmem));

	memset(&sig_cfg, 0, sizeof(sig_cfg));
	out_pixel_fmt = ipu_crtc->ipu_res->interface_pix_fmt;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		sig_cfg.interlaced = 1;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		sig_cfg.Hsync_pol = 1;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		sig_cfg.Vsync_pol = 1;
	sig_cfg.enable_pol = 1;

	sig_cfg.width = mode->hdisplay;
	sig_cfg.height = mode->vdisplay;
	sig_cfg.pixel_fmt = out_pixel_fmt;
	sig_cfg.h_start_width = mode->htotal - mode->hsync_end;
	sig_cfg.h_sync_width = mode->hsync_end - mode->hsync_start;
	sig_cfg.h_end_width = mode->hsync_start - mode->hdisplay;
	sig_cfg.v_start_width = mode->vtotal - mode->vsync_end;
	sig_cfg.v_sync_width = mode->vsync_end - mode->vsync_start;
	sig_cfg.v_end_width = mode->vsync_start - mode->vdisplay;
	sig_cfg.v_to_h_sync = 0;

	ipu_get(ipu);
	clk_set_rate(ipu_crtc->pixclk, mode->clock * 1000);
	ipu_put(ipu);

	if (ipu_crtc->dp) {
		ret = ipu_dp_setup_channel(ipu_crtc->dp, IPU_COLORSPACE_RGB,
				IPU_COLORSPACE_RGB);
		if (ret) {
			dev_err(drm->dev, "initializing display processor failed with %d\n",
				ret);
			return ret;
		}
		ipu_dp_set_global_alpha(ipu_crtc->dp, 1, 0, 1);
	}

	ret = ipu_dc_init_sync(ipu_crtc->dc, ipu_crtc->di_no, sig_cfg.interlaced,
			out_pixel_fmt, mode->hdisplay);
	if (ret) {
		dev_err(drm->dev, "initializing display controller failed with %d\n",
			ret);
		return ret;
	}

	ret = ipu_di_init_sync_panel(ipu_crtc->di, &sig_cfg);
	if (ret) {
		dev_err(drm->dev, "initializing panel failed with %d\n", ret);
		return ret;
	}

	ipu_cpmem_set_resolution(cpmem, mode->hdisplay, mode->vdisplay);
	ipu_cpmem_set_stride(cpmem, fb->pitches[0]);
	ipu_cpmem_set_buffer(cpmem, 0, phys);
	ipu_cpmem_set_format_rgb(cpmem, &def_rgb_32);
	ipu_cpmem_set_high_priority(cpmem);

	ret = ipu_dmfc_init_channel(ipu_crtc->dmfc, mode->hdisplay);
	if (ret) {
		dev_err(drm->dev, "initializing dmfc channel failed with %d\n",
			ret);
		return ret;
	}

	ret = ipu_dmfc_alloc_bandwidth(ipu_crtc->dmfc,
			calc_bandwidth(mode, calc_vref(mode)), 8);
	if (ret) {
		dev_err(drm->dev, "allocating dmfc bandwidth failed with %d\n",
			ret);
		return ret;
	}

	ipu_fb_enable(ipu_crtc);

	return ret;
}

int ipu_framebuffer_init(struct drm_device *drm,
			   struct ipu_framebuffer *ipu_fb,
			   struct drm_mode_fb_cmd2 *mode_cmd)
{
	int ret;

	if (mode_cmd->pitches[0] & 63)
		return -EINVAL;

	switch (mode_cmd->pixel_format) {
	case DRM_FORMAT_RGB332:
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_ARGB2101010:
		break;

	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_YVYU:
	case DRM_FORMAT_VYUY:
		break;

	default:
		DRM_ERROR("unsupported pixel format: %c%c%c%c\n",
			mode_cmd->pixel_format & 0xff,
			(mode_cmd->pixel_format >> 8) & 0xff,
			(mode_cmd->pixel_format >> 16) & 0xff,
			(mode_cmd->pixel_format >> 24));
		return -EINVAL;
	}

	ret = drm_framebuffer_init(drm, &ipu_fb->base, &ipu_fb_funcs);
	if (ret) {
		DRM_ERROR("framebuffer init failed %d\n", ret);
		return ret;
	}

	drm_helper_mode_fill_fb_struct(&ipu_fb->base, mode_cmd);

	return 0;
}

static int ipu_ipufb_create(struct drm_fb_helper *helper,
			  struct drm_fb_helper_surface_size *sizes)
{
	struct drm_device *drm = helper->dev;
	struct ipu_drm_private *priv = drm->dev_private;
	struct fb_info *info;
	struct drm_framebuffer *fb;
	struct ipu_framebuffer *ipu_fb;
	struct drm_mode_fb_cmd2 mode_cmd;
	int size, ret;
	u32 bpp = sizes->surface_bpp;
	u32 depth = sizes->surface_depth;

	/* we don't do packed 24bpp */
	if (bpp == 24)
		bpp = 32;

	mode_cmd.width = sizes->surface_width;
	mode_cmd.height = sizes->surface_height;

	mode_cmd.pitches[0] = ALIGN(mode_cmd.width * ((bpp + 7) / 8), 64);

	size = mode_cmd.pitches[0] * mode_cmd.height;
	size = ALIGN(size, PAGE_SIZE);

	mutex_lock(&drm->struct_mutex);

	info = framebuffer_alloc(0, drm->dev);
	if (!info) {
		dev_err(drm->dev, "Unable to allocate framebuffer struct\n");
		ret = -ENOMEM;
		goto out_unpin;
	}

	info->par = helper;

	mode_cmd.pixel_format = drm_mode_legacy_fb_format(bpp, depth);

	ret = ipu_framebuffer_init(drm, &priv->ifb, &mode_cmd);
	if (ret)
		goto out_unpin;

	ipu_fb = &priv->ifb;
	fb = &ipu_fb->base;

	priv->fb_helper.fb = fb;
	priv->fb_helper.fbdev = info;

	strcpy(info->fix.id, "imx_ipudrmfb");

	info->flags = FBINFO_DEFAULT | FBINFO_CAN_FORCE_OUTPUT;
	info->fbops = &ipu_ipufb_ops;

	info->screen_base = dma_alloc_writecombine(drm->dev,
				size,
				(dma_addr_t *)&info->fix.smem_start,
				GFP_DMA);
	if (!info->screen_base) {
		dev_err(drm->dev, "Unable to allocate framebuffer memory (%d)\n",
				size);
		ret = -ENOMEM;
		goto out_unpin;
	}

	memset(info->screen_base, 0x80, size);

	ipu_fb->virt = info->screen_base;
	ipu_fb->phys = info->fix.smem_start;
	ipu_fb->len = size;
	dev_info(drm->dev, "%s: %p allocated fb %p [%08x]\n", __func__,
		ipu_fb, ipu_fb->virt, ipu_fb->phys);

	info->fix.smem_len = size;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		dev_err(drm->dev, "Unable to allocate color map\n");
		ret = -ENOMEM;
		goto out_unpin;
	}
	info->screen_size = size;

	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(info, &priv->fb_helper, sizes->fb_width, sizes->fb_height);

	info->pixmap.size = 64*1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;

	DRM_DEBUG_KMS("allocated %dx%d\n", fb->width, fb->height);

out_unpin:
	mutex_unlock(&drm->struct_mutex);

	return ret;
}

static void ipu_ipufb_destroy(struct drm_device *drm)
{
	struct ipu_drm_private *priv = drm->dev_private;
	struct ipu_framebuffer *ipu_fb = &priv->ifb;
	struct fb_info *info = priv->fb_helper.fbdev;

	unregister_framebuffer(info);
	if (info->cmap.len)
		fb_dealloc_cmap(&info->cmap);
	framebuffer_release(info);

	drm_fb_helper_fini(&priv->fb_helper);

	drm_framebuffer_cleanup(&ipu_fb->base);

	dev_info(drm->dev, "%s: %p freeing fb %p [%08x]\n", __func__,
		ipu_fb, ipu_fb->virt, ipu_fb->phys);
	dma_free_writecombine(drm->dev, ipu_fb->len, ipu_fb->virt, ipu_fb->phys);
}

static int ipu_fb_find_or_create_single(struct drm_fb_helper *helper,
		struct drm_fb_helper_surface_size *sizes)
{
	int new_fb = 0;
	int ret;

	if (!helper->fb) {
		ret = ipu_ipufb_create(helper, sizes);
		if (ret)
			return ret;
		new_fb = 1;
	}

	return new_fb;
}

static void ipu_crtc_fb_gamma_set(struct drm_crtc *crtc, u16 red, u16 green,
				 u16 blue, int regno)
{
}

static void ipu_crtc_fb_gamma_get(struct drm_crtc *crtc, u16 *red, u16 *green,
			     u16 *blue, int regno)
{
}

static void ipu_crtc_load_lut(struct drm_crtc *crtc)
{
}

static struct drm_fb_helper_funcs ipu_fb_helper_funcs = {
	.gamma_set = ipu_crtc_fb_gamma_set,
	.gamma_get = ipu_crtc_fb_gamma_get,
	.fb_probe = ipu_fb_find_or_create_single,
};

static struct drm_framebuffer *
ipu_user_framebuffer_create(struct drm_device *drm,
			      struct drm_file *filp,
			      struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct ipu_framebuffer *ipu_fb;
	int ret;
	u32 bpp, depth;

	ipu_fb = kzalloc(sizeof(*ipu_fb), GFP_KERNEL);
	if (!ipu_fb)
		return ERR_PTR(-ENOMEM);

	ret = ipu_framebuffer_init(drm, ipu_fb, mode_cmd);
	if (ret) {
		kfree(ipu_fb);
		return ERR_PTR(ret);
	}

	drm_fb_get_bpp_depth(mode_cmd->pixel_format, &depth, &bpp);

	ipu_fb->len = mode_cmd->width * mode_cmd->height * (bpp >> 3);
	ipu_fb->virt = dma_alloc_writecombine(drm->dev,
				ipu_fb->len,
				(dma_addr_t *)&ipu_fb->phys,
				GFP_DMA);
	if (!ipu_fb->virt) {
		kfree(ipu_fb);
		return ERR_PTR(-ENOMEM);
	}

	dev_info(drm->dev, "%s: %p allocated fb %p [%08x]\n", __func__,
		ipu_fb, ipu_fb->virt, ipu_fb->phys);
	return &ipu_fb->base;
}

void ipu_fb_output_poll_changed(struct drm_device *dev)
{
}

static const struct drm_mode_config_funcs ipu_mode_funcs = {
	.fb_create = ipu_user_framebuffer_create,
	.output_poll_changed = ipu_fb_output_poll_changed,
};

static void ipu_crtc_disable(struct drm_crtc *crtc)
{
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	ipu_fb_disable(ipu_crtc);
}

static int ipu_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	struct drm_framebuffer *fb = crtc->fb;
	struct ipu_framebuffer *ipu_fb;
	unsigned long phys;

	ipu_fb = to_ipu_framebuffer(fb);

	phys = ipu_fb->phys;
	phys += x * 4; /* FIXME */
	phys += y * fb->pitches[0];

	ipu_fb_set_par(crtc, mode, phys);

	return 0;
}

static int ipu_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				struct drm_framebuffer *old_fb)
{
	struct drm_framebuffer *fb = crtc->fb;
	struct ipu_framebuffer *ipu_fb;
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);
	unsigned long phys;

	ipu_fb = to_ipu_framebuffer(fb);

	phys = ipu_fb->phys;
	phys += x * 4; /* FIXME */
	phys += y * fb->pitches[0];

	ipu_cpmem_set_stride(ipu_get_cpmem(ipu_crtc->ipu_ch), fb->pitches[0]);
	ipu_cpmem_set_buffer(ipu_get_cpmem(ipu_crtc->ipu_ch),
			  0, phys);
	return 0;
}

static void ipu_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *drm = crtc->dev;
	struct device *dev = drm->dev;
	struct ipu_crtc *ipu_crtc = to_ipu_crtc(crtc);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		ipu_fb_enable(ipu_crtc);
		break;

	case DRM_MODE_DPMS_OFF:
		ipu_fb_disable(ipu_crtc);
		break;

	default:
		dev_warn(dev, "Unsupported DPMS mode %d\n", mode);
	}
}

static bool ipu_crtc_mode_fixup(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void ipu_crtc_prepare(struct drm_crtc *crtc)
{
}

static void ipu_crtc_commit(struct drm_crtc *crtc)
{
}

static struct drm_crtc_helper_funcs ipu_helper_funcs = {
	.dpms = ipu_crtc_dpms,
	.mode_fixup = ipu_crtc_mode_fixup,
	.mode_set = ipu_crtc_mode_set,
	.mode_set_base = ipu_crtc_mode_set_base,
	.prepare = ipu_crtc_prepare,
	.commit = ipu_crtc_commit,
	.load_lut = ipu_crtc_load_lut,
	.disable = ipu_crtc_disable,
};

static int ipu_page_flip(struct drm_crtc *crtc,
			struct drm_framebuffer *fb,
			struct drm_pending_vblank_event *event)
{
	printk("%s\n", __func__);
	dump_stack();
	return 0;
}

static const struct drm_crtc_funcs ipu_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = ipu_page_flip,
};

static void ipu_put_resources(struct drm_device *drm, struct ipu_crtc *ipu_crtc)
{
	if (!IS_ERR(ipu_crtc->pixclk))
		clk_put(ipu_crtc->pixclk);
	if (!IS_ERR_OR_NULL(ipu_crtc->ipu_ch))
		ipu_idmac_put(ipu_crtc->ipu_ch);
	if (!IS_ERR_OR_NULL(ipu_crtc->dmfc))
		ipu_dmfc_put(ipu_crtc->dmfc);
	if (!IS_ERR_OR_NULL(ipu_crtc->dp))
		ipu_dp_put(ipu_crtc->dp);
	if (!IS_ERR_OR_NULL(ipu_crtc->di))
		ipu_di_put(ipu_crtc->di);
	if (!IS_ERR_OR_NULL(ipu_crtc->dc))
		ipu_dc_put(ipu_crtc->dc);
}

static int ipu_get_resources(struct drm_device *drm, struct ipu_crtc *ipu_crtc)
{
	struct ipu_soc *ipu = dev_get_drvdata(drm->dev->parent);
	struct ipu_resource *res = &ipu_resources[ipu_crtc->pipe];
	int ret;
	ipu_crtc->ipu_res = res;

	if (ipu_crtc->pipe == 0)
		ipu_crtc->pixclk = clk_get(drm->dev, "pixclock0");
	else
		ipu_crtc->pixclk = clk_get(drm->dev, "pixclock1");
	if (IS_ERR(ipu_crtc->pixclk)) {
		ret = PTR_ERR(ipu_crtc->pixclk);
		dev_err(drm->dev, "Could not get pixel clock[%d]: %d\n",
			ipu_crtc->pipe, ret);
		goto err_out;
	}

	ipu_crtc->ipu_ch = ipu_idmac_get(ipu, res->ipu_channel_bg);
	if (IS_ERR_OR_NULL(ipu_crtc->ipu_ch)) {
		ret = PTR_ERR(ipu_crtc->ipu_ch);
		dev_err(drm->dev, "Could not get DMA channel %d: %d\n",
			res->ipu_channel_bg, ret);
		goto err_out;
	}

	ipu_crtc->dc = ipu_dc_get(ipu, res->dc_channel);
	if (IS_ERR(ipu_crtc->dc)) {
		ret = PTR_ERR(ipu_crtc->dc);
		dev_err(drm->dev, "Could not get DC channel %d: %d\n",
			res->dc_channel, ret);
		goto err_out;
	}

	ipu_crtc->dmfc = ipu_dmfc_get(ipu, res->ipu_channel_bg);
	if (IS_ERR(ipu_crtc->dmfc)) {
		ret = PTR_ERR(ipu_crtc->dmfc);
		dev_err(drm->dev, "Could not get DMFC channel %d: %d\n",
			res->ipu_channel_bg, ret);
		goto err_out;
	}

	if (res->dp_channel >= 0) {
		ipu_crtc->dp = ipu_dp_get(ipu, res->dp_channel);
		if (IS_ERR(ipu_crtc->dp)) {
			ret = PTR_ERR(ipu_crtc->ipu_ch);
			dev_err(drm->dev, "Could not get DP channel %d: %d\n",
				res->dc_channel, ret);
			goto err_out;
		}
	}

	ipu_crtc->di = ipu_di_get(ipu, res->display);
	if (IS_ERR(ipu_crtc->di)) {
		ret = PTR_ERR(ipu_crtc->di);
		dev_err(drm->dev, "Could not get DI channel %d: %d\n",
			res->display, ret);
		goto err_out;
	}

	return 0;

err_out:
	ipu_put_resources(drm, ipu_crtc);
	return ret;
}

static int ipu_crtc_init(struct drm_device *drm, int pipe)
{
	struct ipu_drm_private *priv = drm->dev_private;
	struct ipu_crtc *ipu_crtc = &priv->crtc[pipe];
	int ret;

	ipu_crtc->pipe = pipe;
	ipu_crtc->di_no = pipe;

	ret = ipu_get_resources(drm, ipu_crtc);
	if (ret)
		return ret;

	drm_crtc_init(drm, &ipu_crtc->base, &ipu_crtc_funcs);
	drm_mode_crtc_set_gamma_size(&ipu_crtc->base, 256);
	drm_crtc_helper_add(&ipu_crtc->base, &ipu_helper_funcs);

	return 0;
}

static void ipu_crtc_cleanup(struct drm_device *drm, int pipe)
{
	struct ipu_drm_private *priv = drm->dev_private;
	struct ipu_crtc *ipu_crtc = &priv->crtc[pipe];

	DRM_DEBUG_KMS("%s: priv=%p crtc[%d]=%p base=%p\n", __func__,
		priv, pipe, ipu_crtc, &ipu_crtc->base);

	drm_crtc_cleanup(&ipu_crtc->base);
	ipu_put_resources(drm, ipu_crtc);
}

static int ipu_fbdev_init(struct drm_device *drm)
{
	struct ipu_drm_private *priv = drm->dev_private;
	struct drm_encoder_connector *encon;
	int i, ret;

	drm_mode_config_init(drm);

	drm->mode_config.min_width = 0;
	drm->mode_config.min_height = 0;

	drm->mode_config.funcs = (void *)&ipu_mode_funcs;

	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 4096;

	drm->mode_config.fb_base = 0xdeadbeef;

	for (i = 0; i < 2; i++) {
		ret = ipu_crtc_init(drm, i);
		if (ret)
			goto out;
		priv->encon[i] = drm_encon_get(drm, i);
		encon = priv->encon[i];
		if (!encon)
			continue;
		encon->encoder.possible_crtcs = 1 << i;
		drm_encoder_connector_init(drm, encon);
		priv->num_crtcs++;
	}

	priv->fb_helper.funcs = &ipu_fb_helper_funcs;

	ret = drm_fb_helper_init(drm, &priv->fb_helper, priv->num_crtcs,
			priv->num_crtcs);
	if (ret)
		goto out;

	drm_fb_helper_single_add_all_connectors(&priv->fb_helper);
	drm_fb_helper_initial_config(&priv->fb_helper, 32);

#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	drm_fb_helper_restore_fbdev_mode(&priv->fb_helper);
#endif
	return 0;
out:
	while (--i >= 0) {
		ipu_crtc_cleanup(drm, i);
		if (priv->encon[i])
			drm_encoder_connector_cleanup(drm, priv->encon[i]);
	};

	return ret;
}

static int ipu_mmap(struct file *filp, struct vm_area_struct * vma)
{
	struct drm_file *priv = filp->private_data;
	struct drm_device *drm = priv->minor->dev;
	struct drm_mode_object *obj;
	struct drm_framebuffer *fb;
	struct ipu_framebuffer *ipu_fb;
	unsigned long off;
	unsigned long start;
	u32 len;

	obj = drm_mode_object_find(drm, vma->vm_pgoff, DRM_MODE_OBJECT_FB);
	if (!obj) {
		dev_err(drm->dev, "could not find object %ld\n", vma->vm_pgoff);
		return -ENOENT;
	}

	fb = obj_to_fb(obj);
	ipu_fb = to_ipu_framebuffer(fb);

	off = 0;

	start = ipu_fb->phys;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + ipu_fb->len);
	start &= PAGE_MASK;

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	fb_pgprotect(filp, vma, off);
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			     vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static int ipu_driver_load(struct drm_device *drm, unsigned long flags)
{
	struct ipu_drm_private *priv;
	int ret;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm->dev_private = priv;

	ret = ipu_fbdev_init(drm);
	if (ret)
		goto out;

	drm_kms_helper_poll_init(drm);

	return 0;

out:
	kfree(priv);

	return ret;
}

static int ipu_driver_open(struct drm_device *drm, struct drm_file *file)
{
	return 0;
}

static void ipu_driver_lastclose(struct drm_device *drm)
{
	struct ipu_drm_private *priv = drm->dev_private;

	drm_fb_helper_restore_fbdev_mode(&priv->fb_helper);
}

static int ipu_driver_unload(struct drm_device *drm)
{
	struct ipu_drm_private *priv = drm->dev_private;
	int i;

	ipu_ipufb_destroy(drm);

	for (i = 0; i < 2; i++) {
		ipu_crtc_cleanup(drm, i);

		if (priv->encon[i])
			drm_encoder_connector_cleanup(drm, priv->encon[i]);
	}

	kfree(priv);

	return 0;
}

static int ipu_suspend(struct drm_device *drm, pm_message_t state)
{
	return 0;
}

static int ipu_resume(struct drm_device *drm)
{
	return 0;
}

static int ipu_enable_vblank(struct drm_device *drm, int crtc)
{
	return 0;
}

static void ipu_disable_vblank(struct drm_device *drm, int crtc)
{
}

struct drm_ioctl_desc ipu_ioctls[] = {
};

static const struct file_operations imx_drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = ipu_mmap,
	.poll = drm_poll,
	.fasync = drm_fasync,
	.read = drm_read,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ipu_compat_ioctl,
#endif
	.llseek = noop_llseek,
};

static struct drm_driver driver = {
	.driver_features = DRIVER_MODESET,
	.load = ipu_driver_load,
	.unload = ipu_driver_unload,
	.open = ipu_driver_open,
	.lastclose = ipu_driver_lastclose,

	/* Used in place of ipu_pm_ops for non-DRIVER_MODESET */
	.suspend = ipu_suspend,
	.resume = ipu_resume,

	.enable_vblank = ipu_enable_vblank,
	.disable_vblank = ipu_disable_vblank,
	.reclaim_buffers = drm_core_reclaim_buffers,
	.ioctls = ipu_ioctls,
	.num_ioctls = ARRAY_SIZE(ipu_ioctls),
	.fops = &imx_drm_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static int __devinit ipu_drm_probe(struct platform_device *pdev)
{
	int ret;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	ret = drm_platform_init(&driver, pdev);
	return ret;
}

static int __devexit ipu_drm_remove(struct platform_device *pdev)
{
	drm_platform_exit(&driver, pdev);

	return 0;
}

static struct platform_driver ipu_drm_driver = {
	.driver = {
		.name = "imx-drm",
	},
	.probe = ipu_drm_probe,
	.remove = __devexit_p(ipu_drm_remove),
};

int __init ipu_drm_init(void)
{
	int ret;
	ret = platform_driver_register(&ipu_drm_driver);
	return ret;
}

void __exit ipu_drm_exit(void)
{
	platform_driver_unregister(&ipu_drm_driver);
}

late_initcall(ipu_drm_init);
module_exit(ipu_drm_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
