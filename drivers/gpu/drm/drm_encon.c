/*
 * Implementation of a 1:1 relationship for drm encoders and connectors
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
#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_encon.h>

#define con_to_encon(x) container_of(x, struct drm_encoder_connector, connector)
#define enc_to_encon(x) container_of(x, struct drm_encoder_connector, encoder)

static enum drm_connector_status connector_detect(struct drm_connector *connector,
		bool force)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->detect)
		return encon->funcs->detect(encon);

	return connector_status_connected;
}

static int connector_set_property(struct drm_connector *connector,
		struct drm_property *property, uint64_t val)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->set_property)
		return encon->funcs->set_property(encon, property, val);

	return -EINVAL;
}

static void connector_destroy(struct drm_connector *connector)
{
	/* not here */
}

static int connector_get_modes(struct drm_connector *connector)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs->get_modes)
		return encon->funcs->get_modes(encon);

	return 0;
}

static int connector_mode_valid(struct drm_connector *connector,
			  struct drm_display_mode *mode)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	if (encon->funcs && encon->funcs->mode_valid)
		return encon->funcs->mode_valid(encon, mode);

	return 0;
}

static struct drm_encoder *connector_best_encoder(struct drm_connector *connector)
{
	struct drm_encoder_connector *encon = con_to_encon(connector);

	return &encon->encoder;
}

static void encoder_reset(struct drm_encoder *encoder)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs && encon->funcs->reset)
		encon->funcs->reset(encon);
}

static void encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->dpms)
		encon->funcs->dpms(encon, mode);
}

static bool encoder_mode_fixup(struct drm_encoder *encoder,
			   struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->mode_fixup)
		return encon->funcs->mode_fixup(encon, mode, adjusted_mode);

	return true;
}

static void encoder_prepare(struct drm_encoder *encoder)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->prepare)
		encon->funcs->prepare(encon);
}

static void encoder_commit(struct drm_encoder *encoder)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->commit)
		encon->funcs->commit(encon);
}

static void encoder_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_connector *encon = enc_to_encon(encoder);

	if (encon->funcs->mode_set)
		encon->funcs->mode_set(encon, mode, adjusted_mode);
}

static void encoder_disable(struct drm_encoder *encoder)
{
}

static void encoder_destroy(struct drm_encoder *encoder)
{
	/* not here */
}

struct drm_connector_funcs connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	/* .save unused in drm, only used in nouveau */
	/* .restore unused in drm, only used in nouveau */
	.detect = connector_detect,
	/* .reset called by encoder */
	.set_property = connector_set_property,
	.destroy = connector_destroy,
};

struct drm_connector_helper_funcs connector_helper_funcs = {
	.get_modes = connector_get_modes,
	.mode_valid = connector_mode_valid,
	.best_encoder = connector_best_encoder,
};

static struct drm_encoder_funcs encoder_funcs = {
	.reset = encoder_reset,
	.destroy = encoder_destroy,
};

static struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.dpms = encoder_dpms,
	/* .save unused in drm, only used in nouveau */
	/* .restore unused in drm, only used in nouveau */
	/* .detect unused in drm, only used in nouveau, radeon */
	.mode_fixup = encoder_mode_fixup,
	.prepare = encoder_prepare,
	.commit = encoder_commit,
	.mode_set = encoder_mode_set,
	.disable = encoder_disable,
};

int drm_encoder_connector_init(struct drm_device *drm,
		struct drm_encoder_connector *c)
{
	struct drm_connector *connector = &c->connector;
	struct drm_encoder *encoder = &c->encoder;

	drm_connector_helper_add(connector, &connector_helper_funcs);
	drm_connector_init(drm, &c->connector,
			   &connector_funcs, DRM_MODE_CONNECTOR_VGA);

	drm_encoder_init(drm, encoder, &encoder_funcs, DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &encoder_helper_funcs);

	drm_mode_connector_attach_encoder(connector, encoder);
	drm_sysfs_connector_add(connector);

	return 0;
}
EXPORT_SYMBOL_GPL(drm_encoder_connector_init);

void drm_encoder_connector_cleanup(struct drm_device *drm,
		struct drm_encoder_connector *c)
{
	struct drm_connector *connector = &c->connector;
	struct drm_encoder *encoder = &c->encoder;

	drm_sysfs_connector_remove(connector);
	drm_mode_connector_detach_encoder(connector, encoder);
	drm_encoder_cleanup(encoder);
	drm_connector_cleanup(connector);
	c->inuse = 0;
}
EXPORT_SYMBOL_GPL(drm_encoder_connector_cleanup);

static LIST_HEAD(encon_list);
static DEFINE_MUTEX(encon_list_mutex);

int drm_encon_register(const char *drm_name, int id,
		struct drm_encoder_connector *encon)
{
	encon->drm_name = kstrdup(drm_name, GFP_KERNEL);
	encon->id = id;

	mutex_lock(&encon_list_mutex);
	list_add_tail(&encon->list, &encon_list);
	mutex_unlock(&encon_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(drm_encon_register);

int drm_encon_unregister(struct drm_encoder_connector *encon)
{
	if (encon->inuse)
		return -EBUSY;

	kfree(encon->drm_name);

	mutex_lock(&encon_list_mutex);
	list_del(&encon->list);
	mutex_unlock(&encon_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(drm_encon_unregister);

struct drm_encoder_connector *drm_encon_get(struct drm_device *drm, int id)
{
	const char *drm_name = dev_name(drm->dev);
	struct drm_encoder_connector *encon;

	mutex_lock(&encon_list_mutex);
	list_for_each_entry(encon, &encon_list, list) {
		if (!strcmp(drm_name, encon->drm_name) && encon->id == id) {
			if (encon->inuse)
				goto out;
			encon->inuse = 1;
			mutex_unlock(&encon_list_mutex);
			return encon;
		}
	}
out:
	mutex_unlock(&encon_list_mutex);

	return NULL;
}
EXPORT_SYMBOL_GPL(drm_encon_get);

static struct drm_encoder_connector_funcs dummy_funcs;

/* TODO: allow to pass an array of fixed modes */
struct drm_encoder_connector *drm_encon_add_dummy(const char *drm_name, int id)
{
	struct drm_encoder_connector *encon;
	int ret;

	encon = kzalloc(sizeof(*encon), GFP_KERNEL);
	if (!encon)
		return NULL;

	encon->funcs = &dummy_funcs;
	ret = drm_encon_register(drm_name, id, encon);
	if (ret) {
		kfree(encon);
		return NULL;
	}

	return encon;
}
EXPORT_SYMBOL_GPL(drm_encon_add_dummy);

int drm_encon_remove_dummy(struct drm_encoder_connector *encon)
{
	int ret;

	ret = drm_encon_unregister(encon);
	if (ret)
		return ret;
	kfree(encon);
	return 0;
}
EXPORT_SYMBOL_GPL(drm_encon_remove_dummy);
