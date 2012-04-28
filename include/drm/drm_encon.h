#ifndef __DRM_ENCON_H
#define __DRM_ENCON_H

struct drm_encoder_connector;

struct drm_encoder_connector_funcs {
	int (*get_modes)(struct drm_encoder_connector *encon);
	int (*mode_valid)(struct drm_encoder_connector *encon,
			struct drm_display_mode *mode);
	void (*mode_set)(struct drm_encoder_connector *encon,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);
	void (*reset)(struct drm_encoder_connector *encon);
	void (*dpms)(struct drm_encoder_connector *encon, int mode);
	enum drm_connector_status (*detect)(struct drm_encoder_connector *encon);
	void (*commit)(struct drm_encoder_connector *encon);
	bool (*mode_fixup)(struct drm_encoder_connector *encon,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode);
	void (*prepare)(struct drm_encoder_connector *encon);
	int (*set_property)(struct drm_encoder_connector *encon,
			struct drm_property *property, uint64_t val);
};

struct drm_encoder_connector {
	struct drm_connector connector;
	struct drm_encoder encoder;
	struct drm_encoder_connector_funcs *funcs;
	struct list_head list;
	int id;
	const char *drm_name;
	int inuse;
};

int drm_encoder_connector_init(struct drm_device *drm,
		struct drm_encoder_connector *c);
void drm_encoder_connector_cleanup(struct drm_device *drm,
		struct drm_encoder_connector *c);
int drm_encon_register(const char *drm_name, int id,
		struct drm_encoder_connector *encon);
int drm_encon_unregister(struct drm_encoder_connector *encon);
struct drm_encoder_connector *drm_encon_get(struct drm_device *drm, int id);
struct drm_encoder_connector *drm_encon_add_dummy(const char *drm_name, int id);
int drm_encon_remove_dummy(struct drm_encoder_connector *encon);

#endif /* __DRM_ENCON_H */
