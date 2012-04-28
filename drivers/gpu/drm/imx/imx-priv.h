struct imx_ipu_encoder {
	struct drm_encoder base;
};

struct imx_ipu_connector {
	struct drm_connector base;
	struct imx_ipu_encoder *encoder;
};

