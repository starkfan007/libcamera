/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * aic.cpp - Intel IA Imaging library C++ wrapper
 *
 * Automatic IPU Configuration
 */

#include <ia_imaging/ia_cmc_parser.h>

#include "aic.h"

#include "libcamera/internal/log.h"

#include "binary_data.h"
#include "parameter_encoder.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define ALIGN128(x) (((x) + 127) & ~127)

namespace libcamera {

LOG_DEFINE_CATEGORY(AIC)

namespace ipa::ipu3::aic {

/*
 * Only a Single Pipeline instance of the AIC is currently supported.
 * The CrOS implementation defines a set of AIC to run for both STILL and VIDEO
 * allowing improved perfomance on preview streams while taking an image
 * capture.
 */

AIC::~AIC()
{
	if (iaCmc_)
		ia_cmc_parser_deinit(iaCmc_);
}

int AIC::init(BinaryData &aiqb)
{
	LOG(AIC, Debug) << "Initialising IA AIC Wrapper";

	pipe_ = std::make_unique<IPU3ISPPipe>();

	CLEAR(mRuntimeParamsOutFrameParams_);
	CLEAR(mRuntimeParamsResCfgParams_);
	CLEAR(mRuntimeParamsInFrameParams_);
	CLEAR(mRuntimeParamsRec_);
	CLEAR(mRuntimeParams_);

	mRuntimeParams_.output_frame_params = &mRuntimeParamsOutFrameParams_;
	mRuntimeParams_.frame_resolution_parameters = &mRuntimeParamsResCfgParams_;
	mRuntimeParams_.input_frame_params = &mRuntimeParamsInFrameParams_;
	mRuntimeParams_.focus_rect = &mRuntimeParamsRec_;

	/*
	 * \todo: Both the AIC and the AIQ use the iaCmc_.
	 * Can this be the same instance or do they need their own instances?
	 */
	iaCmc_ = ia_cmc_parser_init(aiqb.data());
	if (iaCmc_ == nullptr) {
		LOG(AIC, Error) << "Failed to initialise CMC Parser";
		return -EINVAL;
	}

	/* \todo: Initialise the mRuntimeParams with ia_aiq_frame_params before
	 * constructing the KBL_AIC.
	 * In CrOS, GraphConfig::getSensorFrameParams provides all these
	 * details. Start looking from ParameterWorker::configure()
	 */
	ISPPipe *pipe = static_cast<ISPPipe *>(pipe_.get());
	skyCam_ = std::make_unique<KBL_AIC>(&pipe, 1, iaCmc_, aiqb.data(),
					    mRuntimeParams_, 0, 0);

	return 0;
}

int AIC::configure(const struct IPAConfigInfo &configInfo)
{
	LOG(AIC, Debug) << "IA AIC configure(): "
			<< " bds: " << configInfo.bdsOutputSize.width << "x" << configInfo.bdsOutputSize.height
			<< " ifSize: " << configInfo.iif.width << "x" << configInfo.iif.height
			<< " gdcSize: " << configInfo.gdcSize.width << "x" << configInfo.gdcSize.height
			<< " cropRegion: " << configInfo.cropRegion.width << "x" << configInfo.cropRegion.height;

	//Fill AIC input frame params
	mRuntimeParams_.frame_use = ia_aiq_frame_use_still;
	mRuntimeParams_.mode_index = AIC_MODE_STILL;
	mRuntimeParamsInFrameParams_.sensor_frame_params.horizontal_crop_offset = 0;
	mRuntimeParamsInFrameParams_.sensor_frame_params.vertical_crop_offset = 0;
	mRuntimeParamsInFrameParams_.sensor_frame_params.cropped_image_width = configInfo.cropRegion.width;
	mRuntimeParamsInFrameParams_.sensor_frame_params.cropped_image_height = configInfo.cropRegion.height;
	mRuntimeParamsInFrameParams_.sensor_frame_params.horizontal_scaling_numerator = 1;
	mRuntimeParamsInFrameParams_.sensor_frame_params.horizontal_scaling_denominator = 1;
	mRuntimeParamsInFrameParams_.sensor_frame_params.vertical_scaling_numerator = 1;
	mRuntimeParamsInFrameParams_.sensor_frame_params.vertical_scaling_denominator = 1;
	mRuntimeParamsInFrameParams_.fix_flip_x = 0;
	mRuntimeParamsInFrameParams_.fix_flip_y = 0;

	mRuntimeParamsOutFrameParams_.width = configInfo.cropRegion.width;
	mRuntimeParamsOutFrameParams_.height = configInfo.cropRegion.height;

	mRuntimeParamsResCfgParams_.BDSin_img_width = configInfo.iif.width;
	mRuntimeParamsResCfgParams_.BDSin_img_height = configInfo.iif.height;
	mRuntimeParamsResCfgParams_.BDSout_img_width = configInfo.bdsOutputSize.width;
	mRuntimeParamsResCfgParams_.BDSout_img_height = configInfo.bdsOutputSize.height;

	mRuntimeParamsResCfgParams_.horizontal_IF_crop = configInfo.bdsOutputSize.width;
	mRuntimeParamsResCfgParams_.vertical_IF_crop = configInfo.bdsOutputSize.height;
	mRuntimeParamsResCfgParams_.BDS_horizontal_padding =
		static_cast<uint16_t>(ALIGN128(configInfo.bdsOutputSize.width) - configInfo.bdsOutputSize.width);

	return 0;
}

void AIC::reset()
{
}

int AIC::run(ipu3_uapi_params *params)
{
	LOG(AIC, Debug) << "IA AIC Run()";
	skyCam_->Run(&mRuntimeParams_, 1);

	/* IPU3 firmware specific encoding for ISP controls. */
	ParameterEncoder::encode(GetAicConfig(), params);

	return 0;
}

std::string AIC::version()
{
	return "";
}

aic_config_t *AIC::GetAicConfig()
{
	pipe_->dump();
	return pipe_->GetAicConfig();
}

void AIC::updateRuntimeParams(aiq::AiqResults &results)
{
	mRuntimeParams_.pa_results = results.pa();
	mRuntimeParams_.sa_results = results.sa();

	const ia_aiq_ae_results *ae = results.ae();
	mRuntimeParams_.exposure_results = ae->exposures->exposure;
	mRuntimeParams_.weight_grid = ae->weight_grid;

	mRuntimeParams_.isp_vamem_type = 0;
	mRuntimeParams_.awb_results = results.awb();
	mRuntimeParams_.gbce_results = results.gbce();

	/* \todo: Set below parameters from capture settings
	params->time_stamp = 0; //microsecond unit
	params->manual_brightness = settings->ispSettings.manualSettings.manualBrightness;
	params->manual_contrast = settings->ispSettings.manualSettings.manualContrast;
	params->manual_hue = settings->ispSettings.manualSettings.manualHue;
	params->manual_saturation = settings->ispSettings.manualSettings.manualSaturation;
	params->manual_sharpness = settings->ispSettings.manualSettings.manualSharpness;
	*/
}

} /* namespace ipa::ipu3::aic */

} /* namespace libcamera */
