/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Jaslo Ziska
 *
 * GStreamer Camera Controls
 *
 * This file is auto-generated. Do not edit.
 */

#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "gstlibcamera-controls.h"
#include "gstlibcamera-utils.h"

using namespace libcamera;


static const GEnumValue ae_metering_mode_types[] = {
	{
		controls::MeteringCentreWeighted,
		"Centre-weighted metering mode. ",
		"centre-weighted"
	},
	{
		controls::MeteringSpot,
		"Spot metering mode. ",
		"spot"
	},
	{
		controls::MeteringMatrix,
		"Matrix metering mode. ",
		"matrix"
	},
	{
		controls::MeteringCustom,
		"Custom metering mode. ",
		"custom"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AE_METERING_MODE \
	(ae_metering_mode_get_type())
static GType ae_metering_mode_get_type()
{
	static GType ae_metering_mode_type = 0;

	if (!ae_metering_mode_type)
		ae_metering_mode_type =
			g_enum_register_static("AeMeteringMode",
					       ae_metering_mode_types);

	return ae_metering_mode_type;
}

static const GEnumValue ae_constraint_mode_types[] = {
	{
		controls::ConstraintNormal,
		"Default constraint mode. "
		"This mode aims to balance the exposure of different parts of the "
		"image so as to reach a reasonable average level. However, highlights "
		"in the image may appear over-exposed and lowlights may appear "
		"under-exposed. ",
		"normal"
	},
	{
		controls::ConstraintHighlight,
		"Highlight constraint mode. "
		"This mode adjusts the exposure levels in order to try and avoid "
		"over-exposing the brightest parts (highlights) of an image. "
		"Other non-highlight parts of the image may appear under-exposed. ",
		"highlight"
	},
	{
		controls::ConstraintShadows,
		"Shadows constraint mode. "
		"This mode adjusts the exposure levels in order to try and avoid "
		"under-exposing the dark parts (shadows) of an image. Other normally "
		"exposed parts of the image may appear over-exposed. ",
		"shadows"
	},
	{
		controls::ConstraintCustom,
		"Custom constraint mode. ",
		"custom"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AE_CONSTRAINT_MODE \
	(ae_constraint_mode_get_type())
static GType ae_constraint_mode_get_type()
{
	static GType ae_constraint_mode_type = 0;

	if (!ae_constraint_mode_type)
		ae_constraint_mode_type =
			g_enum_register_static("AeConstraintMode",
					       ae_constraint_mode_types);

	return ae_constraint_mode_type;
}

static const GEnumValue ae_exposure_mode_types[] = {
	{
		controls::ExposureNormal,
		"Default exposure mode. ",
		"normal"
	},
	{
		controls::ExposureShort,
		"Exposure mode allowing only short exposure times. ",
		"short"
	},
	{
		controls::ExposureLong,
		"Exposure mode allowing long exposure times. ",
		"long"
	},
	{
		controls::ExposureCustom,
		"Custom exposure mode. ",
		"custom"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AE_EXPOSURE_MODE \
	(ae_exposure_mode_get_type())
static GType ae_exposure_mode_get_type()
{
	static GType ae_exposure_mode_type = 0;

	if (!ae_exposure_mode_type)
		ae_exposure_mode_type =
			g_enum_register_static("AeExposureMode",
					       ae_exposure_mode_types);

	return ae_exposure_mode_type;
}

static const GEnumValue exposure_time_mode_types[] = {
	{
		controls::ExposureTimeModeAuto,
		"The exposure time will be calculated automatically and set by the "
		"AE algorithm. "
		"If ExposureTime is set while this mode is active, it will be "
		"ignored, and its value will not be retained. "
		"When transitioning from Manual to Auto mode, the AEGC should start "
		"its adjustments based on the last set manual ExposureTime value. ",
		"auto"
	},
	{
		controls::ExposureTimeModeManual,
		"The exposure time will not be updated by the AE algorithm. "
		"When transitioning from Auto to Manual mode, the last computed "
		"exposure value is used until a new value is specified through the "
		"ExposureTime control. If an ExposureTime value is specified in the "
		"same request where the ExposureTimeMode is changed from Auto to "
		"Manual, the provided ExposureTime is applied immediately. ",
		"manual"
	},
	{0, nullptr, nullptr}
};

#define TYPE_EXPOSURE_TIME_MODE \
	(exposure_time_mode_get_type())
static GType exposure_time_mode_get_type()
{
	static GType exposure_time_mode_type = 0;

	if (!exposure_time_mode_type)
		exposure_time_mode_type =
			g_enum_register_static("ExposureTimeMode",
					       exposure_time_mode_types);

	return exposure_time_mode_type;
}

static const GEnumValue analogue_gain_mode_types[] = {
	{
		controls::AnalogueGainModeAuto,
		"The analogue gain will be calculated automatically and set by the "
		"AEGC algorithm. "
		"If AnalogueGain is set while this mode is active, it will be "
		"ignored, and it will also not be retained. "
		"When transitioning from Manual to Auto mode, the AEGC should start "
		"its adjustments based on the last set manual AnalogueGain value. ",
		"auto"
	},
	{
		controls::AnalogueGainModeManual,
		"The analogue gain will not be updated by the AEGC algorithm. "
		"When transitioning from Auto to Manual mode, the last computed "
		"gain value is used until a new value is specified through the "
		"AnalogueGain control. If an AnalogueGain value is specified in the "
		"same request where the AnalogueGainMode is changed from Auto to "
		"Manual, the provided AnalogueGain is applied immediately. ",
		"manual"
	},
	{0, nullptr, nullptr}
};

#define TYPE_ANALOGUE_GAIN_MODE \
	(analogue_gain_mode_get_type())
static GType analogue_gain_mode_get_type()
{
	static GType analogue_gain_mode_type = 0;

	if (!analogue_gain_mode_type)
		analogue_gain_mode_type =
			g_enum_register_static("AnalogueGainMode",
					       analogue_gain_mode_types);

	return analogue_gain_mode_type;
}

static const GEnumValue awb_mode_types[] = {
	{
		controls::AwbAuto,
		"Search over the whole colour temperature range. ",
		"auto"
	},
	{
		controls::AwbIncandescent,
		"Incandescent AWB lamp mode. ",
		"incandescent"
	},
	{
		controls::AwbTungsten,
		"Tungsten AWB lamp mode. ",
		"tungsten"
	},
	{
		controls::AwbFluorescent,
		"Fluorescent AWB lamp mode. ",
		"fluorescent"
	},
	{
		controls::AwbIndoor,
		"Indoor AWB lighting mode. ",
		"indoor"
	},
	{
		controls::AwbDaylight,
		"Daylight AWB lighting mode. ",
		"daylight"
	},
	{
		controls::AwbCloudy,
		"Cloudy AWB lighting mode. ",
		"cloudy"
	},
	{
		controls::AwbCustom,
		"Custom AWB mode. ",
		"custom"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AWB_MODE \
	(awb_mode_get_type())
static GType awb_mode_get_type()
{
	static GType awb_mode_type = 0;

	if (!awb_mode_type)
		awb_mode_type =
			g_enum_register_static("AwbMode",
					       awb_mode_types);

	return awb_mode_type;
}

static const GEnumValue af_mode_types[] = {
	{
		controls::AfModeManual,
		"The AF algorithm is in manual mode. "
		"In this mode it will never perform any action nor move the lens of "
		"its own accord, but an application can specify the desired lens "
		"position using the LensPosition control. The AfState will always "
		"report AfStateIdle. "
		"If the camera is started in AfModeManual, it will move the focus "
		"lens to the position specified by the LensPosition control. "
		"This mode is the recommended default value for the AfMode control. "
		"External cameras (as reported by the Location property set to "
		"CameraLocationExternal) may use a different default value. ",
		"manual"
	},
	{
		controls::AfModeAuto,
		"The AF algorithm is in auto mode. "
		"In this mode the algorithm will never move the lens or change state "
		"unless the AfTrigger control is used. The AfTrigger control can be "
		"used to initiate a focus scan, the results of which will be "
		"reported by AfState. "
		"If the autofocus algorithm is moved from AfModeAuto to another mode "
		"while a scan is in progress, the scan is cancelled immediately, "
		"without waiting for the scan to finish. "
		"When first entering this mode the AfState will report AfStateIdle. "
		"When a trigger control is sent, AfState will report AfStateScanning "
		"for a period before spontaneously changing to AfStateFocused or "
		"AfStateFailed, depending on the outcome of the scan. It will remain "
		"in this state until another scan is initiated by the AfTrigger "
		"control. If a scan is cancelled (without changing to another mode), "
		"AfState will return to AfStateIdle. ",
		"auto"
	},
	{
		controls::AfModeContinuous,
		"The AF algorithm is in continuous mode. "
		"In this mode the lens can re-start a scan spontaneously at any "
		"moment, without any user intervention. The AfState still reports "
		"whether the algorithm is currently scanning or not, though the "
		"application has no ability to initiate or cancel scans, nor to move "
		"the lens for itself. "
		"However, applications can pause the AF algorithm from continuously "
		"scanning by using the AfPause control. This allows video or still "
		"images to be captured whilst guaranteeing that the focus is fixed. "
		"When set to AfModeContinuous, the system will immediately initiate a "
		"scan so AfState will report AfStateScanning, and will settle on one "
		"of AfStateFocused or AfStateFailed, depending on the scan result. ",
		"continuous"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AF_MODE \
	(af_mode_get_type())
static GType af_mode_get_type()
{
	static GType af_mode_type = 0;

	if (!af_mode_type)
		af_mode_type =
			g_enum_register_static("AfMode",
					       af_mode_types);

	return af_mode_type;
}

static const GEnumValue af_range_types[] = {
	{
		controls::AfRangeNormal,
		"A wide range of focus distances is scanned. "
		"Scanned distances cover all the way from infinity down to close "
		"distances, though depending on the implementation, possibly not "
		"including the very closest macro positions. ",
		"normal"
	},
	{
		controls::AfRangeMacro,
		"Only close distances are scanned. ",
		"macro"
	},
	{
		controls::AfRangeFull,
		"The full range of focus distances is scanned. "
		"This range is similar to AfRangeNormal but includes the very "
		"closest macro positions. ",
		"full"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AF_RANGE \
	(af_range_get_type())
static GType af_range_get_type()
{
	static GType af_range_type = 0;

	if (!af_range_type)
		af_range_type =
			g_enum_register_static("AfRange",
					       af_range_types);

	return af_range_type;
}

static const GEnumValue af_speed_types[] = {
	{
		controls::AfSpeedNormal,
		"Move the lens at its usual speed. ",
		"normal"
	},
	{
		controls::AfSpeedFast,
		"Move the lens more quickly. ",
		"fast"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AF_SPEED \
	(af_speed_get_type())
static GType af_speed_get_type()
{
	static GType af_speed_type = 0;

	if (!af_speed_type)
		af_speed_type =
			g_enum_register_static("AfSpeed",
					       af_speed_types);

	return af_speed_type;
}

static const GEnumValue af_metering_types[] = {
	{
		controls::AfMeteringAuto,
		"Let the AF algorithm decide for itself where it will measure focus. ",
		"auto"
	},
	{
		controls::AfMeteringWindows,
		"Use the rectangles defined by the AfWindows control to measure focus. "
		"If no windows are specified the behaviour is platform dependent. ",
		"windows"
	},
	{0, nullptr, nullptr}
};

#define TYPE_AF_METERING \
	(af_metering_get_type())
static GType af_metering_get_type()
{
	static GType af_metering_type = 0;

	if (!af_metering_type)
		af_metering_type =
			g_enum_register_static("AfMetering",
					       af_metering_types);

	return af_metering_type;
}


void GstCameraControls::installProperties(GObjectClass *klass, int lastPropId)
{

	g_object_class_install_property(
		klass,
		lastPropId + controls::AE_ENABLE,
		g_param_spec_boolean(
			"ae-enable",
			"AeEnable",
			"Enable or disable the AEGC algorithm. When this control is set to true, "
			"both ExposureTimeMode and AnalogueGainMode are set to auto, and if this "
			"control is set to false then both are set to manual. "
			"If ExposureTimeMode or AnalogueGainMode are also set in the same "
			"request as AeEnable, then the modes supplied by ExposureTimeMode or "
			"AnalogueGainMode will take precedence. "
			"See also: exposure-time-mode, analogue-gain-mode. ",
			false,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AE_METERING_MODE,
		g_param_spec_enum(
			"ae-metering-mode",
			"AeMeteringMode",
			"Specify a metering mode for the AE algorithm to use. "
			"The metering modes determine which parts of the image are used to "
			"determine the scene brightness. Metering modes may be platform specific "
			"and not all metering modes may be supported. ",
			TYPE_AE_METERING_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AE_CONSTRAINT_MODE,
		g_param_spec_enum(
			"ae-constraint-mode",
			"AeConstraintMode",
			"Specify a constraint mode for the AE algorithm to use. "
			"The constraint modes determine how the measured scene brightness is "
			"adjusted to reach the desired target exposure. Constraint modes may be "
			"platform specific, and not all constraint modes may be supported. ",
			TYPE_AE_CONSTRAINT_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AE_EXPOSURE_MODE,
		g_param_spec_enum(
			"ae-exposure-mode",
			"AeExposureMode",
			"Specify an exposure mode for the AE algorithm to use. "
			"The exposure modes specify how the desired total exposure is divided "
			"between the exposure time and the sensor's analogue gain. They are "
			"platform specific, and not all exposure modes may be supported. "
			"When one of AnalogueGainMode or ExposureTimeMode is set to Manual, "
			"the fixed values will override any choices made by AeExposureMode. "
			"See also: analogue-gain-mode. "
			"See also: exposure-time-mode. ",
			TYPE_AE_EXPOSURE_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::EXPOSURE_VALUE,
		g_param_spec_float(
			"exposure-value",
			"ExposureValue",
			"Specify an Exposure Value (EV) parameter. "
			"The EV parameter will only be applied if the AE algorithm is currently "
			"enabled, that is, at least one of AnalogueGainMode and ExposureTimeMode "
			"are in Auto mode. "
			"By convention EV adjusts the exposure as log2. For example "
			"EV = [-2, -1, -0.5, 0, 0.5, 1, 2] results in an exposure adjustment "
			"of [1/4x, 1/2x, 1/sqrt(2)x, 1x, sqrt(2)x, 2x, 4x]. "
			"See also: analogue-gain-mode. "
			"See also: exposure-time-mode. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::EXPOSURE_TIME,
		g_param_spec_int(
			"exposure-time",
			"ExposureTime",
			"Exposure time for the frame applied in the sensor device. "
			"This value is specified in microseconds. "
			"This control will only take effect if ExposureTimeMode is Manual. If "
			"this control is set when ExposureTimeMode is Auto, the value will be "
			"ignored and will not be retained. "
			"When reported in metadata, this control indicates what exposure time "
			"was used for the current frame, regardless of ExposureTimeMode. "
			"ExposureTimeMode will indicate the source of the exposure time value, "
			"whether it came from the AE algorithm or not. "
			"See also: analogue-gain. "
			"See also: exposure-time-mode. ",
			G_MININT, G_MAXINT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::EXPOSURE_TIME_MODE,
		g_param_spec_enum(
			"exposure-time-mode",
			"ExposureTimeMode",
			"Controls the source of the exposure time that is applied to the image "
			"sensor. "
			"When set to Auto, the AE algorithm computes the exposure time and "
			"configures the image sensor accordingly. When set to Manual, the value "
			"of the ExposureTime control is used. "
			"When transitioning from Auto to Manual mode and no ExposureTime control "
			"is provided by the application, the last value computed by the AE "
			"algorithm when the mode was Auto will be used. If the ExposureTimeMode "
			"was never set to Auto (either because the camera started in Manual mode, "
			"or Auto is not supported by the camera), the camera should use a "
			"best-effort default value. "
			"If ExposureTimeModeManual is supported, the ExposureTime control must "
			"also be supported. "
			"Cameras that support manual control of the sensor shall support manual "
			"mode for both ExposureTimeMode and AnalogueGainMode, and shall expose "
			"the ExposureTime and AnalogueGain controls. If the camera also has an "
			"AEGC implementation, both ExposureTimeMode and AnalogueGainMode shall "
			"support both manual and auto mode. If auto mode is available, it shall "
			"be the default mode. These rules do not apply to black box cameras "
			"such as UVC cameras, where the available gain and exposure modes are "
			"completely dependent on what the device exposes. "
			"\\par Flickerless exposure mode transitions "
			"Applications that wish to transition from ExposureTimeModeAuto to direct "
			"control of the exposure time without causing extra flicker can do so by "
			"selecting an ExposureTime value as close as possible to the last value "
			"computed by the auto exposure algorithm in order to avoid any visible "
			"flickering. "
			"To select the correct value to use as ExposureTime value, applications "
			"should accommodate the natural delay in applying controls caused by the "
			"capture pipeline frame depth. "
			"When switching to manual exposure mode, applications should not "
			"immediately specify an ExposureTime value in the same request where "
			"ExposureTimeMode is set to Manual. They should instead wait for the "
			"first Request where ExposureTimeMode is reported as "
			"ExposureTimeModeManual in the Request metadata, and use the reported "
			"ExposureTime to populate the control value in the next Request to be "
			"queued to the Camera. "
			"The implementation of the auto-exposure algorithm should equally try to "
			"minimize flickering and when transitioning from manual exposure mode to "
			"auto exposure use the last value provided by the application as starting "
			"point. "
			"1. Start with ExposureTimeMode set to Auto "
			"2. Set ExposureTimeMode to Manual "
			"3. Wait for the first completed request that has ExposureTimeMode "
			"set to Manual "
			"4. Copy the value reported in ExposureTime into a new request, and "
			"submit it "
			"5. Proceed to run manual exposure time as desired "
			"See also: exposure-time. ",
			TYPE_EXPOSURE_TIME_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::ANALOGUE_GAIN,
		g_param_spec_float(
			"analogue-gain",
			"AnalogueGain",
			"Analogue gain value applied in the sensor device. "
			"The value of the control specifies the gain multiplier applied to all "
			"colour channels. This value cannot be lower than 1.0. "
			"This control will only take effect if AnalogueGainMode is Manual. If "
			"this control is set when AnalogueGainMode is Auto, the value will be "
			"ignored and will not be retained. "
			"When reported in metadata, this control indicates what analogue gain "
			"was used for the current request, regardless of AnalogueGainMode. "
			"AnalogueGainMode will indicate the source of the analogue gain value, "
			"whether it came from the AEGC algorithm or not. "
			"See also: exposure-time. "
			"See also: analogue-gain-mode. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::ANALOGUE_GAIN_MODE,
		g_param_spec_enum(
			"analogue-gain-mode",
			"AnalogueGainMode",
			"Controls the source of the analogue gain that is applied to the image "
			"sensor. "
			"When set to Auto, the AEGC algorithm computes the analogue gain and "
			"configures the image sensor accordingly. When set to Manual, the value "
			"of the AnalogueGain control is used. "
			"When transitioning from Auto to Manual mode and no AnalogueGain control "
			"is provided by the application, the last value computed by the AEGC "
			"algorithm when the mode was Auto will be used. If the AnalogueGainMode "
			"was never set to Auto (either because the camera started in Manual mode, "
			"or Auto is not supported by the camera), the camera should use a "
			"best-effort default value. "
			"If AnalogueGainModeManual is supported, the AnalogueGain control must "
			"also be supported. "
			"For cameras where we have control over the ISP, both ExposureTimeMode "
			"and AnalogueGainMode are expected to support manual mode, and both "
			"controls (as well as ExposureTimeMode and AnalogueGain) are expected to "
			"be present. If the camera also has an AEGC implementation, both "
			"ExposureTimeMode and AnalogueGainMode shall support both manual and "
			"auto mode. If auto mode is available, it shall be the default mode. "
			"These rules do not apply to black box cameras such as UVC cameras, "
			"where the available gain and exposure modes are completely dependent on "
			"what the hardware exposes. "
			"The same procedure described for performing flickerless transitions in "
			"the ExposureTimeMode control documentation can be applied to analogue "
			"gain. "
			"See also: exposure-time-mode. "
			"See also: analogue-gain. ",
			TYPE_ANALOGUE_GAIN_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AE_FLICKER_PERIOD,
		g_param_spec_int(
			"ae-flicker-period",
			"AeFlickerPeriod",
			"Manual flicker period in microseconds. "
			"This value sets the current flicker period to avoid. It is used when "
			"AeFlickerMode is set to FlickerManual. "
			"To cancel 50Hz mains flicker, this should be set to 10000 (corresponding "
			"to 100Hz), or 8333 (120Hz) for 60Hz mains. "
			"Setting the mode to FlickerManual when no AeFlickerPeriod has ever been "
			"set means that no flicker cancellation occurs (until the value of this "
			"control is updated). "
			"Switching to modes other than FlickerManual has no effect on the "
			"value of the AeFlickerPeriod control. "
			"See also: ae-flicker-mode. ",
			G_MININT, G_MAXINT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::BRIGHTNESS,
		g_param_spec_float(
			"brightness",
			"Brightness",
			"Specify a fixed brightness parameter. "
			"Positive values (up to 1.0) produce brighter images; negative values "
			"(up to -1.0) produce darker images and 0.0 leaves pixels unchanged. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::CONTRAST,
		g_param_spec_float(
			"contrast",
			"Contrast",
			"Specify a fixed contrast parameter. "
			"Normal contrast is given by the value 1.0; larger values produce images "
			"with more contrast. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AWB_ENABLE,
		g_param_spec_boolean(
			"awb-enable",
			"AwbEnable",
			"Enable or disable the AWB. "
			"When AWB is enabled, the algorithm estimates the colour temperature of "
			"the scene and computes colour gains and the colour correction matrix "
			"automatically. The computed colour temperature, gains and correction "
			"matrix are reported in metadata. The corresponding controls are ignored "
			"if set in a request. "
			"When AWB is disabled, the colour temperature, gains and correction "
			"matrix are not updated automatically and can be set manually in "
			"requests. "
			"See also: colour-correction-matrix. "
			"See also: colour-gains. "
			"See also: colour-temperature. ",
			false,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AWB_MODE,
		g_param_spec_enum(
			"awb-mode",
			"AwbMode",
			"Specify the range of illuminants to use for the AWB algorithm. "
			"The modes supported are platform specific, and not all modes may be "
			"supported. ",
			TYPE_AWB_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::COLOUR_GAINS,
		gst_param_spec_array(
			"colour-gains",
			"ColourGains",
			"Pair of gain values for the Red and Blue colour channels, in that "
			"order. "
			"ColourGains can only be applied in a Request when the AWB is disabled. "
			"If ColourGains is set in a request but ColourTemperature is not, the "
			"implementation shall calculate and set the ColourTemperature based on "
			"the ColourGains. "
			"See also: awb-enable. "
			"See also: colour-temperature. ",
			g_param_spec_float(
				"colour-gains-value",
				"ColourGains Value",
				"One ColourGains element value",
				-G_MAXFLOAT, G_MAXFLOAT, 0,
				(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
					       G_PARAM_STATIC_STRINGS)
			),
			(GParamFlags) (GST_PARAM_CONTROLLABLE |
				       G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::SATURATION,
		g_param_spec_float(
			"saturation",
			"Saturation",
			"Specify a fixed saturation parameter. "
			"Normal saturation is given by the value 1.0; larger values produce more "
			"saturated colours; 0.0 produces a greyscale image. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::SHARPNESS,
		g_param_spec_float(
			"sharpness",
			"Sharpness",
			"Intensity of the sharpening applied to the image. "
			"A value of 0.0 means no sharpening. The minimum value means "
			"minimal sharpening, and shall be 0.0 unless the camera can't "
			"disable sharpening completely. The default value shall give a "
			"\"reasonable\" level of sharpening, suitable for most use cases. "
			"The maximum value may apply extremely high levels of sharpening, "
			"higher than anyone could reasonably want. Negative values are "
			"not allowed. Note also that sharpening is not applied to raw "
			"streams. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::COLOUR_CORRECTION_MATRIX,
		gst_param_spec_array(
			"colour-correction-matrix",
			"ColourCorrectionMatrix",
			"The 3x3 matrix that converts camera RGB to sRGB within the imaging "
			"pipeline. "
			"This should describe the matrix that is used after pixels have been "
			"white-balanced, but before any gamma transformation. The 3x3 matrix is "
			"stored in conventional reading order in an array of 9 floating point "
			"values. "
			"ColourCorrectionMatrix can only be applied in a Request when the AWB is  "
			"disabled. "
			"See also: awb-enable. "
			"See also: colour-temperature. ",
			g_param_spec_float(
				"colour-correction-matrix-value",
				"ColourCorrectionMatrix Value",
				"One ColourCorrectionMatrix element value",
				-G_MAXFLOAT, G_MAXFLOAT, 0,
				(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
					       G_PARAM_STATIC_STRINGS)
			),
			(GParamFlags) (GST_PARAM_CONTROLLABLE |
				       G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::SCALER_CROP,
		gst_param_spec_array(
			"scaler-crop",
			"ScalerCrop",
			"Sets the image portion that will be scaled to form the whole of "
			"the final output image. "
			"The (x,y) location of this rectangle is relative to the "
			"PixelArrayActiveAreas that is being used. The units remain native "
			"sensor pixels, even if the sensor is being used in a binning or "
			"skipping mode. "
			"This control is only present when the pipeline supports scaling. Its "
			"maximum valid value is given by the properties::ScalerCropMaximum "
			"property, and the two can be used to implement digital zoom. ",
			g_param_spec_int(
				"rectangle-value",
				"Rectangle Value",
				"One rectangle value, either x, y, width or height.",
				0, G_MAXINT, 0,
				(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
					       G_PARAM_STATIC_STRINGS)
			),
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::DIGITAL_GAIN,
		g_param_spec_float(
			"digital-gain",
			"DigitalGain",
			"Digital gain value applied during the processing steps applied "
			"to the image as captured from the sensor. "
			"The global digital gain factor is applied to all the colour channels "
			"of the RAW image. Different pipeline models are free to "
			"specify how the global gain factor applies to each separate "
			"channel. "
			"If an imaging pipeline applies digital gain in distinct "
			"processing steps, this value indicates their total sum. "
			"Pipelines are free to decide how to adjust each processing "
			"step to respect the received gain factor and shall report "
			"their total value in the request metadata. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AF_MODE,
		g_param_spec_enum(
			"af-mode",
			"AfMode",
			"The mode of the AF (autofocus) algorithm. "
			"An implementation may choose not to implement all the modes. ",
			TYPE_AF_MODE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AF_RANGE,
		g_param_spec_enum(
			"af-range",
			"AfRange",
			"The range of focus distances that is scanned. "
			"An implementation may choose not to implement all the options here. ",
			TYPE_AF_RANGE,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AF_SPEED,
		g_param_spec_enum(
			"af-speed",
			"AfSpeed",
			"Determine whether the AF is to move the lens as quickly as possible or "
			"more steadily. "
			"For example, during video recording it may be desirable not to move the "
			"lens too abruptly, but when in a preview mode (waiting for a still "
			"capture) it may be helpful to move the lens as quickly as is reasonably "
			"possible. ",
			TYPE_AF_SPEED,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AF_METERING,
		g_param_spec_enum(
			"af-metering",
			"AfMetering",
			"The parts of the image used by the AF algorithm to measure focus. ",
			TYPE_AF_METERING,
			0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::AF_WINDOWS,
		gst_param_spec_array(
			"af-windows",
			"AfWindows",
			"The focus windows used by the AF algorithm when AfMetering is set to "
			"AfMeteringWindows. "
			"The units used are pixels within the rectangle returned by the "
			"ScalerCropMaximum property. "
			"In order to be activated, a rectangle must be programmed with non-zero "
			"width and height. Internally, these rectangles are intersected with the "
			"ScalerCropMaximum rectangle. If the window becomes empty after this "
			"operation, then the window is ignored. If all the windows end up being "
			"ignored, then the behaviour is platform dependent. "
			"On platforms that support the ScalerCrop control (for implementing "
			"digital zoom, for example), no automatic recalculation or adjustment of "
			"AF windows is performed internally if the ScalerCrop is changed. If any "
			"window lies outside the output image after the scaler crop has been "
			"applied, it is up to the application to recalculate them. "
			"The details of how the windows are used are platform dependent. We note "
			"that when there is more than one AF window, a typical implementation "
			"might find the optimal focus position for each one and finally select "
			"the window where the focal distance for the objects shown in that part "
			"of the image are closest to the camera. ",
			gst_param_spec_array(
				"af-windows-value",
				"AfWindows Value",
				"One AfWindows element value",
				g_param_spec_int(
					"rectangle-value",
					"Rectangle Value",
					"One rectangle value, either x, y, width or height.",
					0, G_MAXINT, 0,
					(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
						       G_PARAM_STATIC_STRINGS)
				),
				(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
					       G_PARAM_STATIC_STRINGS)
			),
			(GParamFlags) (GST_PARAM_CONTROLLABLE |
				       G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::LENS_POSITION,
		g_param_spec_float(
			"lens-position",
			"LensPosition",
			"Set and report the focus lens position. "
			"This control instructs the lens to move to a particular position and "
			"also reports back the position of the lens for each frame. "
			"The LensPosition control is ignored unless the AfMode is set to "
			"AfModeManual, though the value is reported back unconditionally in all "
			"modes. "
			"This value, which is generally a non-integer, is the reciprocal of the "
			"focal distance in metres, also known as dioptres. That is, to set a "
			"focal distance D, the lens position LP is given by "
			"\\f$LP = \\frac{1\\mathrm{m}}{D}\\f$ "
			"For example: "
			"- 0 moves the lens to infinity. "
			"- 0.5 moves the lens to focus on objects 2m away. "
			"- 2 moves the lens to focus on objects 50cm away. "
			"- And larger values will focus the lens closer. "
			"The default value of the control should indicate a good general "
			"position for the lens, often corresponding to the hyperfocal distance "
			"(the closest position for which objects at infinity are still "
			"acceptably sharp). The minimum will often be zero (meaning infinity), "
			"and the maximum value defines the closest focus position. "
			"Todo: Define a property to report the Hyperfocal distance of calibrated "
			"lenses. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);

	g_object_class_install_property(
		klass,
		lastPropId + controls::GAMMA,
		g_param_spec_float(
			"gamma",
			"Gamma",
			"Specify a fixed gamma value. "
			"The default gamma value must be 2.2 which closely mimics sRGB gamma. "
			"Note that this is camera gamma, so it is applied as 1.0/gamma. ",
			-G_MAXFLOAT, G_MAXFLOAT, 0,
			(GParamFlags) (GST_PARAM_CONTROLLABLE | G_PARAM_READWRITE |
				       G_PARAM_STATIC_STRINGS)
		)
	);
}

bool GstCameraControls::getProperty(guint propId, GValue *value,
				    [[maybe_unused]] GParamSpec *pspec)
{
	if (!controls_acc_.contains(propId)) {
		GST_WARNING("Control '%s' is not available, default value will "
			    "be returned",
			    controls::controls.at(propId)->name().c_str());
		return true;
	}
	const ControlValue &cv = controls_acc_.get(propId);

	switch (propId) {

	case controls::AE_ENABLE: {
		auto control = cv.get<bool>();
		g_value_set_boolean(value, control);

		return true;
	}

	case controls::AE_METERING_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AE_CONSTRAINT_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AE_EXPOSURE_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::EXPOSURE_VALUE: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::EXPOSURE_TIME: {
		auto control = cv.get<int32_t>();
		g_value_set_int(value, control);

		return true;
	}

	case controls::EXPOSURE_TIME_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::ANALOGUE_GAIN: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::ANALOGUE_GAIN_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AE_FLICKER_PERIOD: {
		auto control = cv.get<int32_t>();
		g_value_set_int(value, control);

		return true;
	}

	case controls::BRIGHTNESS: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::CONTRAST: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::AWB_ENABLE: {
		auto control = cv.get<bool>();
		g_value_set_boolean(value, control);

		return true;
	}

	case controls::AWB_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::COLOUR_GAINS: {
		auto control = cv.get<Span<const float, 2>>();
		for (size_t i = 0; i < control.size(); ++i) {
			GValue element = G_VALUE_INIT;
			g_value_init(&element, G_TYPE_FLOAT);
			g_value_set_float(&element, control[i]);
			gst_value_array_append_and_take_value(value, &element);
		}

		return true;
	}

	case controls::SATURATION: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::SHARPNESS: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::COLOUR_CORRECTION_MATRIX: {
		auto control = cv.get<Span<const float, 9>>();
		for (size_t i = 0; i < control.size(); ++i) {
			GValue element = G_VALUE_INIT;
			g_value_init(&element, G_TYPE_FLOAT);
			g_value_set_float(&element, control[i]);
			gst_value_array_append_and_take_value(value, &element);
		}

		return true;
	}

	case controls::SCALER_CROP: {
		auto control = cv.get<Rectangle>();
		gst_libcamera_gvalue_set_rectangle(value, control);

		return true;
	}

	case controls::DIGITAL_GAIN: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::AF_MODE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AF_RANGE: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AF_SPEED: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AF_METERING: {
		auto control = cv.get<int32_t>();
		g_value_set_enum(value, control);

		return true;
	}

	case controls::AF_WINDOWS: {
		auto control = cv.get<Span<const Rectangle>>();
		for (size_t i = 0; i < control.size(); ++i) {
			GValue element = G_VALUE_INIT;
			g_value_init(&element, GST_TYPE_PARAM_ARRAY_LIST);
			gst_libcamera_gvalue_set_rectangle(&element, control[i]);
			gst_value_array_append_and_take_value(value, &element);
		}

		return true;
	}

	case controls::LENS_POSITION: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	case controls::GAMMA: {
		auto control = cv.get<float>();
		g_value_set_float(value, control);

		return true;
	}

	default:
		return false;
	}
}

bool GstCameraControls::setProperty(guint propId, const GValue *value,
				    [[maybe_unused]] GParamSpec *pspec)
{
	/*
	 * Check whether the camera capabilities are already available.
	 * They might not be available if the pipeline has not started yet.
	 */
	if (!capabilities_.empty()) {
 		/* If so, check that the control is supported by the camera. */
		const ControlId *cid = capabilities_.idmap().at(propId);
		auto info = capabilities_.find(cid);

		if (info == capabilities_.end()) {
			GST_WARNING("Control '%s' is not supported by the "
				    "camera and will be ignored",
				    cid->name().c_str());
			return true;
		}
	}

	switch (propId) {

	case controls::AE_ENABLE: {
		auto val = g_value_get_boolean(value);
		controls_.set(controls::AeEnable, val);
		controls_acc_.set(controls::AeEnable, val);
		return true;
	}

	case controls::AE_METERING_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AeMeteringMode, val);
		controls_acc_.set(controls::AeMeteringMode, val);
		return true;
	}

	case controls::AE_CONSTRAINT_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AeConstraintMode, val);
		controls_acc_.set(controls::AeConstraintMode, val);
		return true;
	}

	case controls::AE_EXPOSURE_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AeExposureMode, val);
		controls_acc_.set(controls::AeExposureMode, val);
		return true;
	}

	case controls::EXPOSURE_VALUE: {
		auto val = g_value_get_float(value);
		controls_.set(controls::ExposureValue, val);
		controls_acc_.set(controls::ExposureValue, val);
		return true;
	}

	case controls::EXPOSURE_TIME: {
		auto val = g_value_get_int(value);
		controls_.set(controls::ExposureTime, val);
		controls_acc_.set(controls::ExposureTime, val);
		return true;
	}

	case controls::EXPOSURE_TIME_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::ExposureTimeMode, val);
		controls_acc_.set(controls::ExposureTimeMode, val);
		return true;
	}

	case controls::ANALOGUE_GAIN: {
		auto val = g_value_get_float(value);
		controls_.set(controls::AnalogueGain, val);
		controls_acc_.set(controls::AnalogueGain, val);
		return true;
	}

	case controls::ANALOGUE_GAIN_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AnalogueGainMode, val);
		controls_acc_.set(controls::AnalogueGainMode, val);
		return true;
	}

	case controls::AE_FLICKER_PERIOD: {
		auto val = g_value_get_int(value);
		controls_.set(controls::AeFlickerPeriod, val);
		controls_acc_.set(controls::AeFlickerPeriod, val);
		return true;
	}

	case controls::BRIGHTNESS: {
		auto val = g_value_get_float(value);
		controls_.set(controls::Brightness, val);
		controls_acc_.set(controls::Brightness, val);
		return true;
	}

	case controls::CONTRAST: {
		auto val = g_value_get_float(value);
		controls_.set(controls::Contrast, val);
		controls_acc_.set(controls::Contrast, val);
		return true;
	}

	case controls::AWB_ENABLE: {
		auto val = g_value_get_boolean(value);
		controls_.set(controls::AwbEnable, val);
		controls_acc_.set(controls::AwbEnable, val);
		return true;
	}

	case controls::AWB_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AwbMode, val);
		controls_acc_.set(controls::AwbMode, val);
		return true;
	}

	case controls::COLOUR_GAINS: {
		size_t size = gst_value_array_get_size(value);
		if (size != 2) {
			GST_ERROR("Incorrect array size for control "
				  "'colour-gains', must be of "
				  "size 2");
			return true;
		}

		std::vector<float> values(size);
		for (size_t i = 0; i < size; ++i) {
			const GValue *element =
				gst_value_array_get_value(value, i);
			values[i] = g_value_get_float(element);
		}
		Span<const float, 2> val(values.data(), size);
		controls_.set(controls::ColourGains, val);
		controls_acc_.set(controls::ColourGains, val);
		return true;
	}

	case controls::SATURATION: {
		auto val = g_value_get_float(value);
		controls_.set(controls::Saturation, val);
		controls_acc_.set(controls::Saturation, val);
		return true;
	}

	case controls::SHARPNESS: {
		auto val = g_value_get_float(value);
		controls_.set(controls::Sharpness, val);
		controls_acc_.set(controls::Sharpness, val);
		return true;
	}

	case controls::COLOUR_CORRECTION_MATRIX: {
		size_t size = gst_value_array_get_size(value);
		if (size != 9) {
			GST_ERROR("Incorrect array size for control "
				  "'colour-correction-matrix', must be of "
				  "size 9");
			return true;
		}

		std::vector<float> values(size);
		for (size_t i = 0; i < size; ++i) {
			const GValue *element =
				gst_value_array_get_value(value, i);
			values[i] = g_value_get_float(element);
		}
		Span<const float, 9> val(values.data(), size);
		controls_.set(controls::ColourCorrectionMatrix, val);
		controls_acc_.set(controls::ColourCorrectionMatrix, val);
		return true;
	}

	case controls::SCALER_CROP: {
		if (gst_value_array_get_size(value) != 4) {
			GST_ERROR("Rectangle in control "
				  "'scaler-crop' must be an "
				  "array of size 4");
			return true;
		}
		Rectangle val = gst_libcamera_gvalue_get_rectangle(value);
		controls_.set(controls::ScalerCrop, val);
		controls_acc_.set(controls::ScalerCrop, val);
		return true;
	}

	case controls::DIGITAL_GAIN: {
		auto val = g_value_get_float(value);
		controls_.set(controls::DigitalGain, val);
		controls_acc_.set(controls::DigitalGain, val);
		return true;
	}

	case controls::AF_MODE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AfMode, val);
		controls_acc_.set(controls::AfMode, val);
		return true;
	}

	case controls::AF_RANGE: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AfRange, val);
		controls_acc_.set(controls::AfRange, val);
		return true;
	}

	case controls::AF_SPEED: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AfSpeed, val);
		controls_acc_.set(controls::AfSpeed, val);
		return true;
	}

	case controls::AF_METERING: {
		auto val = g_value_get_enum(value);
		controls_.set(controls::AfMetering, val);
		controls_acc_.set(controls::AfMetering, val);
		return true;
	}

	case controls::AF_WINDOWS: {
		size_t size = gst_value_array_get_size(value);

		std::vector<Rectangle> values(size);
		for (size_t i = 0; i < size; ++i) {
			const GValue *element =
				gst_value_array_get_value(value, i);
			if (gst_value_array_get_size(element) != 4) {
				GST_ERROR("Rectangle in control "
					  "'af-windows' at"
					  "index %zu must be an array of size 4",
					  i);
				return true;
			}
			values[i] = gst_libcamera_gvalue_get_rectangle(element);
		}
		Span<const Rectangle> val(values.data(), size);
		controls_.set(controls::AfWindows, val);
		controls_acc_.set(controls::AfWindows, val);
		return true;
	}

	case controls::LENS_POSITION: {
		auto val = g_value_get_float(value);
		controls_.set(controls::LensPosition, val);
		controls_acc_.set(controls::LensPosition, val);
		return true;
	}

	case controls::GAMMA: {
		auto val = g_value_get_float(value);
		controls_.set(controls::Gamma, val);
		controls_acc_.set(controls::Gamma, val);
		return true;
	}

	default:
		return false;
	}
}

void GstCameraControls::setCamera(const std::shared_ptr<libcamera::Camera> &cam)
{
	capabilities_ = cam->controls();

	/*
	 * Check the controls which were set before the camera capabilities were
	 * known. This is required because GStreamer may set properties before
	 * the pipeline has started and thus before the camera was known.
	 */
	ControlList new_controls;
	for (auto control = controls_acc_.begin();
	     control != controls_acc_.end();
	     ++control) {
		unsigned int id = control->first;
		ControlValue value = control->second;

		const ControlId *cid = capabilities_.idmap().at(id);
		auto info = capabilities_.find(cid);

		/* Only add controls which are supported. */
		if (info != capabilities_.end())
			new_controls.set(id, value);
		else
			GST_WARNING("Control '%s' is not supported by the "
				    "camera and will be ignored",
				    cid->name().c_str());
	}

	controls_acc_ = new_controls;
	controls_ = new_controls;
}

void GstCameraControls::applyControls(std::unique_ptr<libcamera::Request> &request)
{
	request->controls().merge(controls_);
	controls_.clear();
}

void GstCameraControls::readMetadata(libcamera::Request *request)
{
	controls_acc_.merge(request->metadata(),
			    ControlList::MergePolicy::OverwriteExisting);
}