/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Auto-generated controls
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/control_ids.h>

#include <pybind11/pybind11.h>

#include "py_main.h"

namespace py = pybind11;

class PyControls
{
};

class PyDebugControls
{
};

class PyDraftControls
{
};

class PyRpiControls
{
};

void init_py_controls_generated(py::module& m)
{
	auto controls = py::class_<PyControls>(m, "controls");
	auto debug = py::class_<PyDebugControls>(controls, "debug");
	auto draft = py::class_<PyDraftControls>(controls, "draft");
	auto rpi = py::class_<PyRpiControls>(controls, "rpi");


        controls.def_readonly_static("AeEnable", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeEnable));

        controls.def_readonly_static("AeState", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeState));

        py::enum_<libcamera::controls::AeStateEnum>(controls, "AeStateEnum")
                .value("Idle", libcamera::controls::AeStateIdle)
                .value("Searching", libcamera::controls::AeStateSearching)
                .value("Converged", libcamera::controls::AeStateConverged)
        ;

        controls.def_readonly_static("AeMeteringMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeMeteringMode));

        py::enum_<libcamera::controls::AeMeteringModeEnum>(controls, "AeMeteringModeEnum")
                .value("CentreWeighted", libcamera::controls::MeteringCentreWeighted)
                .value("Spot", libcamera::controls::MeteringSpot)
                .value("Matrix", libcamera::controls::MeteringMatrix)
                .value("Custom", libcamera::controls::MeteringCustom)
        ;

        controls.def_readonly_static("AeConstraintMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeConstraintMode));

        py::enum_<libcamera::controls::AeConstraintModeEnum>(controls, "AeConstraintModeEnum")
                .value("Normal", libcamera::controls::ConstraintNormal)
                .value("Highlight", libcamera::controls::ConstraintHighlight)
                .value("Shadows", libcamera::controls::ConstraintShadows)
                .value("Custom", libcamera::controls::ConstraintCustom)
        ;

        controls.def_readonly_static("AeExposureMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeExposureMode));

        py::enum_<libcamera::controls::AeExposureModeEnum>(controls, "AeExposureModeEnum")
                .value("Normal", libcamera::controls::ExposureNormal)
                .value("Short", libcamera::controls::ExposureShort)
                .value("Long", libcamera::controls::ExposureLong)
                .value("Custom", libcamera::controls::ExposureCustom)
        ;

        controls.def_readonly_static("ExposureValue", static_cast<const libcamera::ControlId *>(&libcamera::controls::ExposureValue));

        controls.def_readonly_static("ExposureTime", static_cast<const libcamera::ControlId *>(&libcamera::controls::ExposureTime));

        controls.def_readonly_static("ExposureTimeMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::ExposureTimeMode));

        py::enum_<libcamera::controls::ExposureTimeModeEnum>(controls, "ExposureTimeModeEnum")
                .value("Auto", libcamera::controls::ExposureTimeModeAuto)
                .value("Manual", libcamera::controls::ExposureTimeModeManual)
        ;

        controls.def_readonly_static("AnalogueGain", static_cast<const libcamera::ControlId *>(&libcamera::controls::AnalogueGain));

        controls.def_readonly_static("AnalogueGainMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AnalogueGainMode));

        py::enum_<libcamera::controls::AnalogueGainModeEnum>(controls, "AnalogueGainModeEnum")
                .value("Auto", libcamera::controls::AnalogueGainModeAuto)
                .value("Manual", libcamera::controls::AnalogueGainModeManual)
        ;

        controls.def_readonly_static("AeFlickerMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeFlickerMode));

        py::enum_<libcamera::controls::AeFlickerModeEnum>(controls, "AeFlickerModeEnum")
                .value("Off", libcamera::controls::FlickerOff)
                .value("Manual", libcamera::controls::FlickerManual)
                .value("Auto", libcamera::controls::FlickerAuto)
        ;

        controls.def_readonly_static("AeFlickerPeriod", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeFlickerPeriod));

        controls.def_readonly_static("AeFlickerDetected", static_cast<const libcamera::ControlId *>(&libcamera::controls::AeFlickerDetected));

        controls.def_readonly_static("Brightness", static_cast<const libcamera::ControlId *>(&libcamera::controls::Brightness));

        controls.def_readonly_static("Contrast", static_cast<const libcamera::ControlId *>(&libcamera::controls::Contrast));

        controls.def_readonly_static("Lux", static_cast<const libcamera::ControlId *>(&libcamera::controls::Lux));

        controls.def_readonly_static("AwbEnable", static_cast<const libcamera::ControlId *>(&libcamera::controls::AwbEnable));

        controls.def_readonly_static("AwbMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AwbMode));

        py::enum_<libcamera::controls::AwbModeEnum>(controls, "AwbModeEnum")
                .value("Auto", libcamera::controls::AwbAuto)
                .value("Incandescent", libcamera::controls::AwbIncandescent)
                .value("Tungsten", libcamera::controls::AwbTungsten)
                .value("Fluorescent", libcamera::controls::AwbFluorescent)
                .value("Indoor", libcamera::controls::AwbIndoor)
                .value("Daylight", libcamera::controls::AwbDaylight)
                .value("Cloudy", libcamera::controls::AwbCloudy)
                .value("Custom", libcamera::controls::AwbCustom)
        ;

        controls.def_readonly_static("AwbLocked", static_cast<const libcamera::ControlId *>(&libcamera::controls::AwbLocked));

        controls.def_readonly_static("ColourGains", static_cast<const libcamera::ControlId *>(&libcamera::controls::ColourGains));

        controls.def_readonly_static("ColourTemperature", static_cast<const libcamera::ControlId *>(&libcamera::controls::ColourTemperature));

        controls.def_readonly_static("Saturation", static_cast<const libcamera::ControlId *>(&libcamera::controls::Saturation));

        controls.def_readonly_static("SensorBlackLevels", static_cast<const libcamera::ControlId *>(&libcamera::controls::SensorBlackLevels));

        controls.def_readonly_static("Sharpness", static_cast<const libcamera::ControlId *>(&libcamera::controls::Sharpness));

        controls.def_readonly_static("FocusFoM", static_cast<const libcamera::ControlId *>(&libcamera::controls::FocusFoM));

        controls.def_readonly_static("ColourCorrectionMatrix", static_cast<const libcamera::ControlId *>(&libcamera::controls::ColourCorrectionMatrix));

        controls.def_readonly_static("ScalerCrop", static_cast<const libcamera::ControlId *>(&libcamera::controls::ScalerCrop));

        controls.def_readonly_static("DigitalGain", static_cast<const libcamera::ControlId *>(&libcamera::controls::DigitalGain));

        controls.def_readonly_static("FrameDuration", static_cast<const libcamera::ControlId *>(&libcamera::controls::FrameDuration));

        controls.def_readonly_static("FrameDurationLimits", static_cast<const libcamera::ControlId *>(&libcamera::controls::FrameDurationLimits));

        controls.def_readonly_static("SensorTemperature", static_cast<const libcamera::ControlId *>(&libcamera::controls::SensorTemperature));

        controls.def_readonly_static("SensorTimestamp", static_cast<const libcamera::ControlId *>(&libcamera::controls::SensorTimestamp));

        controls.def_readonly_static("AfMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfMode));

        py::enum_<libcamera::controls::AfModeEnum>(controls, "AfModeEnum")
                .value("Manual", libcamera::controls::AfModeManual)
                .value("Auto", libcamera::controls::AfModeAuto)
                .value("Continuous", libcamera::controls::AfModeContinuous)
        ;

        controls.def_readonly_static("AfRange", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfRange));

        py::enum_<libcamera::controls::AfRangeEnum>(controls, "AfRangeEnum")
                .value("Normal", libcamera::controls::AfRangeNormal)
                .value("Macro", libcamera::controls::AfRangeMacro)
                .value("Full", libcamera::controls::AfRangeFull)
        ;

        controls.def_readonly_static("AfSpeed", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfSpeed));

        py::enum_<libcamera::controls::AfSpeedEnum>(controls, "AfSpeedEnum")
                .value("Normal", libcamera::controls::AfSpeedNormal)
                .value("Fast", libcamera::controls::AfSpeedFast)
        ;

        controls.def_readonly_static("AfMetering", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfMetering));

        py::enum_<libcamera::controls::AfMeteringEnum>(controls, "AfMeteringEnum")
                .value("Auto", libcamera::controls::AfMeteringAuto)
                .value("Windows", libcamera::controls::AfMeteringWindows)
        ;

        controls.def_readonly_static("AfWindows", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfWindows));

        controls.def_readonly_static("AfTrigger", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfTrigger));

        py::enum_<libcamera::controls::AfTriggerEnum>(controls, "AfTriggerEnum")
                .value("Start", libcamera::controls::AfTriggerStart)
                .value("Cancel", libcamera::controls::AfTriggerCancel)
        ;

        controls.def_readonly_static("AfPause", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfPause));

        py::enum_<libcamera::controls::AfPauseEnum>(controls, "AfPauseEnum")
                .value("Immediate", libcamera::controls::AfPauseImmediate)
                .value("Deferred", libcamera::controls::AfPauseDeferred)
                .value("Resume", libcamera::controls::AfPauseResume)
        ;

        controls.def_readonly_static("LensPosition", static_cast<const libcamera::ControlId *>(&libcamera::controls::LensPosition));

        controls.def_readonly_static("AfState", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfState));

        py::enum_<libcamera::controls::AfStateEnum>(controls, "AfStateEnum")
                .value("Idle", libcamera::controls::AfStateIdle)
                .value("Scanning", libcamera::controls::AfStateScanning)
                .value("Focused", libcamera::controls::AfStateFocused)
                .value("Failed", libcamera::controls::AfStateFailed)
        ;

        controls.def_readonly_static("AfPauseState", static_cast<const libcamera::ControlId *>(&libcamera::controls::AfPauseState));

        py::enum_<libcamera::controls::AfPauseStateEnum>(controls, "AfPauseStateEnum")
                .value("Running", libcamera::controls::AfPauseStateRunning)
                .value("Pausing", libcamera::controls::AfPauseStatePausing)
                .value("Paused", libcamera::controls::AfPauseStatePaused)
        ;

        controls.def_readonly_static("HdrMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::HdrMode));

        py::enum_<libcamera::controls::HdrModeEnum>(controls, "HdrModeEnum")
                .value("Off", libcamera::controls::HdrModeOff)
                .value("MultiExposureUnmerged", libcamera::controls::HdrModeMultiExposureUnmerged)
                .value("MultiExposure", libcamera::controls::HdrModeMultiExposure)
                .value("SingleExposure", libcamera::controls::HdrModeSingleExposure)
                .value("Night", libcamera::controls::HdrModeNight)
        ;

        controls.def_readonly_static("HdrChannel", static_cast<const libcamera::ControlId *>(&libcamera::controls::HdrChannel));

        py::enum_<libcamera::controls::HdrChannelEnum>(controls, "HdrChannelEnum")
                .value("None", libcamera::controls::HdrChannelNone)
                .value("Short", libcamera::controls::HdrChannelShort)
                .value("Medium", libcamera::controls::HdrChannelMedium)
                .value("Long", libcamera::controls::HdrChannelLong)
        ;

        controls.def_readonly_static("Gamma", static_cast<const libcamera::ControlId *>(&libcamera::controls::Gamma));

        controls.def_readonly_static("DebugMetadataEnable", static_cast<const libcamera::ControlId *>(&libcamera::controls::DebugMetadataEnable));

        controls.def_readonly_static("FrameWallClock", static_cast<const libcamera::ControlId *>(&libcamera::controls::FrameWallClock));

        draft.def_readonly_static("AePrecaptureTrigger", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::AePrecaptureTrigger));

        py::enum_<libcamera::controls::draft::AePrecaptureTriggerEnum>(draft, "AePrecaptureTriggerEnum")
                .value("Idle", libcamera::controls::draft::AePrecaptureTriggerIdle)
                .value("Start", libcamera::controls::draft::AePrecaptureTriggerStart)
                .value("Cancel", libcamera::controls::draft::AePrecaptureTriggerCancel)
        ;

        draft.def_readonly_static("NoiseReductionMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::NoiseReductionMode));

        py::enum_<libcamera::controls::draft::NoiseReductionModeEnum>(draft, "NoiseReductionModeEnum")
                .value("Off", libcamera::controls::draft::NoiseReductionModeOff)
                .value("Fast", libcamera::controls::draft::NoiseReductionModeFast)
                .value("HighQuality", libcamera::controls::draft::NoiseReductionModeHighQuality)
                .value("Minimal", libcamera::controls::draft::NoiseReductionModeMinimal)
                .value("ZSL", libcamera::controls::draft::NoiseReductionModeZSL)
        ;

        draft.def_readonly_static("ColorCorrectionAberrationMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::ColorCorrectionAberrationMode));

        py::enum_<libcamera::controls::draft::ColorCorrectionAberrationModeEnum>(draft, "ColorCorrectionAberrationModeEnum")
                .value("Off", libcamera::controls::draft::ColorCorrectionAberrationOff)
                .value("Fast", libcamera::controls::draft::ColorCorrectionAberrationFast)
                .value("HighQuality", libcamera::controls::draft::ColorCorrectionAberrationHighQuality)
        ;

        draft.def_readonly_static("AwbState", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::AwbState));

        py::enum_<libcamera::controls::draft::AwbStateEnum>(draft, "AwbStateEnum")
                .value("StateInactive", libcamera::controls::draft::AwbStateInactive)
                .value("StateSearching", libcamera::controls::draft::AwbStateSearching)
                .value("Converged", libcamera::controls::draft::AwbConverged)
                .value("Locked", libcamera::controls::draft::AwbLocked)
        ;

        draft.def_readonly_static("SensorRollingShutterSkew", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::SensorRollingShutterSkew));

        draft.def_readonly_static("LensShadingMapMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::LensShadingMapMode));

        py::enum_<libcamera::controls::draft::LensShadingMapModeEnum>(draft, "LensShadingMapModeEnum")
                .value("Off", libcamera::controls::draft::LensShadingMapModeOff)
                .value("On", libcamera::controls::draft::LensShadingMapModeOn)
        ;

        draft.def_readonly_static("PipelineDepth", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::PipelineDepth));

        draft.def_readonly_static("MaxLatency", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::MaxLatency));

        draft.def_readonly_static("TestPatternMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::TestPatternMode));

        py::enum_<libcamera::controls::draft::TestPatternModeEnum>(draft, "TestPatternModeEnum")
                .value("Off", libcamera::controls::draft::TestPatternModeOff)
                .value("SolidColor", libcamera::controls::draft::TestPatternModeSolidColor)
                .value("ColorBars", libcamera::controls::draft::TestPatternModeColorBars)
                .value("ColorBarsFadeToGray", libcamera::controls::draft::TestPatternModeColorBarsFadeToGray)
                .value("Pn9", libcamera::controls::draft::TestPatternModePn9)
                .value("Custom1", libcamera::controls::draft::TestPatternModeCustom1)
        ;

        draft.def_readonly_static("FaceDetectMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::FaceDetectMode));

        py::enum_<libcamera::controls::draft::FaceDetectModeEnum>(draft, "FaceDetectModeEnum")
                .value("Off", libcamera::controls::draft::FaceDetectModeOff)
                .value("Simple", libcamera::controls::draft::FaceDetectModeSimple)
                .value("Full", libcamera::controls::draft::FaceDetectModeFull)
        ;

        draft.def_readonly_static("FaceDetectFaceRectangles", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::FaceDetectFaceRectangles));

        draft.def_readonly_static("FaceDetectFaceScores", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::FaceDetectFaceScores));

        draft.def_readonly_static("FaceDetectFaceLandmarks", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::FaceDetectFaceLandmarks));

        draft.def_readonly_static("FaceDetectFaceIds", static_cast<const libcamera::ControlId *>(&libcamera::controls::draft::FaceDetectFaceIds));

        rpi.def_readonly_static("StatsOutputEnable", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::StatsOutputEnable));

        rpi.def_readonly_static("Bcm2835StatsOutput", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::Bcm2835StatsOutput));

        rpi.def_readonly_static("ScalerCrops", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::ScalerCrops));

        rpi.def_readonly_static("PispStatsOutput", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::PispStatsOutput));

        rpi.def_readonly_static("SyncMode", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::SyncMode));

        py::enum_<libcamera::controls::rpi::SyncModeEnum>(rpi, "SyncModeEnum")
                .value("Off", libcamera::controls::rpi::SyncModeOff)
                .value("Server", libcamera::controls::rpi::SyncModeServer)
                .value("Client", libcamera::controls::rpi::SyncModeClient)
        ;

        rpi.def_readonly_static("SyncReady", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::SyncReady));

        rpi.def_readonly_static("SyncTimer", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::SyncTimer));

        rpi.def_readonly_static("SyncFrames", static_cast<const libcamera::ControlId *>(&libcamera::controls::rpi::SyncFrames));
}