/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm interface for rkisp1
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <libcamera/base/flags.h>
#include <libcamera/base/signal.h>

#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>
#include <libcamera/ipa/ipa_interface.h>

namespace libcamera {

namespace ipa {

namespace rkisp1 {




enum class _RkISP1Cmd {
	Exit = 0,
	Init = 1,
	Start = 2,
	Stop = 3,
	Configure = 4,
	MapBuffers = 5,
	UnmapBuffers = 6,
	QueueRequest = 7,
	ComputeParams = 8,
	ProcessStats = 9,
};

enum class _RkISP1EventCmd {
	ParamsComputed = 1,
	SetSensorControls = 2,
	MetadataReady = 3,
};


struct IPAConfigInfo
{
public:
#ifndef __DOXYGEN__
	IPAConfigInfo() = default;

	template<
		typename T1 = IPACameraSensorInfo,
		typename T2 = ControlInfoMap,
		typename T3 = uint32_t,
		std::enable_if_t<std::is_convertible_v<T1&&, IPACameraSensorInfo>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, ControlInfoMap>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T3&&, uint32_t>> * = nullptr
	>
	IPAConfigInfo(T1 &&_sensorInfo, T2 &&_sensorControls, T3 &&_paramFormat)
		: sensorInfo(std::forward<T1>(_sensorInfo))
		, sensorControls(std::forward<T2>(_sensorControls))
		, paramFormat(std::forward<T3>(_paramFormat))
	{
	}
#endif


	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
	uint32_t paramFormat{ 0 };
};

class IPARkISP1Interface : public IPAInterface
{
public:

	virtual int32_t init(
		const IPASettings &settings,
		const uint32_t hwRevision,
		const IPACameraSensorInfo &sensorInfo,
		const ControlInfoMap &sensorControls,
		ControlInfoMap *ipaControls) = 0;

	virtual int32_t start() = 0;

	virtual void stop() = 0;

	virtual int32_t configure(
		const IPAConfigInfo &configInfo,
		const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
		ControlInfoMap *ipaControls) = 0;

	virtual void mapBuffers(
		const std::vector<libcamera::IPABuffer> &buffers) = 0;

	virtual void unmapBuffers(
		const std::vector<uint32_t> &ids) = 0;

	virtual void queueRequest(
		const uint32_t frame,
		const ControlList &reqControls) = 0;

	virtual void computeParams(
		const uint32_t frame,
		const uint32_t bufferId) = 0;

	virtual void processStats(
		const uint32_t frame,
		const uint32_t bufferId,
		const ControlList &sensorControls) = 0;

	Signal<uint32_t, uint32_t> paramsComputed;

	Signal<uint32_t, const ControlList &> setSensorControls;

	Signal<uint32_t, const ControlList &> metadataReady;
};

} /* namespace rkisp1 */

} /* namespace ipa */

} /* namespace libcamera */