/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm interface for mali-c55
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once


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

namespace mali_c55 {




enum class _MaliC55Cmd {
	Exit = 0,
	Init = 1,
	Start = 2,
	Stop = 3,
	Configure = 4,
	MapBuffers = 5,
	UnmapBuffers = 6,
	QueueRequest = 7,
	FillParams = 8,
	ProcessStats = 9,
};

enum class _MaliC55EventCmd {
	ParamsComputed = 1,
	StatsProcessed = 2,
	SetSensorControls = 3,
};


struct IPAConfigInfo
{
public:
#ifndef __DOXYGEN__
	IPAConfigInfo() = default;

	template<
		typename T1 = IPACameraSensorInfo,
		typename T2 = ControlInfoMap,
		std::enable_if_t<std::is_convertible_v<T1&&, IPACameraSensorInfo>> * = nullptr,
		std::enable_if_t<std::is_convertible_v<T2&&, ControlInfoMap>> * = nullptr
	>
	IPAConfigInfo(T1 &&_sensorInfo, T2 &&_sensorControls)
		: sensorInfo(std::forward<T1>(_sensorInfo))
		, sensorControls(std::forward<T2>(_sensorControls))
	{
	}
#endif


	IPACameraSensorInfo sensorInfo;
	ControlInfoMap sensorControls;
};

class IPAMaliC55Interface : public IPAInterface
{
public:

	virtual int32_t init(
		const IPASettings &settings,
		const IPAConfigInfo &configInfo,
		ControlInfoMap *ipaControls) = 0;

	virtual int32_t start() = 0;

	virtual void stop() = 0;

	virtual int32_t configure(
		const IPAConfigInfo &configInfo,
		const uint8_t bayerOrder,
		ControlInfoMap *ipaControls) = 0;

	virtual void mapBuffers(
		const std::vector<libcamera::IPABuffer> &buffers,
		const bool readOnly) = 0;

	virtual void unmapBuffers(
		const std::vector<libcamera::IPABuffer> &buffers) = 0;

	virtual void queueRequest(
		const uint32_t request,
		const ControlList &reqControls) = 0;

	virtual void fillParams(
		const uint32_t request,
		const uint32_t bufferId) = 0;

	virtual void processStats(
		const uint32_t request,
		const uint32_t bufferId,
		const ControlList &sensorControls) = 0;

	Signal<uint32_t, uint32_t> paramsComputed;

	Signal<uint32_t, const ControlList &> statsProcessed;

	Signal<const ControlList &> setSensorControls;
};

} /* namespace mali_c55 */

} /* namespace ipa */

} /* namespace libcamera */