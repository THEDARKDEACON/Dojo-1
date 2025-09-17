/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for mali-c55
 *
 * This file is auto-generated. Do not edit.
 */

#pragma once

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>

#include <libcamera/base/object.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

namespace ipa {

namespace mali_c55 {


class IPAProxyMaliC55 : public IPAProxy, public IPAMaliC55Interface, public Object
{
public:
	IPAProxyMaliC55(IPAModule *ipam, bool isolate);
	~IPAProxyMaliC55();


        int32_t init(
        	const IPASettings &settings,
        	const IPAConfigInfo &configInfo,
        	ControlInfoMap *ipaControls) override;

        int32_t start() override;

        void stop() override;

        int32_t configure(
        	const IPAConfigInfo &configInfo,
        	const uint8_t bayerOrder,
        	ControlInfoMap *ipaControls) override;

        void mapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers,
        	const bool readOnly) override;

        void unmapBuffers(
        	const std::vector<libcamera::IPABuffer> &buffers) override;

        void queueRequest(
        	const uint32_t request,
        	const ControlList &reqControls) override;

        void fillParams(
        	const uint32_t request,
        	const uint32_t bufferId) override;

        void processStats(
        	const uint32_t request,
        	const uint32_t bufferId,
        	const ControlList &sensorControls) override;


private:
	void recvMessage(const IPCMessage &data);


        int32_t initThread(
        	const IPASettings &settings,
        	const IPAConfigInfo &configInfo,
        	ControlInfoMap *ipaControls);
        int32_t initIPC(
        	const IPASettings &settings,
        	const IPAConfigInfo &configInfo,
        	ControlInfoMap *ipaControls);

        int32_t startThread();
        int32_t startIPC();

        void stopThread();
        void stopIPC();

        int32_t configureThread(
        	const IPAConfigInfo &configInfo,
        	const uint8_t bayerOrder,
        	ControlInfoMap *ipaControls);
        int32_t configureIPC(
        	const IPAConfigInfo &configInfo,
        	const uint8_t bayerOrder,
        	ControlInfoMap *ipaControls);

        void mapBuffersThread(
        	const std::vector<libcamera::IPABuffer> &buffers,
        	const bool readOnly);
        void mapBuffersIPC(
        	const std::vector<libcamera::IPABuffer> &buffers,
        	const bool readOnly);

        void unmapBuffersThread(
        	const std::vector<libcamera::IPABuffer> &buffers);
        void unmapBuffersIPC(
        	const std::vector<libcamera::IPABuffer> &buffers);

        void queueRequestThread(
        	const uint32_t request,
        	const ControlList &reqControls);
        void queueRequestIPC(
        	const uint32_t request,
        	const ControlList &reqControls);

        void fillParamsThread(
        	const uint32_t request,
        	const uint32_t bufferId);
        void fillParamsIPC(
        	const uint32_t request,
        	const uint32_t bufferId);

        void processStatsThread(
        	const uint32_t request,
        	const uint32_t bufferId,
        	const ControlList &sensorControls);
        void processStatsIPC(
        	const uint32_t request,
        	const uint32_t bufferId,
        	const ControlList &sensorControls);


        void paramsComputedThread(
        	const uint32_t request,
        	const uint32_t bytesused);
	void paramsComputedIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

        void statsProcessedThread(
        	const uint32_t request,
        	const ControlList &metadata);
	void statsProcessedIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);

        void setSensorControlsThread(
        	const ControlList &sensorControls);
	void setSensorControlsIPC(
		std::vector<uint8_t>::const_iterator data,
		size_t dataSize,
		const std::vector<SharedFD> &fds);


	/* Helper class to invoke async functions in another thread. */
	class ThreadProxy : public Object
	{
	public:
		ThreadProxy()
			: ipa_(nullptr)
		{
		}

		void setIPA(IPAMaliC55Interface *ipa)
		{
			ipa_ = ipa;
		}

		void stop()
		{
			ipa_->stop();
		}

		int32_t start()
		{
			return ipa_->start();
		}
		void queueRequest(
                	const uint32_t request,
                	const ControlList &reqControls)
		{
			ipa_->queueRequest(request, reqControls);
		}
		void fillParams(
                	const uint32_t request,
                	const uint32_t bufferId)
		{
			ipa_->fillParams(request, bufferId);
		}
		void processStats(
                	const uint32_t request,
                	const uint32_t bufferId,
                	const ControlList &sensorControls)
		{
			ipa_->processStats(request, bufferId, sensorControls);
		}

	private:
		IPAMaliC55Interface *ipa_;
	};

	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPAMaliC55Interface> ipa_;

	const bool isolate_;

	std::unique_ptr<IPCPipeUnixSocket> ipc_;

	ControlSerializer controlSerializer_;


	uint32_t seq_;
};

} /* namespace mali_c55 */

} /* namespace ipa */

} /* namespace libcamera */