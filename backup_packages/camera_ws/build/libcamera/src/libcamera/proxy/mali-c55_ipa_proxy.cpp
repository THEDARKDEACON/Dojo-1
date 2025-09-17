/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for mali-c55
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/ipa/mali-c55_ipa_proxy.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_serializer.h>

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"
#include "libcamera/internal/process.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAProxy)

namespace ipa {

namespace mali_c55 {


IPAProxyMaliC55::IPAProxyMaliC55(IPAModule *ipam, bool isolate)
	: IPAProxy(ipam), isolate_(isolate),
	  controlSerializer_(ControlSerializer::Role::Proxy), seq_(0)
{
	LOG(IPAProxy, Debug)
		<< "initializing mali-c55 proxy: loading IPA from "
		<< ipam->path();

	if (isolate_) {
		const std::string proxyWorkerPath = resolvePath("mali-c55_ipa_proxy");
		if (proxyWorkerPath.empty()) {
			LOG(IPAProxy, Error)
				<< "Failed to get proxy worker path";
			return;
		}

		auto ipc = std::make_unique<IPCPipeUnixSocket>(ipam->path().c_str(),
							       proxyWorkerPath.c_str());
		if (!ipc->isConnected()) {
			LOG(IPAProxy, Error) << "Failed to create IPCPipe";
			return;
		}

		ipc->recv.connect(this, &IPAProxyMaliC55::recvMessage);

		ipc_ = std::move(ipc);
		valid_ = true;
		return;
	}

	if (!ipam->load())
		return;

	IPAInterface *ipai = ipam->createInterface();
	if (!ipai) {
		LOG(IPAProxy, Error)
			<< "Failed to create IPA context for " << ipam->path();
		return;
	}

	ipa_ = std::unique_ptr<IPAMaliC55Interface>(static_cast<IPAMaliC55Interface *>(ipai));
	proxy_.setIPA(ipa_.get());


	ipa_->paramsComputed.connect(this, &IPAProxyMaliC55::paramsComputedThread);
	ipa_->statsProcessed.connect(this, &IPAProxyMaliC55::statsProcessedThread);
	ipa_->setSensorControls.connect(this, &IPAProxyMaliC55::setSensorControlsThread);

	valid_ = true;
}

IPAProxyMaliC55::~IPAProxyMaliC55()
{
	if (ipc_) {
		IPCMessage::Header header =
			{ static_cast<uint32_t>(_MaliC55Cmd::Exit), seq_++ };
		IPCMessage msg(header);
		ipc_->sendAsync(msg);
	}
}


void IPAProxyMaliC55::recvMessage(const IPCMessage &data)
{
	size_t dataSize = data.data().size();
	_MaliC55EventCmd _cmd = static_cast<_MaliC55EventCmd>(data.header().cmd);

	switch (_cmd) {
	case _MaliC55EventCmd::ParamsComputed: {
		paramsComputedIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _MaliC55EventCmd::StatsProcessed: {
		statsProcessedIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _MaliC55EventCmd::SetSensorControls: {
		setSensorControlsIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	default:
		LOG(IPAProxy, Error) << "Unknown command " << static_cast<uint32_t>(_cmd);
	}
}


int32_t IPAProxyMaliC55::init(
	const IPASettings &settings,
	const IPAConfigInfo &configInfo,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return initIPC(settings, configInfo, ipaControls);
	else
		return initThread(settings, configInfo, ipaControls);
}

int32_t IPAProxyMaliC55::initThread(
	const IPASettings &settings,
	const IPAConfigInfo &configInfo,
	ControlInfoMap *ipaControls)
{
	int32_t _ret = ipa_->init(settings, configInfo, ipaControls);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxyMaliC55::initIPC(
	const IPASettings &settings,
	const IPAConfigInfo &configInfo,
	ControlInfoMap *ipaControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::Init), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> settingsBuf;
	std::tie(settingsBuf, std::ignore) =
		IPADataSerializer<IPASettings>::serialize(settings
);
	std::vector<uint8_t> configInfoBuf;
	std::tie(configInfoBuf, std::ignore) =
		IPADataSerializer<IPAConfigInfo>::serialize(configInfo
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), settingsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), configInfoBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call init: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);



	const size_t ipaControlsStart = 4;


	if (ipaControls) {
                *ipaControls =
                IPADataSerializer<ControlInfoMap>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart,
                	_ipcOutputBuf.data().cend(),
                	&controlSerializer_);
	}


	return _retValue;

}


int32_t IPAProxyMaliC55::start()
{
	if (isolate_)
		return startIPC();
	else
		return startThread();
}

int32_t IPAProxyMaliC55::startThread()
{
	state_ = ProxyRunning;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

int32_t IPAProxyMaliC55::startIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::Start), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;




	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call start: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);






	return _retValue;

}


void IPAProxyMaliC55::stop()
{
	if (isolate_)
		return stopIPC();
	else
		return stopThread();
}

void IPAProxyMaliC55::stopThread()
{
	ASSERT(state_ != ProxyStopping);
	if (state_ != ProxyRunning)
		return;

	state_ = ProxyStopping;

	proxy_.invokeMethod(&ThreadProxy::stop, ConnectionTypeBlocking);

	thread_.exit();
	thread_.wait();

	Thread::current()->dispatchMessages(Message::Type::InvokeMessage, this);

	state_ = ProxyStopped;
}

void IPAProxyMaliC55::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop: " << _ret;
		return;
	}
}


int32_t IPAProxyMaliC55::configure(
	const IPAConfigInfo &configInfo,
	const uint8_t bayerOrder,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return configureIPC(configInfo, bayerOrder, ipaControls);
	else
		return configureThread(configInfo, bayerOrder, ipaControls);
}

int32_t IPAProxyMaliC55::configureThread(
	const IPAConfigInfo &configInfo,
	const uint8_t bayerOrder,
	ControlInfoMap *ipaControls)
{
	return ipa_->configure(configInfo, bayerOrder, ipaControls);

}

int32_t IPAProxyMaliC55::configureIPC(
	const IPAConfigInfo &configInfo,
	const uint8_t bayerOrder,
	ControlInfoMap *ipaControls)
{
	controlSerializer_.reset();
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::Configure), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> configInfoBuf;
	std::tie(configInfoBuf, std::ignore) =
		IPADataSerializer<IPAConfigInfo>::serialize(configInfo
, &controlSerializer_);
	std::vector<uint8_t> bayerOrderBuf;
	std::tie(bayerOrderBuf, std::ignore) =
		IPADataSerializer<uint8_t>::serialize(bayerOrder
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), configInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bayerOrderBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bayerOrderBuf.begin(), bayerOrderBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf, &_ipcOutputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call configure: " << _ret;
		return static_cast<int32_t>(_ret);
	}

	int32_t _retValue = IPADataSerializer<int32_t>::deserialize(_ipcOutputBuf.data(), 0);



	const size_t ipaControlsStart = 4;


	if (ipaControls) {
                *ipaControls =
                IPADataSerializer<ControlInfoMap>::deserialize(
                	_ipcOutputBuf.data().cbegin() + ipaControlsStart,
                	_ipcOutputBuf.data().cend(),
                	&controlSerializer_);
	}


	return _retValue;

}


void IPAProxyMaliC55::mapBuffers(
	const std::vector<libcamera::IPABuffer> &buffers,
	const bool readOnly)
{
	if (isolate_)
		return mapBuffersIPC(buffers, readOnly);
	else
		return mapBuffersThread(buffers, readOnly);
}

void IPAProxyMaliC55::mapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers,
	const bool readOnly)
{
	return ipa_->mapBuffers(buffers, readOnly);

}

void IPAProxyMaliC55::mapBuffersIPC(
	const std::vector<libcamera::IPABuffer> &buffers,
	const bool readOnly)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::MapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> buffersBuf;
	std::vector<SharedFD> buffersFds;
	std::tie(buffersBuf, buffersFds) =
		IPADataSerializer<std::vector<libcamera::IPABuffer>>::serialize(buffers
);
	std::vector<uint8_t> readOnlyBuf;
	std::tie(readOnlyBuf, std::ignore) =
		IPADataSerializer<bool>::serialize(readOnly
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), buffersBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), buffersFds.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), readOnlyBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), buffersBuf.begin(), buffersBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), readOnlyBuf.begin(), readOnlyBuf.end());
	_ipcInputBuf.fds().insert(_ipcInputBuf.fds().end(), buffersFds.begin(), buffersFds.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call mapBuffers: " << _ret;
		return;
	}
}


void IPAProxyMaliC55::unmapBuffers(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	if (isolate_)
		return unmapBuffersIPC(buffers);
	else
		return unmapBuffersThread(buffers);
}

void IPAProxyMaliC55::unmapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	return ipa_->unmapBuffers(buffers);

}

void IPAProxyMaliC55::unmapBuffersIPC(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::UnmapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> buffersBuf;
	std::vector<SharedFD> buffersFds;
	std::tie(buffersBuf, buffersFds) =
		IPADataSerializer<std::vector<libcamera::IPABuffer>>::serialize(buffers
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), buffersBuf.begin(), buffersBuf.end());
	_ipcInputBuf.fds().insert(_ipcInputBuf.fds().end(), buffersFds.begin(), buffersFds.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call unmapBuffers: " << _ret;
		return;
	}
}


void IPAProxyMaliC55::queueRequest(
	const uint32_t request,
	const ControlList &reqControls)
{
	if (isolate_)
		return queueRequestIPC(request, reqControls);
	else
		return queueRequestThread(request, reqControls);
}

void IPAProxyMaliC55::queueRequestThread(
	const uint32_t request,
	const ControlList &reqControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::queueRequest, ConnectionTypeQueued, request, reqControls);
}

void IPAProxyMaliC55::queueRequestIPC(
	const uint32_t request,
	const ControlList &reqControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::QueueRequest), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> requestBuf;
	std::tie(requestBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(request
);
	std::vector<uint8_t> reqControlsBuf;
	std::tie(reqControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(reqControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), requestBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), reqControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), requestBuf.begin(), requestBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), reqControlsBuf.begin(), reqControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call queueRequest: " << _ret;
		return;
	}
}


void IPAProxyMaliC55::fillParams(
	const uint32_t request,
	const uint32_t bufferId)
{
	if (isolate_)
		return fillParamsIPC(request, bufferId);
	else
		return fillParamsThread(request, bufferId);
}

void IPAProxyMaliC55::fillParamsThread(
	const uint32_t request,
	const uint32_t bufferId)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::fillParams, ConnectionTypeQueued, request, bufferId);
}

void IPAProxyMaliC55::fillParamsIPC(
	const uint32_t request,
	const uint32_t bufferId)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::FillParams), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> requestBuf;
	std::tie(requestBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(request
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), requestBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), requestBuf.begin(), requestBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call fillParams: " << _ret;
		return;
	}
}


void IPAProxyMaliC55::processStats(
	const uint32_t request,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	if (isolate_)
		return processStatsIPC(request, bufferId, sensorControls);
	else
		return processStatsThread(request, bufferId, sensorControls);
}

void IPAProxyMaliC55::processStatsThread(
	const uint32_t request,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::processStats, ConnectionTypeQueued, request, bufferId, sensorControls);
}

void IPAProxyMaliC55::processStatsIPC(
	const uint32_t request,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_MaliC55Cmd::ProcessStats), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> requestBuf;
	std::tie(requestBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(request
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), requestBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), requestBuf.begin(), requestBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call processStats: " << _ret;
		return;
	}
}




void IPAProxyMaliC55::paramsComputedThread(
	const uint32_t request,
	const uint32_t bytesused)
{
	ASSERT(state_ != ProxyStopped);
	paramsComputed.emit(request, bytesused);
}

void IPAProxyMaliC55::paramsComputedIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t requestBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t bytesusedBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t requestStart = 8;
	const size_t bytesusedStart = requestStart + requestBufSize;


	uint32_t request =
        IPADataSerializer<uint32_t>::deserialize(
        	data + requestStart,
        	data + requestStart + requestBufSize);

	uint32_t bytesused =
        IPADataSerializer<uint32_t>::deserialize(
        	data + bytesusedStart,
        	data + bytesusedStart + bytesusedBufSize);

	paramsComputed.emit(request, bytesused);
}

void IPAProxyMaliC55::statsProcessedThread(
	const uint32_t request,
	const ControlList &metadata)
{
	ASSERT(state_ != ProxyStopped);
	statsProcessed.emit(request, metadata);
}

void IPAProxyMaliC55::statsProcessedIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t requestBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t metadataBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t requestStart = 8;
	const size_t metadataStart = requestStart + requestBufSize;


	uint32_t request =
        IPADataSerializer<uint32_t>::deserialize(
        	data + requestStart,
        	data + requestStart + requestBufSize);

	ControlList metadata =
        IPADataSerializer<ControlList>::deserialize(
        	data + metadataStart,
        	data + metadataStart + metadataBufSize,
        	&controlSerializer_);

	statsProcessed.emit(request, metadata);
}

void IPAProxyMaliC55::setSensorControlsThread(
	const ControlList &sensorControls)
{
	ASSERT(state_ != ProxyStopped);
	setSensorControls.emit(sensorControls);
}

void IPAProxyMaliC55::setSensorControlsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{


	const size_t sensorControlsStart = 0;


	ControlList sensorControls =
        IPADataSerializer<ControlList>::deserialize(
        	data + sensorControlsStart,
        	data + dataSize,
        	&controlSerializer_);

	setSensorControls.emit(sensorControls);
}


} /* namespace mali_c55 */

} /* namespace ipa */

} /* namespace libcamera */