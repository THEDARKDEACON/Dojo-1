/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy for rkisp1
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/ipa/rkisp1_ipa_proxy.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/rkisp1_ipa_interface.h>
#include <libcamera/ipa/rkisp1_ipa_serializer.h>

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

namespace rkisp1 {


IPAProxyRkISP1::IPAProxyRkISP1(IPAModule *ipam, bool isolate)
	: IPAProxy(ipam), isolate_(isolate),
	  controlSerializer_(ControlSerializer::Role::Proxy), seq_(0)
{
	LOG(IPAProxy, Debug)
		<< "initializing rkisp1 proxy: loading IPA from "
		<< ipam->path();

	if (isolate_) {
		const std::string proxyWorkerPath = resolvePath("rkisp1_ipa_proxy");
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

		ipc->recv.connect(this, &IPAProxyRkISP1::recvMessage);

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

	ipa_ = std::unique_ptr<IPARkISP1Interface>(static_cast<IPARkISP1Interface *>(ipai));
	proxy_.setIPA(ipa_.get());


	ipa_->paramsComputed.connect(this, &IPAProxyRkISP1::paramsComputedThread);
	ipa_->setSensorControls.connect(this, &IPAProxyRkISP1::setSensorControlsThread);
	ipa_->metadataReady.connect(this, &IPAProxyRkISP1::metadataReadyThread);

	valid_ = true;
}

IPAProxyRkISP1::~IPAProxyRkISP1()
{
	if (ipc_) {
		IPCMessage::Header header =
			{ static_cast<uint32_t>(_RkISP1Cmd::Exit), seq_++ };
		IPCMessage msg(header);
		ipc_->sendAsync(msg);
	}
}


void IPAProxyRkISP1::recvMessage(const IPCMessage &data)
{
	size_t dataSize = data.data().size();
	_RkISP1EventCmd _cmd = static_cast<_RkISP1EventCmd>(data.header().cmd);

	switch (_cmd) {
	case _RkISP1EventCmd::ParamsComputed: {
		paramsComputedIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _RkISP1EventCmd::SetSensorControls: {
		setSensorControlsIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	case _RkISP1EventCmd::MetadataReady: {
		metadataReadyIPC(data.data().cbegin(), dataSize, data.fds());
		break;
	}
	default:
		LOG(IPAProxy, Error) << "Unknown command " << static_cast<uint32_t>(_cmd);
	}
}


int32_t IPAProxyRkISP1::init(
	const IPASettings &settings,
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return initIPC(settings, hwRevision, sensorInfo, sensorControls, ipaControls);
	else
		return initThread(settings, hwRevision, sensorInfo, sensorControls, ipaControls);
}

int32_t IPAProxyRkISP1::initThread(
	const IPASettings &settings,
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls)
{
	int32_t _ret = ipa_->init(settings, hwRevision, sensorInfo, sensorControls, ipaControls);

	proxy_.moveToThread(&thread_);

	return _ret;
}

int32_t IPAProxyRkISP1::initIPC(
	const IPASettings &settings,
	const uint32_t hwRevision,
	const IPACameraSensorInfo &sensorInfo,
	const ControlInfoMap &sensorControls,
	ControlInfoMap *ipaControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::Init), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> settingsBuf;
	std::tie(settingsBuf, std::ignore) =
		IPADataSerializer<IPASettings>::serialize(settings
);
	std::vector<uint8_t> hwRevisionBuf;
	std::tie(hwRevisionBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(hwRevision
);
	std::vector<uint8_t> sensorInfoBuf;
	std::tie(sensorInfoBuf, std::ignore) =
		IPADataSerializer<IPACameraSensorInfo>::serialize(sensorInfo
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlInfoMap>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), settingsBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), hwRevisionBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), settingsBuf.begin(), settingsBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), hwRevisionBuf.begin(), hwRevisionBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorInfoBuf.begin(), sensorInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


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


int32_t IPAProxyRkISP1::start()
{
	if (isolate_)
		return startIPC();
	else
		return startThread();
}

int32_t IPAProxyRkISP1::startThread()
{
	state_ = ProxyRunning;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

int32_t IPAProxyRkISP1::startIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::Start), seq_++ };
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


void IPAProxyRkISP1::stop()
{
	if (isolate_)
		return stopIPC();
	else
		return stopThread();
}

void IPAProxyRkISP1::stopThread()
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

void IPAProxyRkISP1::stopIPC()
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::Stop), seq_++ };
	IPCMessage _ipcInputBuf(_header);




	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call stop: " << _ret;
		return;
	}
}


int32_t IPAProxyRkISP1::configure(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	ControlInfoMap *ipaControls)
{
	if (isolate_)
		return configureIPC(configInfo, streamConfig, ipaControls);
	else
		return configureThread(configInfo, streamConfig, ipaControls);
}

int32_t IPAProxyRkISP1::configureThread(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	ControlInfoMap *ipaControls)
{
	return ipa_->configure(configInfo, streamConfig, ipaControls);

}

int32_t IPAProxyRkISP1::configureIPC(
	const IPAConfigInfo &configInfo,
	const std::map<uint32_t, libcamera::IPAStream> &streamConfig,
	ControlInfoMap *ipaControls)
{
	controlSerializer_.reset();
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::Configure), seq_++ };
	IPCMessage _ipcInputBuf(_header);
	IPCMessage _ipcOutputBuf;


	std::vector<uint8_t> configInfoBuf;
	std::tie(configInfoBuf, std::ignore) =
		IPADataSerializer<IPAConfigInfo>::serialize(configInfo
, &controlSerializer_);
	std::vector<uint8_t> streamConfigBuf;
	std::tie(streamConfigBuf, std::ignore) =
		IPADataSerializer<std::map<uint32_t, libcamera::IPAStream>>::serialize(streamConfig
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), configInfoBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), streamConfigBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), configInfoBuf.begin(), configInfoBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), streamConfigBuf.begin(), streamConfigBuf.end());


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


void IPAProxyRkISP1::mapBuffers(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	if (isolate_)
		return mapBuffersIPC(buffers);
	else
		return mapBuffersThread(buffers);
}

void IPAProxyRkISP1::mapBuffersThread(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	return ipa_->mapBuffers(buffers);

}

void IPAProxyRkISP1::mapBuffersIPC(
	const std::vector<libcamera::IPABuffer> &buffers)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::MapBuffers), seq_++ };
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
		LOG(IPAProxy, Error) << "Failed to call mapBuffers: " << _ret;
		return;
	}
}


void IPAProxyRkISP1::unmapBuffers(
	const std::vector<uint32_t> &ids)
{
	if (isolate_)
		return unmapBuffersIPC(ids);
	else
		return unmapBuffersThread(ids);
}

void IPAProxyRkISP1::unmapBuffersThread(
	const std::vector<uint32_t> &ids)
{
	return ipa_->unmapBuffers(ids);

}

void IPAProxyRkISP1::unmapBuffersIPC(
	const std::vector<uint32_t> &ids)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::UnmapBuffers), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> idsBuf;
	std::tie(idsBuf, std::ignore) =
		IPADataSerializer<std::vector<uint32_t>>::serialize(ids
);
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), idsBuf.begin(), idsBuf.end());


	int _ret = ipc_->sendSync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call unmapBuffers: " << _ret;
		return;
	}
}


void IPAProxyRkISP1::queueRequest(
	const uint32_t frame,
	const ControlList &reqControls)
{
	if (isolate_)
		return queueRequestIPC(frame, reqControls);
	else
		return queueRequestThread(frame, reqControls);
}

void IPAProxyRkISP1::queueRequestThread(
	const uint32_t frame,
	const ControlList &reqControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::queueRequest, ConnectionTypeQueued, frame, reqControls);
}

void IPAProxyRkISP1::queueRequestIPC(
	const uint32_t frame,
	const ControlList &reqControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::QueueRequest), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> reqControlsBuf;
	std::tie(reqControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(reqControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), reqControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), reqControlsBuf.begin(), reqControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call queueRequest: " << _ret;
		return;
	}
}


void IPAProxyRkISP1::computeParams(
	const uint32_t frame,
	const uint32_t bufferId)
{
	if (isolate_)
		return computeParamsIPC(frame, bufferId);
	else
		return computeParamsThread(frame, bufferId);
}

void IPAProxyRkISP1::computeParamsThread(
	const uint32_t frame,
	const uint32_t bufferId)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::computeParams, ConnectionTypeQueued, frame, bufferId);
}

void IPAProxyRkISP1::computeParamsIPC(
	const uint32_t frame,
	const uint32_t bufferId)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::ComputeParams), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call computeParams: " << _ret;
		return;
	}
}


void IPAProxyRkISP1::processStats(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	if (isolate_)
		return processStatsIPC(frame, bufferId, sensorControls);
	else
		return processStatsThread(frame, bufferId, sensorControls);
}

void IPAProxyRkISP1::processStatsThread(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	ASSERT(state_ == ProxyRunning);
	proxy_.invokeMethod(&ThreadProxy::processStats, ConnectionTypeQueued, frame, bufferId, sensorControls);
}

void IPAProxyRkISP1::processStatsIPC(
	const uint32_t frame,
	const uint32_t bufferId,
	const ControlList &sensorControls)
{
	IPCMessage::Header _header = { static_cast<uint32_t>(_RkISP1Cmd::ProcessStats), seq_++ };
	IPCMessage _ipcInputBuf(_header);


	std::vector<uint8_t> frameBuf;
	std::tie(frameBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(frame
);
	std::vector<uint8_t> bufferIdBuf;
	std::tie(bufferIdBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bufferId
);
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	appendPOD<uint32_t>(_ipcInputBuf.data(), frameBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), bufferIdBuf.size());
	appendPOD<uint32_t>(_ipcInputBuf.data(), sensorControlsBuf.size());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), frameBuf.begin(), frameBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), bufferIdBuf.begin(), bufferIdBuf.end());
	_ipcInputBuf.data().insert(_ipcInputBuf.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());


	int _ret = ipc_->sendAsync(_ipcInputBuf);
	if (_ret < 0) {
		LOG(IPAProxy, Error) << "Failed to call processStats: " << _ret;
		return;
	}
}




void IPAProxyRkISP1::paramsComputedThread(
	const uint32_t frame,
	const uint32_t bytesused)
{
	ASSERT(state_ != ProxyStopped);
	paramsComputed.emit(frame, bytesused);
}

void IPAProxyRkISP1::paramsComputedIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t bytesusedBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t bytesusedStart = frameStart + frameBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	uint32_t bytesused =
        IPADataSerializer<uint32_t>::deserialize(
        	data + bytesusedStart,
        	data + bytesusedStart + bytesusedBufSize);

	paramsComputed.emit(frame, bytesused);
}

void IPAProxyRkISP1::setSensorControlsThread(
	const uint32_t frame,
	const ControlList &sensorControls)
{
	ASSERT(state_ != ProxyStopped);
	setSensorControls.emit(frame, sensorControls);
}

void IPAProxyRkISP1::setSensorControlsIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t sensorControlsStart = frameStart + frameBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	ControlList sensorControls =
        IPADataSerializer<ControlList>::deserialize(
        	data + sensorControlsStart,
        	data + sensorControlsStart + sensorControlsBufSize,
        	&controlSerializer_);

	setSensorControls.emit(frame, sensorControls);
}

void IPAProxyRkISP1::metadataReadyThread(
	const uint32_t frame,
	const ControlList &metadata)
{
	ASSERT(state_ != ProxyStopped);
	metadataReady.emit(frame, metadata);
}

void IPAProxyRkISP1::metadataReadyIPC(
	[[maybe_unused]] std::vector<uint8_t>::const_iterator data,
	[[maybe_unused]] size_t dataSize,
	[[maybe_unused]] const std::vector<SharedFD> &fds)
{

	[[maybe_unused]] const size_t frameBufSize = readPOD<uint32_t>(data, 0, data + dataSize);
	[[maybe_unused]] const size_t metadataBufSize = readPOD<uint32_t>(data, 4, data + dataSize);

	const size_t frameStart = 8;
	const size_t metadataStart = frameStart + frameBufSize;


	uint32_t frame =
        IPADataSerializer<uint32_t>::deserialize(
        	data + frameStart,
        	data + frameStart + frameBufSize);

	ControlList metadata =
        IPADataSerializer<ControlList>::deserialize(
        	data + metadataStart,
        	data + metadataStart + metadataBufSize,
        	&controlSerializer_);

	metadataReady.emit(frame, metadata);
}


} /* namespace rkisp1 */

} /* namespace ipa */

} /* namespace libcamera */