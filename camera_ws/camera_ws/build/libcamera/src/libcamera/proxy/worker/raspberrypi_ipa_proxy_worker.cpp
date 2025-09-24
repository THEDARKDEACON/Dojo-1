/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy worker for raspberrypi
 *
 * This file is auto-generated. Do not edit.
 */

#include <algorithm>
#include <iostream>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/raspberrypi_ipa_interface.h>
#include <libcamera/ipa/raspberrypi_ipa_serializer.h>
#include <libcamera/logging.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/unique_fd.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/control_serializer.h"
#include "libcamera/internal/ipa_data_serializer.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_pipe_unixsocket.h"
#include "libcamera/internal/ipc_unixsocket.h"

using namespace libcamera;

LOG_DEFINE_CATEGORY(IPAProxyRPiWorker)
using namespace ipa;
using namespace RPi;


class IPAProxyRPiWorker
{
public:
	IPAProxyRPiWorker()
		: ipa_(nullptr),
		  controlSerializer_(ControlSerializer::Role::Worker),
		  exit_(false) {}

	~IPAProxyRPiWorker() {}

	void readyRead()
	{
		IPCUnixSocket::Payload _message;
		int _retRecv = socket_.receive(&_message);
		if (_retRecv) {
			LOG(IPAProxyRPiWorker, Error)
				<< "Receive message failed: " << _retRecv;
			return;
		}

		IPCMessage _ipcMessage(_message);

		_RPiCmd _cmd = static_cast<_RPiCmd>(_ipcMessage.header().cmd);

		switch (_cmd) {
		case _RPiCmd::Exit: {
			exit_ = true;
			break;
		}


		case _RPiCmd::Init: {
		                
                	[[maybe_unused]] const size_t settingsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t paramsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t paramsFdsSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t settingsStart = 12;
                	const size_t paramsStart = settingsStart + settingsBufSize;

                	const size_t paramsFdStart = 0;

                	IPASettings settings =
                        IPADataSerializer<IPASettings>::deserialize(
                        	_ipcMessage.data().cbegin() + settingsStart,
                        	_ipcMessage.data().cbegin() + settingsStart + settingsBufSize);

                	InitParams params =
                        IPADataSerializer<InitParams>::deserialize(
                        	_ipcMessage.data().cbegin() + paramsStart,
                        	_ipcMessage.data().cend(),
                        	_ipcMessage.fds().cbegin() + paramsFdStart,
                        	_ipcMessage.fds().cend());


			InitResult result;

			int32_t _callRet =ipa_->init(settings, params, &result);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> resultBuf;
                	std::tie(resultBuf, std::ignore) =
                		IPADataSerializer<InitResult>::serialize(result
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), resultBuf.begin(), resultBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to init() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to init()";
			break;
		}

		case _RPiCmd::Start: {
		                

                	const size_t controlsStart = 0;


                	ControlList controls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + controlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);


			StartResult result;
ipa_->start(controls, &result);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
                	std::vector<uint8_t> resultBuf;
                	std::tie(resultBuf, std::ignore) =
                		IPADataSerializer<StartResult>::serialize(result
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), resultBuf.begin(), resultBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to start() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to start()";
			break;
		}

		case _RPiCmd::Stop: {
		                



ipa_->stop();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to stop() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to stop()";
			break;
		}

		case _RPiCmd::Configure: {
			controlSerializer_.reset();
		                
                	[[maybe_unused]] const size_t sensorInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t paramsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t paramsFdsSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t sensorInfoStart = 12;
                	const size_t paramsStart = sensorInfoStart + sensorInfoBufSize;

                	const size_t paramsFdStart = 0;

                	IPACameraSensorInfo sensorInfo =
                        IPADataSerializer<IPACameraSensorInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorInfoStart,
                        	_ipcMessage.data().cbegin() + sensorInfoStart + sensorInfoBufSize);

                	ConfigParams params =
                        IPADataSerializer<ConfigParams>::deserialize(
                        	_ipcMessage.data().cbegin() + paramsStart,
                        	_ipcMessage.data().cend(),
                        	_ipcMessage.fds().cbegin() + paramsFdStart,
                        	_ipcMessage.fds().cend(),
                        	&controlSerializer_);


			ConfigResult result;

			int32_t _callRet =ipa_->configure(sensorInfo, params, &result);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> resultBuf;
                	std::tie(resultBuf, std::ignore) =
                		IPADataSerializer<ConfigResult>::serialize(result
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), resultBuf.begin(), resultBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to configure() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to configure()";
			break;
		}

		case _RPiCmd::MapBuffers: {
		                

                	const size_t buffersStart = 0;

                	const size_t buffersFdStart = 0;

                	std::vector<libcamera::IPABuffer> buffers =
                        IPADataSerializer<std::vector<libcamera::IPABuffer>>::deserialize(
                        	_ipcMessage.data().cbegin() + buffersStart,
                        	_ipcMessage.data().cend(),
                        	_ipcMessage.fds().cbegin() + buffersFdStart,
                        	_ipcMessage.fds().cend());

ipa_->mapBuffers(buffers);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to mapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to mapBuffers()";
			break;
		}

		case _RPiCmd::UnmapBuffers: {
		                

                	const size_t idsStart = 0;


                	std::vector<uint32_t> ids =
                        IPADataSerializer<std::vector<uint32_t>>::deserialize(
                        	_ipcMessage.data().cbegin() + idsStart,
                        	_ipcMessage.data().cend());

ipa_->unmapBuffers(ids);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyRPiWorker, Error)
					<< "Reply to unmapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyRPiWorker, Debug) << "Done replying to unmapBuffers()";
			break;
		}

		case _RPiCmd::PrepareIsp: {
		                

                	const size_t paramsStart = 0;


                	PrepareParams params =
                        IPADataSerializer<PrepareParams>::deserialize(
                        	_ipcMessage.data().cbegin() + paramsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->prepareIsp(params);

			break;
		}

		case _RPiCmd::ProcessStats: {
		                

                	const size_t paramsStart = 0;


                	ProcessParams params =
                        IPADataSerializer<ProcessParams>::deserialize(
                        	_ipcMessage.data().cbegin() + paramsStart,
                        	_ipcMessage.data().cend());

ipa_->processStats(params);

			break;
		}

		default:
			LOG(IPAProxyRPiWorker, Error) << "Unknown command " << _ipcMessage.header().cmd;
		}
	}

	int init(std::unique_ptr<IPAModule> &ipam, UniqueFD socketfd)
	{
		if (socket_.bind(std::move(socketfd)) < 0) {
			LOG(IPAProxyRPiWorker, Error)
				<< "IPC socket binding failed";
			return EXIT_FAILURE;
		}
		socket_.readyRead.connect(this, &IPAProxyRPiWorker::readyRead);

		ipa_ = dynamic_cast<IPARPiInterface *>(ipam->createInterface());
		if (!ipa_) {
			LOG(IPAProxyRPiWorker, Error)
				<< "Failed to create IPA interface instance";
			return EXIT_FAILURE;
		}

		ipa_->prepareIspComplete.connect(this, &IPAProxyRPiWorker::prepareIspComplete);
		ipa_->processStatsComplete.connect(this, &IPAProxyRPiWorker::processStatsComplete);
		ipa_->metadataReady.connect(this, &IPAProxyRPiWorker::metadataReady);
		ipa_->setIspControls.connect(this, &IPAProxyRPiWorker::setIspControls);
		ipa_->setDelayedControls.connect(this, &IPAProxyRPiWorker::setDelayedControls);
		ipa_->setLensControls.connect(this, &IPAProxyRPiWorker::setLensControls);
		ipa_->setCameraTimeout.connect(this, &IPAProxyRPiWorker::setCameraTimeout);
		return 0;
	}

	void run()
	{
		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();
		while (!exit_)
			dispatcher->processEvents();
	}

	void cleanup()
	{
		delete ipa_;
		socket_.close();
	}

private:


        void prepareIspComplete(
        	const BufferIds &buffers,
        	const bool stitchSwapBuffers)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::PrepareIspComplete),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> buffersBuf;
	std::tie(buffersBuf, std::ignore) =
		IPADataSerializer<BufferIds>::serialize(buffers
);
	std::vector<uint8_t> stitchSwapBuffersBuf;
	std::tie(stitchSwapBuffersBuf, std::ignore) =
		IPADataSerializer<bool>::serialize(stitchSwapBuffers
);
	appendPOD<uint32_t>(_message.data(), buffersBuf.size());
	appendPOD<uint32_t>(_message.data(), stitchSwapBuffersBuf.size());
	_message.data().insert(_message.data().end(), buffersBuf.begin(), buffersBuf.end());
	_message.data().insert(_message.data().end(), stitchSwapBuffersBuf.begin(), stitchSwapBuffersBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event prepareIspComplete() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "prepareIspComplete done";
	}

        void processStatsComplete(
        	const BufferIds &buffers)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::ProcessStatsComplete),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> buffersBuf;
	std::tie(buffersBuf, std::ignore) =
		IPADataSerializer<BufferIds>::serialize(buffers
);
	_message.data().insert(_message.data().end(), buffersBuf.begin(), buffersBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event processStatsComplete() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "processStatsComplete done";
	}

        void metadataReady(
        	const ControlList &metadata)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::MetadataReady),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> metadataBuf;
	std::tie(metadataBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(metadata
, &controlSerializer_);
	_message.data().insert(_message.data().end(), metadataBuf.begin(), metadataBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event metadataReady() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "metadataReady done";
	}

        void setIspControls(
        	const ControlList &controls)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::SetIspControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> controlsBuf;
	std::tie(controlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(controls
, &controlSerializer_);
	_message.data().insert(_message.data().end(), controlsBuf.begin(), controlsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event setIspControls() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "setIspControls done";
	}

        void setDelayedControls(
        	const ControlList &controls,
        	const uint32_t delayContext)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::SetDelayedControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> controlsBuf;
	std::tie(controlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(controls
, &controlSerializer_);
	std::vector<uint8_t> delayContextBuf;
	std::tie(delayContextBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(delayContext
);
	appendPOD<uint32_t>(_message.data(), controlsBuf.size());
	appendPOD<uint32_t>(_message.data(), delayContextBuf.size());
	_message.data().insert(_message.data().end(), controlsBuf.begin(), controlsBuf.end());
	_message.data().insert(_message.data().end(), delayContextBuf.begin(), delayContextBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event setDelayedControls() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "setDelayedControls done";
	}

        void setLensControls(
        	const ControlList &controls)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::SetLensControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> controlsBuf;
	std::tie(controlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(controls
, &controlSerializer_);
	_message.data().insert(_message.data().end(), controlsBuf.begin(), controlsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event setLensControls() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "setLensControls done";
	}

        void setCameraTimeout(
        	const uint32_t maxFrameLengthMs)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_RPiEventCmd::SetCameraTimeout),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> maxFrameLengthMsBuf;
	std::tie(maxFrameLengthMsBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(maxFrameLengthMs
);
	_message.data().insert(_message.data().end(), maxFrameLengthMsBuf.begin(), maxFrameLengthMsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyRPiWorker, Error)
				<< "Sending event setCameraTimeout() failed: " << _ret;

		LOG(IPAProxyRPiWorker, Debug) << "setCameraTimeout done";
	}


	IPARPiInterface *ipa_;
	IPCUnixSocket socket_;

	ControlSerializer controlSerializer_;

	bool exit_;
};

int main(int argc, char **argv)
{
	/* Uncomment this for debugging. */
#if 0
	std::string logPath = "/tmp/libcamera.worker." +
			      std::to_string(getpid()) + ".log";
	logSetFile(logPath.c_str());
#endif

	if (argc < 3) {
		LOG(IPAProxyRPiWorker, Error)
			<< "Tried to start worker with no args: "
			<< "expected <path to IPA so> <fd to bind unix socket>";
		return EXIT_FAILURE;
	}

	UniqueFD fd(std::stoi(argv[2]));
	LOG(IPAProxyRPiWorker, Info)
		<< "Starting worker for IPA module " << argv[1]
		<< " with IPC fd = " << fd.get();

	std::unique_ptr<IPAModule> ipam = std::make_unique<IPAModule>(argv[1]);
	if (!ipam->isValid() || !ipam->load()) {
		LOG(IPAProxyRPiWorker, Error)
			<< "IPAModule " << argv[1] << " isn't valid";
		return EXIT_FAILURE;
	}

	/*
	 * Shutdown of proxy worker can be pre-empted by events like
	 * SIGINT/SIGTERM, even before the pipeline handler can request
	 * shutdown. Hence, assign a new gid to prevent signals on the
	 * application being delivered to the proxy.
	 */
	if (setpgid(0, 0) < 0) {
		int err = errno;
		LOG(IPAProxyRPiWorker, Warning)
			<< "Failed to set new gid: " << strerror(err);
	}

	IPAProxyRPiWorker proxyWorker;
	int ret = proxyWorker.init(ipam, std::move(fd));
	if (ret < 0) {
		LOG(IPAProxyRPiWorker, Error)
			<< "Failed to initialize proxy worker";
		return ret;
	}

	LOG(IPAProxyRPiWorker, Debug) << "Proxy worker successfully initialized";

	proxyWorker.run();

	proxyWorker.cleanup();

	return 0;
}