/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm proxy worker for mali-c55
 *
 * This file is auto-generated. Do not edit.
 */

#include <algorithm>
#include <iostream>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>
#include <libcamera/ipa/mali-c55_ipa_serializer.h>
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

LOG_DEFINE_CATEGORY(IPAProxyMaliC55Worker)
using namespace ipa;
using namespace mali_c55;


class IPAProxyMaliC55Worker
{
public:
	IPAProxyMaliC55Worker()
		: ipa_(nullptr),
		  controlSerializer_(ControlSerializer::Role::Worker),
		  exit_(false) {}

	~IPAProxyMaliC55Worker() {}

	void readyRead()
	{
		IPCUnixSocket::Payload _message;
		int _retRecv = socket_.receive(&_message);
		if (_retRecv) {
			LOG(IPAProxyMaliC55Worker, Error)
				<< "Receive message failed: " << _retRecv;
			return;
		}

		IPCMessage _ipcMessage(_message);

		_MaliC55Cmd _cmd = static_cast<_MaliC55Cmd>(_ipcMessage.header().cmd);

		switch (_cmd) {
		case _MaliC55Cmd::Exit: {
			exit_ = true;
			break;
		}


		case _MaliC55Cmd::Init: {
		                
                	[[maybe_unused]] const size_t settingsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t configInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t settingsStart = 8;
                	const size_t configInfoStart = settingsStart + settingsBufSize;


                	IPASettings settings =
                        IPADataSerializer<IPASettings>::deserialize(
                        	_ipcMessage.data().cbegin() + settingsStart,
                        	_ipcMessage.data().cbegin() + settingsStart + settingsBufSize);

                	IPAConfigInfo configInfo =
                        IPADataSerializer<IPAConfigInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + configInfoStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);


			ControlInfoMap ipaControls;

			int32_t _callRet =ipa_->init(settings, configInfo, &ipaControls);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> ipaControlsBuf;
                	std::tie(ipaControlsBuf, std::ignore) =
                		IPADataSerializer<ControlInfoMap>::serialize(ipaControls
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), ipaControlsBuf.begin(), ipaControlsBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to init() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to init()";
			break;
		}

		case _MaliC55Cmd::Start: {
		                




			int32_t _callRet =ipa_->start();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to start() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to start()";
			break;
		}

		case _MaliC55Cmd::Stop: {
		                



ipa_->stop();

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to stop() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to stop()";
			break;
		}

		case _MaliC55Cmd::Configure: {
			controlSerializer_.reset();
		                
                	[[maybe_unused]] const size_t configInfoBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t bayerOrderBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t configInfoStart = 8;
                	const size_t bayerOrderStart = configInfoStart + configInfoBufSize;


                	IPAConfigInfo configInfo =
                        IPADataSerializer<IPAConfigInfo>::deserialize(
                        	_ipcMessage.data().cbegin() + configInfoStart,
                        	_ipcMessage.data().cbegin() + configInfoStart + configInfoBufSize,
                        	&controlSerializer_);

                	uint8_t bayerOrder =
                        IPADataSerializer<uint8_t>::deserialize(
                        	_ipcMessage.data().cbegin() + bayerOrderStart,
                        	_ipcMessage.data().cend());


			ControlInfoMap ipaControls;

			int32_t _callRet =ipa_->configure(configInfo, bayerOrder, &ipaControls);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
			std::vector<uint8_t> _callRetBuf;
			std::tie(_callRetBuf, std::ignore) =
				IPADataSerializer<int32_t>::serialize(_callRet);
			_response.data().insert(_response.data().end(), _callRetBuf.cbegin(), _callRetBuf.cend());
		                
                	std::vector<uint8_t> ipaControlsBuf;
                	std::tie(ipaControlsBuf, std::ignore) =
                		IPADataSerializer<ControlInfoMap>::serialize(ipaControls
                , &controlSerializer_);
                	_response.data().insert(_response.data().end(), ipaControlsBuf.begin(), ipaControlsBuf.end());
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to configure() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to configure()";
			break;
		}

		case _MaliC55Cmd::MapBuffers: {
		                
                	[[maybe_unused]] const size_t buffersBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t buffersFdsSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t readOnlyBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t buffersStart = 12;
                	const size_t readOnlyStart = buffersStart + buffersBufSize;

                	const size_t buffersFdStart = 0;

                	std::vector<libcamera::IPABuffer> buffers =
                        IPADataSerializer<std::vector<libcamera::IPABuffer>>::deserialize(
                        	_ipcMessage.data().cbegin() + buffersStart,
                        	_ipcMessage.data().cbegin() + buffersStart + buffersBufSize,
                        	_ipcMessage.fds().cbegin() + buffersFdStart,
                        	_ipcMessage.fds().cbegin() + buffersFdStart + buffersFdsSize);

                	bool readOnly =
                        IPADataSerializer<bool>::deserialize(
                        	_ipcMessage.data().cbegin() + readOnlyStart,
                        	_ipcMessage.data().cend());

ipa_->mapBuffers(buffers, readOnly);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to mapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to mapBuffers()";
			break;
		}

		case _MaliC55Cmd::UnmapBuffers: {
		                

                	const size_t buffersStart = 0;

                	const size_t buffersFdStart = 0;

                	std::vector<libcamera::IPABuffer> buffers =
                        IPADataSerializer<std::vector<libcamera::IPABuffer>>::deserialize(
                        	_ipcMessage.data().cbegin() + buffersStart,
                        	_ipcMessage.data().cend(),
                        	_ipcMessage.fds().cbegin() + buffersFdStart,
                        	_ipcMessage.fds().cend());

ipa_->unmapBuffers(buffers);

			IPCMessage::Header header = { _ipcMessage.header().cmd, _ipcMessage.header().cookie };
			IPCMessage _response(header);
		                
			int _ret = socket_.send(_response.payload());
			if (_ret < 0) {
				LOG(IPAProxyMaliC55Worker, Error)
					<< "Reply to unmapBuffers() failed: " << _ret;
			}
			LOG(IPAProxyMaliC55Worker, Debug) << "Done replying to unmapBuffers()";
			break;
		}

		case _MaliC55Cmd::QueueRequest: {
		                
                	[[maybe_unused]] const size_t requestBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t reqControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t requestStart = 8;
                	const size_t reqControlsStart = requestStart + requestBufSize;


                	uint32_t request =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + requestStart,
                        	_ipcMessage.data().cbegin() + requestStart + requestBufSize);

                	ControlList reqControls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + reqControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->queueRequest(request, reqControls);

			break;
		}

		case _MaliC55Cmd::FillParams: {
		                
                	[[maybe_unused]] const size_t requestBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t bufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);

                	const size_t requestStart = 8;
                	const size_t bufferIdStart = requestStart + requestBufSize;


                	uint32_t request =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + requestStart,
                        	_ipcMessage.data().cbegin() + requestStart + requestBufSize);

                	uint32_t bufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + bufferIdStart,
                        	_ipcMessage.data().cend());

ipa_->fillParams(request, bufferId);

			break;
		}

		case _MaliC55Cmd::ProcessStats: {
		                
                	[[maybe_unused]] const size_t requestBufSize = readPOD<uint32_t>(_ipcMessage.data(), 0);
                	[[maybe_unused]] const size_t bufferIdBufSize = readPOD<uint32_t>(_ipcMessage.data(), 4);
                	[[maybe_unused]] const size_t sensorControlsBufSize = readPOD<uint32_t>(_ipcMessage.data(), 8);

                	const size_t requestStart = 12;
                	const size_t bufferIdStart = requestStart + requestBufSize;
                	const size_t sensorControlsStart = bufferIdStart + bufferIdBufSize;


                	uint32_t request =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + requestStart,
                        	_ipcMessage.data().cbegin() + requestStart + requestBufSize);

                	uint32_t bufferId =
                        IPADataSerializer<uint32_t>::deserialize(
                        	_ipcMessage.data().cbegin() + bufferIdStart,
                        	_ipcMessage.data().cbegin() + bufferIdStart + bufferIdBufSize);

                	ControlList sensorControls =
                        IPADataSerializer<ControlList>::deserialize(
                        	_ipcMessage.data().cbegin() + sensorControlsStart,
                        	_ipcMessage.data().cend(),
                        	&controlSerializer_);

ipa_->processStats(request, bufferId, sensorControls);

			break;
		}

		default:
			LOG(IPAProxyMaliC55Worker, Error) << "Unknown command " << _ipcMessage.header().cmd;
		}
	}

	int init(std::unique_ptr<IPAModule> &ipam, UniqueFD socketfd)
	{
		if (socket_.bind(std::move(socketfd)) < 0) {
			LOG(IPAProxyMaliC55Worker, Error)
				<< "IPC socket binding failed";
			return EXIT_FAILURE;
		}
		socket_.readyRead.connect(this, &IPAProxyMaliC55Worker::readyRead);

		ipa_ = dynamic_cast<IPAMaliC55Interface *>(ipam->createInterface());
		if (!ipa_) {
			LOG(IPAProxyMaliC55Worker, Error)
				<< "Failed to create IPA interface instance";
			return EXIT_FAILURE;
		}

		ipa_->paramsComputed.connect(this, &IPAProxyMaliC55Worker::paramsComputed);
		ipa_->statsProcessed.connect(this, &IPAProxyMaliC55Worker::statsProcessed);
		ipa_->setSensorControls.connect(this, &IPAProxyMaliC55Worker::setSensorControls);
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


        void paramsComputed(
        	const uint32_t request,
        	const uint32_t bytesused)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_MaliC55EventCmd::ParamsComputed),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> requestBuf;
	std::tie(requestBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(request
);
	std::vector<uint8_t> bytesusedBuf;
	std::tie(bytesusedBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(bytesused
);
	appendPOD<uint32_t>(_message.data(), requestBuf.size());
	appendPOD<uint32_t>(_message.data(), bytesusedBuf.size());
	_message.data().insert(_message.data().end(), requestBuf.begin(), requestBuf.end());
	_message.data().insert(_message.data().end(), bytesusedBuf.begin(), bytesusedBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyMaliC55Worker, Error)
				<< "Sending event paramsComputed() failed: " << _ret;

		LOG(IPAProxyMaliC55Worker, Debug) << "paramsComputed done";
	}

        void statsProcessed(
        	const uint32_t request,
        	const ControlList &metadata)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_MaliC55EventCmd::StatsProcessed),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> requestBuf;
	std::tie(requestBuf, std::ignore) =
		IPADataSerializer<uint32_t>::serialize(request
);
	std::vector<uint8_t> metadataBuf;
	std::tie(metadataBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(metadata
, &controlSerializer_);
	appendPOD<uint32_t>(_message.data(), requestBuf.size());
	appendPOD<uint32_t>(_message.data(), metadataBuf.size());
	_message.data().insert(_message.data().end(), requestBuf.begin(), requestBuf.end());
	_message.data().insert(_message.data().end(), metadataBuf.begin(), metadataBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyMaliC55Worker, Error)
				<< "Sending event statsProcessed() failed: " << _ret;

		LOG(IPAProxyMaliC55Worker, Debug) << "statsProcessed done";
	}

        void setSensorControls(
        	const ControlList &sensorControls)
	{
		IPCMessage::Header header = {
			static_cast<uint32_t>(_MaliC55EventCmd::SetSensorControls),
			0
		};
		IPCMessage _message(header);

		
	std::vector<uint8_t> sensorControlsBuf;
	std::tie(sensorControlsBuf, std::ignore) =
		IPADataSerializer<ControlList>::serialize(sensorControls
, &controlSerializer_);
	_message.data().insert(_message.data().end(), sensorControlsBuf.begin(), sensorControlsBuf.end());

		int _ret = socket_.send(_message.payload());
		if (_ret < 0)
			LOG(IPAProxyMaliC55Worker, Error)
				<< "Sending event setSensorControls() failed: " << _ret;

		LOG(IPAProxyMaliC55Worker, Debug) << "setSensorControls done";
	}


	IPAMaliC55Interface *ipa_;
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
		LOG(IPAProxyMaliC55Worker, Error)
			<< "Tried to start worker with no args: "
			<< "expected <path to IPA so> <fd to bind unix socket>";
		return EXIT_FAILURE;
	}

	UniqueFD fd(std::stoi(argv[2]));
	LOG(IPAProxyMaliC55Worker, Info)
		<< "Starting worker for IPA module " << argv[1]
		<< " with IPC fd = " << fd.get();

	std::unique_ptr<IPAModule> ipam = std::make_unique<IPAModule>(argv[1]);
	if (!ipam->isValid() || !ipam->load()) {
		LOG(IPAProxyMaliC55Worker, Error)
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
		LOG(IPAProxyMaliC55Worker, Warning)
			<< "Failed to set new gid: " << strerror(err);
	}

	IPAProxyMaliC55Worker proxyWorker;
	int ret = proxyWorker.init(ipam, std::move(fd));
	if (ret < 0) {
		LOG(IPAProxyMaliC55Worker, Error)
			<< "Failed to initialize proxy worker";
		return ret;
	}

	LOG(IPAProxyMaliC55Worker, Debug) << "Proxy worker successfully initialized";

	proxyWorker.run();

	proxyWorker.cleanup();

	return 0;
}