/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Auto-generated formats
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/formats.h>

#include <pybind11/pybind11.h>

#include "py_main.h"

namespace py = pybind11;

class PyFormats
{
};

void init_py_formats_generated(py::module& m)
{
	py::class_<PyFormats>(m, "formats")
		.def_readonly_static("R8", &libcamera::formats::R8)
		.def_readonly_static("R10", &libcamera::formats::R10)
		.def_readonly_static("R12", &libcamera::formats::R12)
		.def_readonly_static("R16", &libcamera::formats::R16)
		.def_readonly_static("RGB565", &libcamera::formats::RGB565)
		.def_readonly_static("RGB565_BE", &libcamera::formats::RGB565_BE)
		.def_readonly_static("RGB888", &libcamera::formats::RGB888)
		.def_readonly_static("BGR888", &libcamera::formats::BGR888)
		.def_readonly_static("XRGB8888", &libcamera::formats::XRGB8888)
		.def_readonly_static("XBGR8888", &libcamera::formats::XBGR8888)
		.def_readonly_static("RGBX8888", &libcamera::formats::RGBX8888)
		.def_readonly_static("BGRX8888", &libcamera::formats::BGRX8888)
		.def_readonly_static("ARGB8888", &libcamera::formats::ARGB8888)
		.def_readonly_static("ABGR8888", &libcamera::formats::ABGR8888)
		.def_readonly_static("RGBA8888", &libcamera::formats::RGBA8888)
		.def_readonly_static("BGRA8888", &libcamera::formats::BGRA8888)
		.def_readonly_static("RGB161616", &libcamera::formats::RGB161616)
		.def_readonly_static("BGR161616", &libcamera::formats::BGR161616)
		.def_readonly_static("YUYV", &libcamera::formats::YUYV)
		.def_readonly_static("YVYU", &libcamera::formats::YVYU)
		.def_readonly_static("UYVY", &libcamera::formats::UYVY)
		.def_readonly_static("VYUY", &libcamera::formats::VYUY)
		.def_readonly_static("AVUY8888", &libcamera::formats::AVUY8888)
		.def_readonly_static("XVUY8888", &libcamera::formats::XVUY8888)
		.def_readonly_static("NV12", &libcamera::formats::NV12)
		.def_readonly_static("NV21", &libcamera::formats::NV21)
		.def_readonly_static("NV16", &libcamera::formats::NV16)
		.def_readonly_static("NV61", &libcamera::formats::NV61)
		.def_readonly_static("NV24", &libcamera::formats::NV24)
		.def_readonly_static("NV42", &libcamera::formats::NV42)
		.def_readonly_static("YUV420", &libcamera::formats::YUV420)
		.def_readonly_static("YVU420", &libcamera::formats::YVU420)
		.def_readonly_static("YUV422", &libcamera::formats::YUV422)
		.def_readonly_static("YVU422", &libcamera::formats::YVU422)
		.def_readonly_static("YUV444", &libcamera::formats::YUV444)
		.def_readonly_static("YVU444", &libcamera::formats::YVU444)
		.def_readonly_static("MJPEG", &libcamera::formats::MJPEG)
		.def_readonly_static("SRGGB8", &libcamera::formats::SRGGB8)
		.def_readonly_static("SGRBG8", &libcamera::formats::SGRBG8)
		.def_readonly_static("SGBRG8", &libcamera::formats::SGBRG8)
		.def_readonly_static("SBGGR8", &libcamera::formats::SBGGR8)
		.def_readonly_static("SRGGB10", &libcamera::formats::SRGGB10)
		.def_readonly_static("SGRBG10", &libcamera::formats::SGRBG10)
		.def_readonly_static("SGBRG10", &libcamera::formats::SGBRG10)
		.def_readonly_static("SBGGR10", &libcamera::formats::SBGGR10)
		.def_readonly_static("SRGGB12", &libcamera::formats::SRGGB12)
		.def_readonly_static("SGRBG12", &libcamera::formats::SGRBG12)
		.def_readonly_static("SGBRG12", &libcamera::formats::SGBRG12)
		.def_readonly_static("SBGGR12", &libcamera::formats::SBGGR12)
		.def_readonly_static("SRGGB14", &libcamera::formats::SRGGB14)
		.def_readonly_static("SGRBG14", &libcamera::formats::SGRBG14)
		.def_readonly_static("SGBRG14", &libcamera::formats::SGBRG14)
		.def_readonly_static("SBGGR14", &libcamera::formats::SBGGR14)
		.def_readonly_static("SRGGB16", &libcamera::formats::SRGGB16)
		.def_readonly_static("SGRBG16", &libcamera::formats::SGRBG16)
		.def_readonly_static("SGBRG16", &libcamera::formats::SGBRG16)
		.def_readonly_static("SBGGR16", &libcamera::formats::SBGGR16)
		.def_readonly_static("R10_CSI2P", &libcamera::formats::R10_CSI2P)
		.def_readonly_static("R12_CSI2P", &libcamera::formats::R12_CSI2P)
		.def_readonly_static("SRGGB10_CSI2P", &libcamera::formats::SRGGB10_CSI2P)
		.def_readonly_static("SGRBG10_CSI2P", &libcamera::formats::SGRBG10_CSI2P)
		.def_readonly_static("SGBRG10_CSI2P", &libcamera::formats::SGBRG10_CSI2P)
		.def_readonly_static("SBGGR10_CSI2P", &libcamera::formats::SBGGR10_CSI2P)
		.def_readonly_static("SRGGB12_CSI2P", &libcamera::formats::SRGGB12_CSI2P)
		.def_readonly_static("SGRBG12_CSI2P", &libcamera::formats::SGRBG12_CSI2P)
		.def_readonly_static("SGBRG12_CSI2P", &libcamera::formats::SGBRG12_CSI2P)
		.def_readonly_static("SBGGR12_CSI2P", &libcamera::formats::SBGGR12_CSI2P)
		.def_readonly_static("SRGGB14_CSI2P", &libcamera::formats::SRGGB14_CSI2P)
		.def_readonly_static("SGRBG14_CSI2P", &libcamera::formats::SGRBG14_CSI2P)
		.def_readonly_static("SGBRG14_CSI2P", &libcamera::formats::SGBRG14_CSI2P)
		.def_readonly_static("SBGGR14_CSI2P", &libcamera::formats::SBGGR14_CSI2P)
		.def_readonly_static("SRGGB10_IPU3", &libcamera::formats::SRGGB10_IPU3)
		.def_readonly_static("SGRBG10_IPU3", &libcamera::formats::SGRBG10_IPU3)
		.def_readonly_static("SGBRG10_IPU3", &libcamera::formats::SGBRG10_IPU3)
		.def_readonly_static("SBGGR10_IPU3", &libcamera::formats::SBGGR10_IPU3)
		.def_readonly_static("RGGB_PISP_COMP1", &libcamera::formats::RGGB_PISP_COMP1)
		.def_readonly_static("GRBG_PISP_COMP1", &libcamera::formats::GRBG_PISP_COMP1)
		.def_readonly_static("GBRG_PISP_COMP1", &libcamera::formats::GBRG_PISP_COMP1)
		.def_readonly_static("BGGR_PISP_COMP1", &libcamera::formats::BGGR_PISP_COMP1)
		.def_readonly_static("MONO_PISP_COMP1", &libcamera::formats::MONO_PISP_COMP1)
	;
}
