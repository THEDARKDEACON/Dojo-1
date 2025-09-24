/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 *
 * Python bindings - Auto-generated properties
 *
 * This file is auto-generated. Do not edit.
 */

#include <libcamera/property_ids.h>

#include <pybind11/pybind11.h>

#include "py_main.h"

namespace py = pybind11;

class PyProperties
{
};

class PyDraftProperties
{
};

void init_py_properties_generated(py::module& m)
{
	auto properties = py::class_<PyProperties>(m, "properties");
	auto draft = py::class_<PyDraftProperties>(properties, "draft");


        draft.def_readonly_static("ColorFilterArrangement", static_cast<const libcamera::ControlId *>(&libcamera::properties::draft::ColorFilterArrangement));

        py::enum_<libcamera::properties::draft::ColorFilterArrangementEnum>(draft, "ColorFilterArrangementEnum")
                .value("RGGB", libcamera::properties::draft::RGGB)
                .value("GRBG", libcamera::properties::draft::GRBG)
                .value("GBRG", libcamera::properties::draft::GBRG)
                .value("BGGR", libcamera::properties::draft::BGGR)
                .value("RGB", libcamera::properties::draft::RGB)
                .value("MONO", libcamera::properties::draft::MONO)
        ;

        properties.def_readonly_static("Location", static_cast<const libcamera::ControlId *>(&libcamera::properties::Location));

        py::enum_<libcamera::properties::LocationEnum>(properties, "LocationEnum")
                .value("Front", libcamera::properties::CameraLocationFront)
                .value("Back", libcamera::properties::CameraLocationBack)
                .value("External", libcamera::properties::CameraLocationExternal)
        ;

        properties.def_readonly_static("Rotation", static_cast<const libcamera::ControlId *>(&libcamera::properties::Rotation));

        properties.def_readonly_static("Model", static_cast<const libcamera::ControlId *>(&libcamera::properties::Model));

        properties.def_readonly_static("UnitCellSize", static_cast<const libcamera::ControlId *>(&libcamera::properties::UnitCellSize));

        properties.def_readonly_static("PixelArraySize", static_cast<const libcamera::ControlId *>(&libcamera::properties::PixelArraySize));

        properties.def_readonly_static("PixelArrayOpticalBlackRectangles", static_cast<const libcamera::ControlId *>(&libcamera::properties::PixelArrayOpticalBlackRectangles));

        properties.def_readonly_static("PixelArrayActiveAreas", static_cast<const libcamera::ControlId *>(&libcamera::properties::PixelArrayActiveAreas));

        properties.def_readonly_static("ScalerCropMaximum", static_cast<const libcamera::ControlId *>(&libcamera::properties::ScalerCropMaximum));

        properties.def_readonly_static("SensorSensitivity", static_cast<const libcamera::ControlId *>(&libcamera::properties::SensorSensitivity));

        properties.def_readonly_static("SystemDevices", static_cast<const libcamera::ControlId *>(&libcamera::properties::SystemDevices));
}