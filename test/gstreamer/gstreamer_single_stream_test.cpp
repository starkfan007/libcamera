/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Vedant Paranjape
 *
 * gstreamer_single_stream_test.cpp - GStreamer single stream capture test
 */

#include <iostream>
#include <unistd.h>

#include <libcamera/base/utils.h>

#include "libcamera/internal/source_paths.h"

#include <gst/gst.h>

#include "test.h"

using namespace std;

extern "C" {
const char *__asan_default_options()
{
	/*
	 * Disable leak detection due to a known global variable initialization
	 * leak in glib's g_quark_init(). This should ideally be handled by
	 * using a suppression file instead of disabling leak detection.
	 */
	return "detect_leaks=false";
}
}

class GstreamerSingleStreamTest : public Test
{
protected:
	int init() override
	{
		/*
		 * GStreamer by default spawns a process to run the
		 * gst-plugin-scanner helper. If libcamera is compiled with ASan
		 * enabled, and as GStreamer is most likely not, this causes the
		 * ASan link order check to fail when gst-plugin-scanner
		 * dlopen()s the plugin as many libraries will have already been
		 * loaded by then. Fix this issue by disabling spawning of a
		 * child helper process when scanning the build directory for
		 * plugins.
		 */
		gst_registry_fork_set_enabled(false);

		/* Initialize GStreamer */
		g_autoptr(GError) errInit = NULL;
		if (!gst_init_check(nullptr, nullptr, &errInit)) {
			g_printerr("Could not initialize GStreamer: %s\n",
				   errInit ? errInit->message : "unknown error");

			return TestFail;
		}

		/*
		 * Remove the system libcamera plugin, if any, and add the
		 * plugin from the build directory.
		 */
		GstRegistry *registry = gst_registry_get();
		GstPlugin *plugin = gst_registry_lookup(registry, "libgstlibcamera.so");
		if (plugin) {
			gst_registry_remove_plugin(registry, plugin);
			gst_object_unref(plugin);
		}

		std::string path = libcamera::utils::libcameraBuildPath()
				 + "src/gstreamer";
		if (!gst_registry_scan_path(registry, path.c_str())) {
			g_printerr("Failed to add plugin to registry\n");
			gst_deinit();
			return TestFail;
		}

		/* Create the elements */
		libcameraSrc_ = gst_element_factory_make("libcamerasrc", "libcamera");
		convert0_ = gst_element_factory_make("videoconvert", "convert0");
		sink0_ = gst_element_factory_make("fakesink", "sink0");

		/* Create the empty pipeline_ */
		pipeline_ = gst_pipeline_new("test-pipeline");

		if (!pipeline_ || !convert0_ || !sink0_ || !libcameraSrc_) {
			g_printerr("Not all elements could be created. %p.%p.%p.%p\n",
				   pipeline_, convert0_, sink0_, libcameraSrc_);
			if (pipeline_)
				gst_object_unref(pipeline_);
			if (convert0_)
				gst_object_unref(convert0_);
			if (sink0_)
				gst_object_unref(sink0_);
			if (libcameraSrc_)
				gst_object_unref(libcameraSrc_);
			gst_deinit();

			return TestFail;
		}

		return TestPass;
	}

	void cleanup() override
	{
		gst_object_unref(pipeline_);
		gst_deinit();
	}

	int run() override
	{
		GstStateChangeReturn ret;

		/* Build the pipeline */
		gst_bin_add_many(GST_BIN(pipeline_), libcameraSrc_, convert0_, sink0_, NULL);
		if (gst_element_link_many(libcameraSrc_, convert0_, sink0_, NULL) != TRUE) {
			g_printerr("Elements could not be linked.\n");
			return TestFail;
		}

		/* Start playing */
		ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
		if (ret == GST_STATE_CHANGE_FAILURE) {
			g_printerr("Unable to set the pipeline to the playing state.\n");
			return TestFail;
		}

		/* Wait until error or EOS or timeout after 2 seconds */
		constexpr GstMessageType msgType =
			static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS);
		constexpr GstClockTime timeout = 2 * GST_SECOND;

		g_autoptr(GstBus) bus = gst_element_get_bus(pipeline_);
		g_autoptr(GstMessage) msg = gst_bus_timed_pop_filtered(bus, timeout, msgType);

		gst_element_set_state(pipeline_, GST_STATE_NULL);

		/* Parse error message */
		if (msg == NULL)
			return TestPass;

		switch (GST_MESSAGE_TYPE(msg)) {
		case GST_MESSAGE_ERROR:
			gstreamer_print_error(msg);
			break;
		case GST_MESSAGE_EOS:
			g_print("End-Of-Stream reached.\n");
			break;
		default:
			g_printerr("Unexpected message received.\n");
			break;
		}

		return TestFail;
	}

private:
	void gstreamer_print_error(GstMessage *msg)
	{
		g_autoptr(GError) err = NULL;
		g_autofree gchar *debug_info = NULL;

		gst_message_parse_error(msg, &err, &debug_info);
		g_printerr("Error received from element %s: %s\n",
			   GST_OBJECT_NAME(msg->src), err->message);
		g_printerr("Debugging information: %s\n",
			   debug_info ? debug_info : "none");
	}

	GstElement *pipeline_;
	GstElement *libcameraSrc_;
	GstElement *convert0_;
	GstElement *sink0_;
};

TEST_REGISTER(GstreamerSingleStreamTest)
