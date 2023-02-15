#!/usr/bin/env python3

import sys
sys.path.append("../")
from deepstreamapps.common.bus_call import bus_call
from deepstreamapps.common.is_aarch_64 import is_aarch64
import pyds
import platform
import math
import time
from ctypes import *
import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GLib
import configparser

import argparse

MAX_DISPLAY_LEN = 64
PGIE_CLASS_ID_VEHICLE = 0
PGIE_CLASS_ID_BICYCLE = 1
PGIE_CLASS_ID_PERSON = 2
PGIE_CLASS_ID_ROADSIGN = 3
MUXER_OUTPUT_WIDTH = 1920
MUXER_OUTPUT_HEIGHT = 1080
MUXER_BATCH_TIMEOUT_USEC = 4000000
TILED_OUTPUT_WIDTH = 1280
TILED_OUTPUT_HEIGHT = 720
GST_CAPS_FEATURES_NVMM = "memory:NVMM"
OSD_PROCESS_MODE = 0
OSD_DISPLAY_TEXT = 0
pgie_classes_str = ["Vehicle", "TwoWheeler", "Person", "RoadSign"]

# tiler_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
# and update params for drawing rectangle, object information etc.


def tiler_src_pad_buffer_probe(pad, info, u_data):
    frame_number = 0
    num_rects = 0
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    # Note that pyds.gst_buffer_get_nvds_batch_meta() expects the
    # C address of gst_buffer as input, which is obtained with hash(gst_buffer)
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        try:
            # Note that l_frame.data needs a cast to pyds.NvDsFrameMeta
            # The casting is done by pyds.NvDsFrameMeta.cast()
            # The casting also keeps ownership of the underlying memory
            # in the C code, so the Python garbage collector will leave
            # it alone.
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_number = frame_meta.frame_num
        l_obj = frame_meta.obj_meta_list
        num_rects = frame_meta.num_obj_meta
        obj_counter = {
            PGIE_CLASS_ID_VEHICLE: 0,
            PGIE_CLASS_ID_PERSON: 0,
            PGIE_CLASS_ID_BICYCLE: 0,
            PGIE_CLASS_ID_ROADSIGN: 0,
        }
        while l_obj is not None:
            try:
                # Casting l_obj.data to pyds.NvDsObjectMeta
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
            obj_counter[obj_meta.class_id] += 1
            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        print(
            "Frame Number=",
            frame_number,
            "Number of Objects=",
            num_rects,
            "Vehicle_count=",
            obj_counter[PGIE_CLASS_ID_VEHICLE],
            "Person_count=",
            obj_counter[PGIE_CLASS_ID_PERSON],
        )

        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK


def main(args):
    # Standard GStreamer initialization
    Gst.init(None)

    # Create gstreamer elements */
    # Create Pipeline element that will form a connection of other elements
    print("Creating Pipeline \n ")
    pipeline = Gst.Pipeline()
    is_live = False

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
    print("Creating streamux \n ")

    source = Gst.ElementFactory.make("udpsrc", "UDP-source")
    source.set_property('address', '192.168.62.224')
    source.set_property('port', 30301)
    pipeline.add(source)

    source.set_property("caps", Gst.Caps.from_string("application/x-rtp"))

    rtpdepay = None
    if codec == "H264":
        rtpdepay = Gst.ElementFactory.make("rtph264depay", "rtpdepay")
        print("Creating H264 rtppay")
    elif codec == "H265":
        rtpdepay = Gst.ElementFactory.make("rtph265depay", "rtpdepay")
        print("Creating H265 rtpdepay")
    if not rtpdepay:
        sys.stderr.write(" Unable to create rtpdepay")
    pipeline.add(rtpdepay)
    source.link(rtpdepay)

    decoder = Gst.ElementFactory.make("nvv4l2decoder", "decoder")
    print("Creating V4L2 Encoder")
    if not decoder:
        sys.stderr.write(" Unable to create decoder")
    pipeline.add(decoder)
    rtpdepay.link(decoder)

    streammux = Gst.ElementFactory.make("nvstreammux", "Stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")
    pipeline.add(streammux)
    padname = "sink_%u" % 0
    sinkpad = streammux.get_request_pad(padname)
    if not sinkpad:
        sys.stderr.write("Unable to create sink pad bin \n")
    srcpad = decoder.get_static_pad("src")
    if not srcpad:
        sys.stderr.write("Unable to create src pad bin \n")
    srcpad.link(sinkpad)

    streammux.set_property('live-source', True)
    streammux.set_property('width', 640)
    streammux.set_property('height', 480)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)

    print("Creating EGLSink \n")
    sink = Gst.ElementFactory.make("nveglglessink", "nvvideo-renderer")
    if not sink:
        sys.stderr.write(" Unable to create egl sink \n")

    # sink.set_property('async', False)
    sink.set_property('sync', False)
    pipeline.add(sink)
    # Finally render the osd output
    transform = None
    if is_aarch64():
        transform = Gst.ElementFactory.make("nvegltransform", "nvegl-transform")
        pipeline.add(transform)

    if is_aarch64():
        streammux.link(transform)
        transform.link(sink)
    else:
        streammux.link(sink)

    # create an event loop and feed gstreamer bus mesages to it
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)


    # start play back and listen to events
    print("Starting pipeline \n")
    pipeline.set_state(Gst.State.PLAYING)
    try:
        loop.run()
    except BaseException:
        pass
    # cleanup
    pipeline.set_state(Gst.State.NULL)


def parse_args():
    parser = argparse.ArgumentParser(description='RTSP Output Sample Application Help ')
    parser.add_argument("-i", "--input",
                  help="Path to input H264 elementry stream", nargs="+", default=["a"], required=True)
    parser.add_argument("-g", "--gie", default="nvinfer",
                  help="choose GPU inference engine type nvinfer or nvinferserver , default=nvinfer", choices=['nvinfer','nvinferserver'])
    parser.add_argument("-c", "--codec", default="H264",
                  help="RTSP Streaming Codec H264/H265 , default=H264", choices=['H264','H265'])
    parser.add_argument("-b", "--bitrate", default=4000000,
                  help="Set the encoding bitrate ", type=int)
    # Check input arguments
    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)
    args = parser.parse_args()
    global codec
    global bitrate
    global stream_path
    global gie
    gie = args.gie
    codec = args.codec
    bitrate = args.bitrate
    stream_path = args.input
    return stream_path


codec = 'H264'
stream_path0 = ('rtsp://192.168.10.140:8554/ds-test',)
stream_path1 = ('rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4',)

if __name__ == '__main__':
    # stream_path = parse_args()
    sys.exit(main(0))
