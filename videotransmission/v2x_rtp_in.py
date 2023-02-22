#!/usr/bin/env python3

import sys

import numpy as np

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
import threading
from udpsocket import *
from tlvmessage import *
import socket
import _thread

from tlvmessage import *

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
    source.set_property('address', udpsrc_address)
    source.set_property('port', udpsrc_port)
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


def v2x_to_loop(tlv_enable=False):
    """
    将从OBU接收到的包进行解包，发送到本地回环中。
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++
    | 8bit | 8bit | 16bit | 16bit |...| 16bit |  packet ...
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++
    第一个byte表示类型，第二个byte表示有几个包，后续的2byte表示对应的包的长度，新包在前旧包在后。
    :return:
    """
    tlv_en = tlv_enable
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # sock.settimeout(10)
    sock.bind(('192.168.62.224', 30301))

    remote_address = ('127.0.0.10', 30300)
    while True:
        raw_data = sock.recv(4096)
        if tlv_en:
            tlv_msg = TLVMessage(raw_data, RECEIVE)
            message = tlv_msg.get_payload()
        else:
            message = raw_data
        if message[0:1] == b'\x04':
            packet_num = int.from_bytes(message[1:2], 'big')
            packet_lengths = packet_num * [0]
            rtp_packets = []
            index = 2 + packet_num * 2
            for i in range(packet_num):
                packet_lengths[i] = int.from_bytes(message[2+i*2: 4+i*2], 'big')
                rtp_packets.append(message[index: index+packet_lengths[i]])
                index += packet_lengths[i]
            for _ in range(packet_num):
                send = sock.sendto(rtp_packets.pop(), remote_address)
                if send == 0:
                    raise RuntimeError("Socket connection broken!")


udpsrc_address = '127.0.0.10'
udpsrc_port = 30300
codec = 'H265'
stream_path0 = ('rtsp://192.168.10.140:8554/ds-test',)
stream_path1 = ('rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mp4',)

if __name__ == '__main__':
    try:
        _thread.start_new_thread(v2x_to_loop, (False,))
    except _thread.error:
        print("Unable to start thread: v2x_to_loop.")
    sys.exit(main(sys.argv))
