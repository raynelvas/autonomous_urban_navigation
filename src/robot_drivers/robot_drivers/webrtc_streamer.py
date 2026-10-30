#!/usr/bin/env python3
#webrtc_streamer.py - ROS2 node that serves as a bridge between ROS2 and a WebRTC server

import asyncio
import threading
import logging
from concurrent.futures import ThreadPoolExecutor
from enum import Enum

import cv2
import numpy as np
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from av import VideoFrame

from flask import Flask, request, jsonify
from flask_cors import CORS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

import sys

handler = logging.StreamHandler(sys.stdout)
handler.setLevel(logging.INFO)
formatter = logging.Formatter(
    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
handler.setFormatter(formatter)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc_streamer_multi")
logger.addHandler(handler)
logger.setLevel(logging.INFO)

# Mute other loggers
for lib in ['aioice', 'werkzeug', 'websockets', 'av']:
    logging.getLogger(lib).setLevel(logging.WARNING)

class StreamType(Enum):
    CAMERA = "camera"
    DEBUG = "debug" 

# ---------------- ROS2 multi-stream subscriber ----------------
class RosMultiStream(Node):
    def __init__(self):
        super().__init__("webrtc_stream")
        self.bridge = CvBridge()
        
        # Store the most recent frames for each stream.
        self.latest_frames = {
            StreamType.CAMERA: None,
            StreamType.DEBUG: None,
        }

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscriber for original RGB camera with BEST_EFFORT QoS
        self.subscription_camera = self.create_subscription(Image, "/image_raw", self.camera_listener,qos)

        # Subscriber for debug
        self.subscription_debug = self.create_subscription(Image,"/debug/camera_front",self.debug_listener,10) 

    def camera_listener(self, msg: Image):
        try:
            self.latest_frames[StreamType.CAMERA] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def debug_listener(self, msg: Image):
        try:
            self.latest_frames[StreamType.DEBUG] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            pass

    def get_latest_frame(self, stream_type: StreamType):
        return self.latest_frames.get(stream_type)
    
    def destroy_node(self):
        super().destroy_node()

# ---------------- WebRTC video track for multiple streams ----------------
class RosVideoTrack(VideoStreamTrack):
    def __init__(self, ros_node: RosMultiStream, stream_type: StreamType):
        super().__init__()
        self.ros_node = ros_node
        self.stream_type = stream_type

    async def recv(self):
        # Target ~30 FPS
        await asyncio.sleep(1 / 30.0)

        img = self.ros_node.get_latest_frame(self.stream_type)
        if img is None:
            # Frame placeholder
            img = np.zeros((288, 512, 3), dtype=np.uint8)

            # Messages based on stream type
            messages = {
                "CAMERA": "THERE IS NO CAMERA FRAME",
                "DEBUG": "THERE IS NO DEBUG FRAME",
            }
            
            stream_upper = self.stream_type.value.upper()
            message = messages.get(stream_upper, f"NO {stream_upper} FRAME")
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, message, 
                    (img.shape[1]//4, img.shape[0]//2),
                    font, 0.8, (255, 255, 255), 2)

        else:
            if img.dtype != np.uint8:
                img = img.astype(np.uint8)

        frame = VideoFrame.from_ndarray(img, format="bgr24")
        frame.pts, frame.time_base = await self.next_timestamp()
        
        return frame

# ---------------- WebRTC event loop ----------------
webrtc_loop = None
webrtc_executor = ThreadPoolExecutor(max_workers=2)

def setup_webrtc_loop():
    global webrtc_loop
    webrtc_loop = asyncio.new_event_loop()
    def run_loop():
        asyncio.set_event_loop(webrtc_loop)
        webrtc_loop.run_forever()
    t = threading.Thread(target=run_loop, daemon=True)
    t.start()

async def create_peer_connection_async(offer: RTCSessionDescription, track: RosVideoTrack):
    """    Create a WebRTC connection for a specific stream    """
    ice_servers = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
    pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=ice_servers))

    @pc.on("iceconnectionstatechange")
    async def on_ice_state_change():
        if pc.iceConnectionState == "failed":
            await pc.close()

    # Configure to send video
    pc.addTransceiver("video", direction="sendonly")
    pc.addTrack(track)

    # Establish remote offer and create answer
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return pc, pc.localDescription

# ---------------- Flask app with multiple endpoints ----------------
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

pcs = set()
ros_node = None

@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "status": "healthy",
        "ros_connected": ros_node is not None,
        "webrtc_loop_running": webrtc_loop is not None,
        "streams_available": ["camera", "debug", "segmentation"]
    })

# Endpoint for original camera stream
@app.route("/offer/camera", methods=["POST", "OPTIONS"])
def offer_camera():
    return handle_offer(StreamType.CAMERA)

# Endpoint for ddebug stream
@app.route("/offer/detection", methods=["POST", "OPTIONS"])
def offer_debug():
    return handle_offer(StreamType.DEBUG)

def handle_offer(stream_type: StreamType):
    if request.method == "OPTIONS":
        return ('', 200)

    try:
        params = request.get_json()
        if not params:
            return jsonify({"error": "No JSON body"}), 400
        if "sdp" not in params or "type" not in params:
            return jsonify({"error": "Missing sdp/type"}), 400

        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        # Create track for the specific stream
        track = RosVideoTrack(ros_node, stream_type)

        fut = asyncio.run_coroutine_threadsafe(
            create_peer_connection_async(offer, track),
            webrtc_loop
        )
        pc, local_desc = fut.result(timeout=30)
        pcs.add(pc)
        return jsonify({"sdp": local_desc.sdp, "type": local_desc.type})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# ---------------- main ----------------
def start_flask():
    app.run(host="0.0.0.0", port=8081, debug=False, use_reloader=False)

def main():
    global ros_node
    rclpy.init()
    ros_node = RosMultiStream()

    setup_webrtc_loop()

    flask_thread = threading.Thread(target=start_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for pc in list(pcs):
                asyncio.run_coroutine_threadsafe(pc.close(), webrtc_loop).result(timeout=5)
        except Exception:
            pass
        if webrtc_loop:
            webrtc_loop.call_soon_threadsafe(webrtc_loop.stop)
        webrtc_executor.shutdown(wait=False)

        try:
            ros_node.destroy_node()
            rclpy.shutdown()
        except:
            pass 

if __name__ == "__main__":
    main()