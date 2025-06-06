import threading
import asyncio
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from open_gopro import WirelessGoPro, WiredGoPro, constants
from queue import Queue, Empty
import yaml

class GoProPublisher(Node):
    def __init__(self):
        super().__init__('gopro_node')

        # Load parameters from configuration file
        self.declare_parameter('connection_type', 'wireless')
        self.declare_parameter('fps', 30)
        self.declare_parameter('udp_port', 8554)
        self.declare_parameter('camera_serial', None)
        self.declare_parameter('frame_id', 'gopro_frame')
        self.declare_parameter('resolution', '1080p')
        self.declare_parameter('hypersmooth', None)
        self.declare_parameter('lens', None)
        self.declare_parameter('bitrate', None)
        self.declare_parameter('white_balance', None)
        self.declare_parameter('iso_min', None)
        self.declare_parameter('iso_max', None)
        self.declare_parameter('shutter', None)
        self.declare_parameter('ev_comp', None)
        self.declare_parameter('color', None)
        self.declare_parameter('sharpness', None)
        self.declare_parameter('audio_mode', None)
        self.declare_parameter('exposure_control', None)
        self.declare_parameter('zoom', None)
        self.declare_parameter('digital_lens', None)
        self.declare_parameter('media_mod', None)
        self.declare_parameter('protune', None)
        self.declare_parameter('spot_meter', None)
        self.declare_parameter('auto_low_light', None)
        self.declare_parameter('hdr', None)
        self.declare_parameter('raw_audio', None)
        self.declare_parameter('wind_noise_reduction', None)
        self.declare_parameter('mic_input', None)
        self.declare_parameter('gps', None)
        self.declare_parameter('leds', None)
        self.declare_parameter('beeps', None)
        self.declare_parameter('auto_off', None)
        self.declare_parameter('date_time', None)
        self.declare_parameter('orientation', None)
        self.declare_parameter('looping', None)
        self.declare_parameter('interval', None)
        self.declare_parameter('night_lapse', None)
        self.declare_parameter('night_lapse_exposure', None)
        self.declare_parameter('burst_rate', None)
        self.declare_parameter('photo_resolution', None)
        self.declare_parameter('photo_mode', None)
        self.declare_parameter('timelapse_interval', None)
        self.declare_parameter('timelapse_resolution', None)
        self.declare_parameter('timelapse_mode', None)
        self.declare_parameter('timelapse_speed', None)
        self.declare_parameter('timelapse_auto_interval', None)
        self.declare_parameter('timelapse_auto_shutter', None)
        self.declare_parameter('timelapse_auto_exposure', None)
        self.declare_parameter('timelapse_auto_iso', None)
        self.declare_parameter('timelapse_auto_white_balance', None)
        self.declare_parameter('timelapse_auto_ev_comp', None)
        self.declare_parameter('timelapse_auto_color', None)
        self.declare_parameter('timelapse_auto_sharpness', None)
        self.declare_parameter('timelapse_auto_audio', None)
        self.declare_parameter('timelapse_auto_gps', None)
        self.declare_parameter('timelapse_auto_leds', None)
        self.declare_parameter('timelapse_auto_beeps', None)
        self.declare_parameter('timelapse_auto_auto_off', None)
        self.declare_parameter('timelapse_auto_date_time', None)
        self.declare_parameter('timelapse_auto_orientation', None)
        self.declare_parameter('timelapse_auto_looping', None)
        self.declare_parameter('timelapse_auto_interval', None)
        self.declare_parameter('timelapse_auto_night_lapse', None)
        self.declare_parameter('timelapse_auto_night_lapse_exposure', None)
        self.declare_parameter('timelapse_auto_burst_rate', None)
        self.declare_parameter('timelapse_auto_photo_resolution', None)
        self.declare_parameter('timelapse_auto_photo_mode', None)

        self.fps = self.get_parameter('fps').value
        self.udp_port = self.get_parameter('udp_port').value
        self.camera_serial = self.get_parameter('camera_serial').value
        self.frame_id = self.get_parameter('frame_id').value
        self.connection_type = self.get_parameter('connection_type').value
        self.resolution = self.get_parameter('resolution').value
        self.hypersmooth = self.get_parameter('hypersmooth').value
        self.lens = self.get_parameter('lens').value
        self.bitrate = self.get_parameter('bitrate').value
        self.white_balance = self.get_parameter('white_balance').value
        self.iso_min = self.get_parameter('iso_min').value
        self.iso_max = self.get_parameter('iso_max').value
        self.shutter = self.get_parameter('shutter').value
        self.ev_comp = self.get_parameter('ev_comp').value
        self.color = self.get_parameter('color').value
        self.sharpness = self.get_parameter('sharpness').value
        self.audio_mode = self.get_parameter('audio_mode').value
        self.exposure_control = self.get_parameter('exposure_control').value
        self.zoom = self.get_parameter('zoom').value
        self.digital_lens = self.get_parameter('digital_lens').value
        self.media_mod = self.get_parameter('media_mod').value
        self.protune = self.get_parameter('protune').value
        self.spot_meter = self.get_parameter('spot_meter').value
        self.auto_low_light = self.get_parameter('auto_low_light').value
        self.hdr = self.get_parameter('hdr').value
        self.raw_audio = self.get_parameter('raw_audio').value
        self.wind_noise_reduction = self.get_parameter('wind_noise_reduction').value
        self.mic_input = self.get_parameter('mic_input').value
        self.gps = self.get_parameter('gps').value
        self.leds = self.get_parameter('leds').value
        self.beeps = self.get_parameter('beeps').value
        self.auto_off = self.get_parameter('auto_off').value
        self.date_time = self.get_parameter('date_time').value
        self.orientation = self.get_parameter('orientation').value
        self.looping = self.get_parameter('looping').value
        self.interval = self.get_parameter('interval').value
        self.night_lapse = self.get_parameter('night_lapse').value
        self.night_lapse_exposure = self.get_parameter('night_lapse_exposure').value
        self.burst_rate = self.get_parameter('burst_rate').value
        self.photo_resolution = self.get_parameter('photo_resolution').value
        self.photo_mode = self.get_parameter('photo_mode').value
        self.timelapse_interval = self.get_parameter('timelapse_interval').value
        self.timelapse_resolution = self.get_parameter('timelapse_resolution').value
        self.timelapse_mode = self.get_parameter('timelapse_mode').value
        self.timelapse_speed = self.get_parameter('timelapse_speed').value
        self.timelapse_auto_interval = self.get_parameter('timelapse_auto_interval').value
        self.timelapse_auto_shutter = self.get_parameter('timelapse_auto_shutter').value
        self.timelapse_auto_exposure = self.get_parameter('timelapse_auto_exposure').value
        self.timelapse_auto_iso = self.get_parameter('timelapse_auto_iso').value
        self.timelapse_auto_white_balance = self.get_parameter('timelapse_auto_white_balance').value
        self.timelapse_auto_ev_comp = self.get_parameter('timelapse_auto_ev_comp').value
        self.timelapse_auto_color = self.get_parameter('timelapse_auto_color').value
        self.timelapse_auto_sharpness = self.get_parameter('timelapse_auto_sharpness').value
        self.timelapse_auto_audio = self.get_parameter('timelapse_auto_audio').value
        self.timelapse_auto_gps = self.get_parameter('timelapse_auto_gps').value
        self.timelapse_auto_leds = self.get_parameter('timelapse_auto_leds').value
        self.timelapse_auto_beeps = self.get_parameter('timelapse_auto_beeps').value
        self.timelapse_auto_auto_off = self.get_parameter('timelapse_auto_auto_off').value
        self.timelapse_auto_date_time = self.get_parameter('timelapse_auto_date_time').value
        self.timelapse_auto_orientation = self.get_parameter('timelapse_auto_orientation').value
        self.timelapse_auto_looping = self.get_parameter('timelapse_auto_looping').value
        self.timelapse_auto_interval = self.get_parameter('timelapse_auto_interval').value
        self.timelapse_auto_night_lapse = self.get_parameter('timelapse_auto_night_lapse').value
        self.timelapse_auto_night_lapse_exposure = self.get_parameter('timelapse_auto_night_lapse_exposure').value
        self.timelapse_auto_burst_rate = self.get_parameter('timelapse_auto_burst_rate').value
        self.timelapse_auto_photo_resolution = self.get_parameter('timelapse_auto_photo_resolution').value
        self.timelapse_auto_photo_mode = self.get_parameter('timelapse_auto_photo_mode').value

        # --- Publisher for image ---
        self._bridge = CvBridge()
        qos_profile = rclpy.qos.QoSProfile(depth=5)
        qos_profile.best_effort()
        self.image_pub = self.create_publisher(Image, 'image_raw', qos_profile)

        # --- Thread-safe queue for raw CV frames ---
        self._frame_queue = Queue(maxsize=1)

        # --- Start background asyncio task to manage GoPro & streaming ---
        self._async_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._async_thread.start()

        # --- ROS Timer to publish frames at ~self.fps ---
        publish_period = 1.0 / self.fps
        self.create_timer(publish_period, self._publish_frame)

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self._on_parameter_update)

    def _run_async_loop(self):
        asyncio.run(self._async_main())

    async def _async_main(self):
        if self.connection_type == 'wireless':
            await self._connect_wireless()
        elif self.connection_type == 'wired':
            await self._connect_wired()
        else:
            self.get_logger().error('Invalid connection type specified.')

    async def _apply_gopro_settings(self, gopro):
        # Set all supported GoPro parameters if available
        try:
            if self.resolution:
                await gopro.http_command.set_video_resolution(self.resolution)
            if self.fps:
                await gopro.http_command.set_video_fps(self.fps)
            if self.hypersmooth:
                await gopro.http_command.set_hypersmooth(self.hypersmooth)
            if self.lens:
                await gopro.http_command.set_lens(self.lens)
            if self.bitrate:
                await gopro.http_command.set_bitrate(self.bitrate)
            if self.white_balance:
                await gopro.http_command.set_white_balance(self.white_balance)
            if self.iso_min:
                await gopro.http_command.set_iso_min(self.iso_min)
            if self.iso_max:
                await gopro.http_command.set_iso_max(self.iso_max)
            if self.shutter:
                await gopro.http_command.set_shutter(self.shutter)
            if self.ev_comp:
                await gopro.http_command.set_ev_comp(self.ev_comp)
            if self.color:
                await gopro.http_command.set_color(self.color)
            if self.sharpness:
                await gopro.http_command.set_sharpness(self.sharpness)
            if self.audio_mode:
                await gopro.http_command.set_audio_mode(self.audio_mode)
            if self.exposure_control:
                await gopro.http_command.set_exposure_control(self.exposure_control)
            if self.zoom:
                await gopro.http_command.set_zoom(self.zoom)
            if self.digital_lens:
                await gopro.http_command.set_digital_lens(self.digital_lens)
            if self.media_mod:
                await gopro.http_command.set_media_mod(self.media_mod)
            if self.protune:
                await gopro.http_command.set_protune(self.protune)
            if self.spot_meter:
                await gopro.http_command.set_spot_meter(self.spot_meter)
            if self.auto_low_light:
                await gopro.http_command.set_auto_low_light(self.auto_low_light)
            if self.hdr:
                await gopro.http_command.set_hdr(self.hdr)
            if self.raw_audio:
                await gopro.http_command.set_raw_audio(self.raw_audio)
            if self.wind_noise_reduction:
                await gopro.http_command.set_wind_noise_reduction(self.wind_noise_reduction)
            if self.mic_input:
                await gopro.http_command.set_mic_input(self.mic_input)
            if self.gps:
                await gopro.http_command.set_gps(self.gps)
            if self.leds:
                await gopro.http_command.set_leds(self.leds)
            if self.beeps:
                await gopro.http_command.set_beeps(self.beeps)
            if self.auto_off:
                await gopro.http_command.set_auto_off(self.auto_off)
            if self.date_time:
                await gopro.http_command.set_date_time(self.date_time)
            if self.orientation:
                await gopro.http_command.set_orientation(self.orientation)
            if self.looping:
                await gopro.http_command.set_looping(self.looping)
            if self.interval:
                await gopro.http_command.set_interval(self.interval)
            if self.night_lapse:
                await gopro.http_command.set_night_lapse(self.night_lapse)
            if self.night_lapse_exposure:
                await gopro.http_command.set_night_lapse_exposure(self.night_lapse_exposure)
            if self.burst_rate:
                await gopro.http_command.set_burst_rate(self.burst_rate)
            if self.photo_resolution:
                await gopro.http_command.set_photo_resolution(self.photo_resolution)
            if self.photo_mode:
                await gopro.http_command.set_photo_mode(self.photo_mode)
            if self.timelapse_interval:
                await gopro.http_command.set_timelapse_interval(self.timelapse_interval)
            if self.timelapse_resolution:
                await gopro.http_command.set_timelapse_resolution(self.timelapse_resolution)
            if self.timelapse_mode:
                await gopro.http_command.set_timelapse_mode(self.timelapse_mode)
            if self.timelapse_speed:
                await gopro.http_command.set_timelapse_speed(self.timelapse_speed)
            # Timelapse auto settings (if supported by SDK)
            if self.timelapse_auto_interval:
                await gopro.http_command.set_timelapse_auto_interval(self.timelapse_auto_interval)
            if self.timelapse_auto_shutter:
                await gopro.http_command.set_timelapse_auto_shutter(self.timelapse_auto_shutter)
            if self.timelapse_auto_exposure:
                await gopro.http_command.set_timelapse_auto_exposure(self.timelapse_auto_exposure)
            if self.timelapse_auto_iso:
                await gopro.http_command.set_timelapse_auto_iso(self.timelapse_auto_iso)
            if self.timelapse_auto_white_balance:
                await gopro.http_command.set_timelapse_auto_white_balance(self.timelapse_auto_white_balance)
            if self.timelapse_auto_ev_comp:
                await gopro.http_command.set_timelapse_auto_ev_comp(self.timelapse_auto_ev_comp)
            if self.timelapse_auto_color:
                await gopro.http_command.set_timelapse_auto_color(self.timelapse_auto_color)
            if self.timelapse_auto_sharpness:
                await gopro.http_command.set_timelapse_auto_sharpness(self.timelapse_auto_sharpness)
            if self.timelapse_auto_audio:
                await gopro.http_command.set_timelapse_auto_audio(self.timelapse_auto_audio)
            if self.timelapse_auto_gps:
                await gopro.http_command.set_timelapse_auto_gps(self.timelapse_auto_gps)
            if self.timelapse_auto_leds:
                await gopro.http_command.set_timelapse_auto_leds(self.timelapse_auto_leds)
            if self.timelapse_auto_beeps:
                await gopro.http_command.set_timelapse_auto_beeps(self.timelapse_auto_beeps)
            if self.timelapse_auto_auto_off:
                await gopro.http_command.set_timelapse_auto_auto_off(self.timelapse_auto_auto_off)
            if self.timelapse_auto_date_time:
                await gopro.http_command.set_timelapse_auto_date_time(self.timelapse_auto_date_time)
            if self.timelapse_auto_orientation:
                await gopro.http_command.set_timelapse_auto_orientation(self.timelapse_auto_orientation)
            if self.timelapse_auto_looping:
                await gopro.http_command.set_timelapse_auto_looping(self.timelapse_auto_looping)
            if self.timelapse_auto_night_lapse:
                await gopro.http_command.set_timelapse_auto_night_lapse(self.timelapse_auto_night_lapse)
            if self.timelapse_auto_night_lapse_exposure:
                await gopro.http_command.set_timelapse_auto_night_lapse_exposure(self.timelapse_auto_night_lapse_exposure)
            if self.timelapse_auto_burst_rate:
                await gopro.http_command.set_timelapse_auto_burst_rate(self.timelapse_auto_burst_rate)
            if self.timelapse_auto_photo_resolution:
                await gopro.http_command.set_timelapse_auto_photo_resolution(self.timelapse_auto_photo_resolution)
            if self.timelapse_auto_photo_mode:
                await gopro.http_command.set_timelapse_auto_photo_mode(self.timelapse_auto_photo_mode)
        except Exception as e:
            self.get_logger().warn(f'Failed to set some GoPro parameters: {e}')

    def _is_gopro_setting_param(self, param_name):
        # List of parameters that can be set dynamically (extend as needed)
        dynamic_params = [
            'resolution', 'fps', 'hypersmooth', 'lens', 'bitrate', 'white_balance', 'iso_min', 'iso_max',
            'shutter', 'ev_comp', 'color', 'sharpness', 'audio_mode', 'exposure_control', 'zoom',
            'digital_lens', 'media_mod', 'protune', 'spot_meter', 'auto_low_light', 'hdr', 'raw_audio',
            'wind_noise_reduction', 'mic_input', 'gps', 'leds', 'beeps', 'auto_off', 'date_time',
            'orientation', 'looping', 'interval', 'night_lapse', 'night_lapse_exposure', 'burst_rate',
            'photo_resolution', 'photo_mode', 'timelapse_interval', 'timelapse_resolution', 'timelapse_mode',
            'timelapse_speed', 'timelapse_auto_interval', 'timelapse_auto_shutter', 'timelapse_auto_exposure',
            'timelapse_auto_iso', 'timelapse_auto_white_balance', 'timelapse_auto_ev_comp', 'timelapse_auto_color',
            'timelapse_auto_sharpness', 'timelapse_auto_audio', 'timelapse_auto_gps', 'timelapse_auto_leds',
            'timelapse_auto_beeps', 'timelapse_auto_auto_off', 'timelapse_auto_date_time', 'timelapse_auto_orientation',
            'timelapse_auto_looping', 'timelapse_auto_night_lapse', 'timelapse_auto_night_lapse_exposure',
            'timelapse_auto_burst_rate', 'timelapse_auto_photo_resolution', 'timelapse_auto_photo_mode'
        ]
        return param_name in dynamic_params

    def _get_gopro_setter(self, param_name):
        # Map parameter name to GoPro SDK setter method name
        return {
            'resolution': 'set_video_resolution',
            'fps': 'set_video_fps',
            'hypersmooth': 'set_hypersmooth',
            'lens': 'set_lens',
            'bitrate': 'set_bitrate',
            'white_balance': 'set_white_balance',
            'iso_min': 'set_iso_min',
            'iso_max': 'set_iso_max',
            'shutter': 'set_shutter',
            'ev_comp': 'set_ev_comp',
            'color': 'set_color',
            'sharpness': 'set_sharpness',
            'audio_mode': 'set_audio_mode',
            'exposure_control': 'set_exposure_control',
            'zoom': 'set_zoom',
            'digital_lens': 'set_digital_lens',
            'media_mod': 'set_media_mod',
            'protune': 'set_protune',
            'spot_meter': 'set_spot_meter',
            'auto_low_light': 'set_auto_low_light',
            'hdr': 'set_hdr',
            'raw_audio': 'set_raw_audio',
            'wind_noise_reduction': 'set_wind_noise_reduction',
            'mic_input': 'set_mic_input',
            'gps': 'set_gps',
            'leds': 'set_leds',
            'beeps': 'set_beeps',
            'auto_off': 'set_auto_off',
            'date_time': 'set_date_time',
            'orientation': 'set_orientation',
            'looping': 'set_looping',
            'interval': 'set_interval',
            'night_lapse': 'set_night_lapse',
            'night_lapse_exposure': 'set_night_lapse_exposure',
            'burst_rate': 'set_burst_rate',
            'photo_resolution': 'set_photo_resolution',
            'photo_mode': 'set_photo_mode',
            'timelapse_interval': 'set_timelapse_interval',
            'timelapse_resolution': 'set_timelapse_resolution',
            'timelapse_mode': 'set_timelapse_mode',
            'timelapse_speed': 'set_timelapse_speed',
            'timelapse_auto_interval': 'set_timelapse_auto_interval',
            'timelapse_auto_shutter': 'set_timelapse_auto_shutter',
            'timelapse_auto_exposure': 'set_timelapse_auto_exposure',
            'timelapse_auto_iso': 'set_timelapse_auto_iso',
            'timelapse_auto_white_balance': 'set_timelapse_auto_white_balance',
            'timelapse_auto_ev_comp': 'set_timelapse_auto_ev_comp',
            'timelapse_auto_color': 'set_timelapse_auto_color',
            'timelapse_auto_sharpness': 'set_timelapse_auto_sharpness',
            'timelapse_auto_audio': 'set_timelapse_auto_audio',
            'timelapse_auto_gps': 'set_timelapse_auto_gps',
            'timelapse_auto_leds': 'set_timelapse_auto_leds',
            'timelapse_auto_beeps': 'set_timelapse_auto_beeps',
            'timelapse_auto_auto_off': 'set_timelapse_auto_auto_off',
            'timelapse_auto_date_time': 'set_timelapse_auto_date_time',
            'timelapse_auto_orientation': 'set_timelapse_auto_orientation',
            'timelapse_auto_looping': 'set_timelapse_auto_looping',
            'timelapse_auto_night_lapse': 'set_timelapse_auto_night_lapse',
            'timelapse_auto_night_lapse_exposure': 'set_timelapse_auto_night_lapse_exposure',
            'timelapse_auto_burst_rate': 'set_timelapse_auto_burst_rate',
            'timelapse_auto_photo_resolution': 'set_timelapse_auto_photo_resolution',
            'timelapse_auto_photo_mode': 'set_timelapse_auto_photo_mode',
        }.get(param_name, None)

    def _on_parameter_update(self, params):
        # Called when parameters are set at runtime
        results = []
        # Only update if GoPro is connected and http_command is available
        gopro = getattr(self, '_active_gopro', None)
        if gopro is None or not hasattr(gopro, 'http_command'):
            for p in params:
                results.append(rclpy.parameter.Parameter(p.name, p.type_, p.value))
            return rclpy.parameter.SetParametersResult(successful=True)
        async def update_param():
            for p in params:
                if self._is_gopro_setting_param(p.name):
                    setter = self._get_gopro_setter(p.name)
                    if setter and hasattr(gopro.http_command, setter):
                        try:
                            await getattr(gopro.http_command, setter)(p.value)
                            setattr(self, p.name, p.value)
                            self.get_logger().info(f"Updated GoPro setting: {p.name} -> {p.value}")
                        except Exception as e:
                            self.get_logger().warn(f"Failed to update {p.name}: {e}")
        # Schedule the update in the event loop
        asyncio.run_coroutine_threadsafe(update_param(), asyncio.get_event_loop())
        return rclpy.parameter.SetParametersResult(successful=True)

    # In _connect_wireless and _connect_wired, store the gopro object for dynamic updates
    async def _connect_wireless(self):
        async with WirelessGoPro(identifier=self.camera_serial) as gopro:
            self._active_gopro = gopro
            self.get_logger().info('Connected to GoPro (Wireless)')
            await self._apply_gopro_settings(gopro)
            preview_req = await gopro.http_command.start_preview()
            port = preview_req.data.udp_port
            cap = cv2.VideoCapture(f"udp://127.0.0.1:{port}", cv2.CAP_FOURCC(*'H264'))
            await self._stream_frames(cap, gopro)

    async def _connect_wired(self):
        async with WiredGoPro(identifier=self.camera_serial) as gopro:
            self._active_gopro = gopro
            self.get_logger().info('Connected to GoPro (Wired)')
            await self._apply_gopro_settings(gopro)
            preview_req = await gopro.http_command.start_preview()
            port = preview_req.data.udp_port
            cap = cv2.VideoCapture(f"udp://127.0.0.1:{port}", cv2.CAP_FOURCC(*'H264'))
            await self._stream_frames(cap, gopro)

    async def _stream_frames(self, cap, gopro):
        if not cap.isOpened():
            self.get_logger().error('Cannot open UDP stream.')
            return

        self.get_logger().info('Preview stream started.')

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                await asyncio.sleep(0.01)
                continue

            try:
                self._frame_queue.get_nowait()
            except Empty:
                pass
            self._frame_queue.put(frame)

        cap.release()
        await gopro.http_command.stop_preview()
        self.get_logger().info('Preview stopped.')

    def _publish_frame(self):
        try:
            frame = self._frame_queue.get_nowait()
        except Empty:
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoProPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()