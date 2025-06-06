import asyncio
from open_gopro import WiredGoPro, constants

async def record_short_clip():
    # 1. Connect via USB Ethernet → automatically switch to HTTP over USB
    async with WiredGoPro() as gopro:
        # 2. Set resolution & lens via HTTP settings
        await gopro.http_setting.video_resolution.set(constants.settings.VideoResolution.NUM_4K)
        await gopro.http_setting.video_lens.set(constants.settings.VideoLens.LINEAR)

        # 3. Start recording (shutter ON), wait, then stop (shutter OFF)
        await gopro.http_command.set_shutter(shutter=constants.Toggle.ENABLE)
        await asyncio.sleep(2)  # record for 2 seconds
        await gopro.http_command.set_shutter(shutter=constants.Toggle.DISABLE)

        # 4. Fetch media list (HTTP), then download each file
        media_list = (await gopro.http_command.get_media_list()).data.files
        for item in media_list:
            await gopro.http_command.download_file(camera_file=item.filename)

asyncio.run(record_short_clip())
