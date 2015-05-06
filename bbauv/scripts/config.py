import vision.cfg.laneConfig as laneConfig
import vision.cfg.binsConfig as binsConfig
import vision.cfg.pickupConfig as pickupConfig

import vision.cfg.pegsConfig as pegsConfig
import vision.cfg.rgbConfig as rgbConfig
import vision.cfg.torpedoConfig as torpedoConfig
import vision.cfg.sonarConfig as sonarConfig

botCamTopic = '/bot_camera/camera/image_raw'
frontCamTopic = '/front_camera/camera/image_raw'
botCamLocalTopic = '/bottomcam/camera/image_rect_color'
frontCamLocalTopic = ''

compassTopic = '/euler'
visionFilterTopic = '/Vision/image_filter_lynn'

screen = { 'width': 640, 'height': 480 }
