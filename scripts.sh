
# usage: camera_registration.py [-h] -a {PSM1,PSM2,PSM3} -m MARKER_SIZE [-i INTERVAL] -c CAMERA_IMAGE_TOPIC -t CAMERA_INFO_TOPIC
python camera_registration.py \
    -a PSM2 \
    -m 0.01 \
    -c /jhu_daVinci/decklink/left/image_rect_color \
    -t /jhu_daVinci/decklink/left/camera_info 
