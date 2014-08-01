%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%stop camera:
dc1394('capture_stop', camera);
dc1394('video_set_transmission', camera, 0);
dc1394('camera_free', camera);
clear camera;