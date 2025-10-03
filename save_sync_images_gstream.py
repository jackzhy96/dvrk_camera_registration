import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp
import cv2
import numpy as np
import sys
import os

# Inizializza GStreamer
Gst.init(None)

def make_pipeline(device_number):
    """
    Create a GStreamer pipeline that captures video from a DeckLink device and sends it to an appsink.
    """
    pipeline_str = (
    f"v4l2src device={device_number} ! "
    "videoconvert ! "
    "videocrop left=0 right=0 top=60 bottom=60 ! "
    "video/x-raw, format=BGR ! "
    "appsink name=appsink"
    )
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("appsink")
    appsink.set_property("emit-signals", True)
    appsink.set_property("max-buffers", 1)
    appsink.set_property("drop", True)
    return pipeline, appsink

def gst_to_opencv(sample):
    """
    Convert a GStreamer buffer into an OpenCV frame (numpy array)
    """
    buf = sample.get_buffer()
    caps = sample.get_caps()
    arr = np.ndarray(
        shape=(caps.get_structure(0).get_value("height"),
               caps.get_structure(0).get_value("width"),
               3),
        buffer=buf.extract_dup(0, buf.get_size()),
        dtype=np.uint8)
    return arr

def main():
    # Create the directory to save images
    save_dir = os.path.join(os.getcwd(), "Images_GStream")
    os.makedirs(save_dir, exist_ok=True)

    # Create two pipelines
    pipeline1, appsink1 = make_pipeline("/dev/csr-left")
    pipeline2, appsink2 = make_pipeline("/dev/csr-right")

    # Start the pipelines
    pipeline1.set_state(Gst.State.PLAYING)
    pipeline2.set_state(Gst.State.PLAYING)

    print("Press ENTER to save frames (up to 33). Press 'q' to quit.")
    count = 0

    try:
        while count < 33:
            # Retrieve the current frames
            sample1 = appsink1.emit("pull-sample")
            sample2 = appsink2.emit("pull-sample")

            if sample1 is None or sample2 is None:
                continue

            frame1 = gst_to_opencv(sample1)
            #frame1 = cv2.resize(frame1, (1300, 1024))
            frame2 = gst_to_opencv(sample2)
            #frame2 = cv2.resize(frame2, (1300, 1024))
            w, h = frame1.shape[1], frame1.shape[0]
            #print(f"Frame size: {w}x{h}")
            # Show the frames in a window
            cv2.imshow("Camera left", frame1)
            cv2.imshow("Camera right", frame2)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("\r") or key == 13:  # ENTER
                cv2.imwrite(os.path.join(save_dir, f"frame1_{count:02d}.png"), frame1)
                cv2.imwrite(os.path.join(save_dir, f"frame2_{count:02d}.png"), frame2)
                print(f"Saved frame {count} in {save_dir}")
                count += 1

            elif key == ord("q"):  # quit
                print("Forced exit by user")
                break

    except KeyboardInterrupt:
        print("Manually interrupted")

    # Close everything
    pipeline1.set_state(Gst.State.NULL)
    pipeline2.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
