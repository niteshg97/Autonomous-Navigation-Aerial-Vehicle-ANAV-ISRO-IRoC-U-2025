# src/control_node.py
import time
import math
import cv2
import numpy as np

from ekf_2d import EKF2D
from pid_controller import PID
from mavlink_controller import MAVLinkController

# Import your YOLO detector and marker helper
# Make sure hovering_position_estimation.py is in the same folder or accessible via Python path
from hovering_position_estimation import YOLOv8Detector, draw_fixed_marker

# --- CONFIG ---
CAMERA_INDEX = 0            # change to 1 if you use external camera / adjust
PIXEL_TO_METER = 0.001      # calibrate: meters per pixel (e.g., 0.001 m = 0.1 cm per pixel)
TOLERANCE_M = 0.02          # 2 cm tolerance
MAX_VEL_MPS = 0.6           # max body-frame velocity command
MAV_CONNECTION = "udp:127.0.0.1:14540"  # change to your vehicle connection
# ----------------

def clamp(v, a, b):
    return max(a, min(b, v))

def main():
    # set up detector
    detector = YOLOv8Detector(model_path='yolov8n.pt')

    # video capture
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera index %s" % CAMERA_INDEX)

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Cannot read from camera")
    h, w = frame.shape[:2]
    fixed_position = (w//2, h//2)

    # ekf (dt will be computed each loop)
    ekf = EKF2D(dt=0.05, process_var=1e-3, meas_var=4.0)

    # simple PID controllers for body-frame vx, vy
    pid_x = PID(kp=0.8, ki=0.0, kd=0.05, out_min=-MAX_VEL_MPS, out_max=MAX_VEL_MPS)
    pid_y = PID(kp=0.8, ki=0.0, kd=0.05, out_min=-MAX_VEL_MPS, out_max=MAX_VEL_MPS)

    # mavlink
    mav = MAVLinkController(MAV_CONNECTION)
    print("MAVLink controller ready. Sending small zero velocity to ensure connection...")
    mav.send_velocity_body(0,0,0)

    prev_time = time.time()

    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                break
            now = time.time()
            dt = now - prev_time
            if dt <= 0: dt = 0.05
            ekf.set_dt(dt)
            prev_time = now

            # draw fixed marker
            draw_fixed_marker(frame, fixed_position)

            # detection (from user's detector)
            detections = detector.detect_image(frame)
            measured = None
            if detections:
                _, conf, cx, cy, bbox = detections[0]
                measured = (float(cx), float(cy))
                # update EKF with measurement
                ekf.predict(acc=(0.0, 0.0))
                xhat = ekf.update_position(measured[0], measured[1])
            else:
                # no visual measurement -> just predict
                ekf.predict(acc=(0.0, 0.0))
                xhat = ekf.get_state()
                xhat = np.array(xhat).reshape(-1,1)

            px = float(xhat[0,0])
            py = float(xhat[1,0])
            vx = float(xhat[2,0])
            vy = float(xhat[3,0])

            # compute displacement from fixed point in pixels -> convert to meters (body frame axes: x right, y down)
            dx_px = px - fixed_position[0]
            dy_px = py - fixed_position[1]
            # convert pixel to meters (camera plane). Depending on camera orientation, you may need to swap signs
            dx_m = dx_px * PIXEL_TO_METER
            dy_m = dy_px * PIXEL_TO_METER

            distance_m = math.hypot(dx_m, dy_m)
            angle_deg = math.degrees(math.atan2(dy_m, dx_m))

            # PID: we want to drive displacement to zero -> setpoint 0, measurement is dx_m
            vx_cmd = pid_x.compute(dx_m, now=time.time())  # command along body x (right)
            vy_cmd = pid_y.compute(dy_m, now=time.time())  # command along body y (forward/down depending on frame)

            # invert signs if necessary depending on vehicle coordinate frame
            # For PX4 body NED: x forward, y right, but we sent MAV_FRAME_BODY_NED above (vx,vy are forward,right)
            # Here we assume dx_m positive means object is to the right, to bring object to center we need to command vy? adapt in field test.
            # We'll map: body_x = -dy_m (forward/back), body_y = -dx_m (right/left) as an example mapping.
            body_vx = clamp(-dy_m * 1.0 + vx_cmd, -MAX_VEL_MPS, MAX_VEL_MPS)
            body_vy = clamp(-dx_m * 1.0 + vy_cmd, -MAX_VEL_MPS, MAX_VEL_MPS)

            # if within tolerance, send zero velocity
            if distance_m < TOLERANCE_M:
                body_vx = 0.0
                body_vy = 0.0

            # Send command via MAVLink (vz=0 for hover)
            mav.send_velocity_body(body_vx, body_vy, 0.0)

            # draw state
            pred_x = int(px)
            pred_y = int(py)
            cv2.circle(frame, (pred_x, pred_y), 6, (0,0,255), -1)
            cv2.putText(frame, f"{distance_m*100:.1f}cm {angle_deg:.1f}deg", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            cv2.putText(frame, f"cmd vx:{body_vx:.2f} vy:{body_vy:.2f}", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

            cv2.imshow("ANAV Control Node", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # maintain loop at reasonable rate
            time.sleep(max(0.0, 0.01))
    finally:
        cap.release()
        cv2.destroyAllWindows()
