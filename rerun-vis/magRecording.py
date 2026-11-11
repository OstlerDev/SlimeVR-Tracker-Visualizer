import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R

rr.init("magnetometer_recording", spawn=True)

log_data = np.loadtxt("2026-11-11_00-27-07_null_F024F9CC2DF90.log", delimiter=",", dtype=float)
tracker_data = np.loadtxt("2026-11-11_00-27-08_null_F024F9CC2DF90_QUAT.log", delimiter=",", dtype=float)
hmd_data = np.loadtxt("2026-11-11_00-27-08_HEAD_230YC0XXXX00XX_QUAT.log", delimiter=",", dtype=float)

hardiron = np.array([62.076767, -118.574814, -10.320274])
softiron = np.array([
    [2.722946, -0.091657, 0.072858],
    [-0.091657, 2.640528, -0.000081],
    [0.072858, -0.000081, 2.748402],
])
hm = 139.425720
evals, evecs = np.linalg.eigh(softiron)
if np.linalg.det(evecs) < 0:
    evecs[:, 2] *= -1
half_sizes = hm / evals
rot_quat = R.from_matrix(evecs).as_quat()
currentSecond = round(log_data[0][0]/1000)
rr.set_time("sensor_time", timestamp=currentSecond)
rr.log("mag/ellipsoid", rr.Ellipsoids3D(
        centers=[hardiron],
        half_sizes=[half_sizes], 
        quaternions=[rr.Quaternion(xyzw=rot_quat)],
        colors=[[0,0,255,128]],
    )
)

# secondsPoints = np.array([])
# for row in log_data:
#     if round(row[0]/1000) == currentSecond:
#         secondsPoints = np.append(secondsPoints, [ row[7], row[8], row[9] ])
#     else:
#         rr.set_time("sensor_time", timestamp=currentSecond)
#         rr.log("mag/points", rr.Points3D(positions=secondsPoints))
#         currentSecond=round(row[0]/1000)
#         secondsPoints = np.array([
#             [ row[7], row[8], row[9] ]
#         ])

for row in log_data:
    if row[0] < 1794383094874:
        continue
    if row[1] != 2:
        continue
    rr.set_time("sensor_time", timestamp=row[0]/1000)
    rr.log("mag/points", rr.Points3D(positions=np.array([ [ row[2], row[3], row[4] ] ])))

rr.set_time("sensor_time", timestamp=tracker_data[0][0]/1000)
rr.log("world/tracker", rr.Asset3D(path="./tracker.gltf"))
rr.log("world/tracker", rr.Transform3D(translation=[hardiron[0]-10, hardiron[1], hardiron[2]], scale=25.0))
offset_Tracker = R.from_quat([ tracker_data[0][1], tracker_data[0][2], tracker_data[0][3], tracker_data[0][4] ]).inv()
quat_tracker = offset_Tracker * R.from_quat([ tracker_data[0][1], tracker_data[0][2], tracker_data[0][3], tracker_data[0][4] ])
rr.log("world/tracker", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker.as_quat())))
for row in tracker_data:
    if row[0] < 1794383094874:
        offset_Tracker = R.from_quat([ row[1], row[2], row[3], row[4] ]).inv()
        continue
    rr.set_time("sensor_time", timestamp=row[0]/1000)
    quat_tracker = offset_Tracker * R.from_quat([ row[1], row[2], row[3], row[4] ])
    rr.log("world/tracker", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker.as_quat())))

rr.set_time("sensor_time", timestamp=hmd_data[0][0]/1000)
rr.log("world/hmd", rr.Asset3D(path="./tracker.gltf"))
rr.log("world/hmd", rr.Transform3D(translation=[hardiron[0]+10, hardiron[1], hardiron[2]], scale=25.0))
offset_HMD = R.from_quat([ hmd_data[0][1], hmd_data[0][2], hmd_data[0][3], hmd_data[0][4] ]).inv()
quat_hmd = offset_HMD * R.from_quat([ hmd_data[0][1], hmd_data[0][2], hmd_data[0][3], hmd_data[0][4] ])
rr.log("world/hmd", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_hmd.as_quat())))
for row in hmd_data:
    if row[0] < 1794383094874:
        offset_HMD = R.from_quat([ row[1], row[2], row[3], row[4] ]).inv()
        continue
    rr.set_time("sensor_time", timestamp=row[0]/1000)
    quat_hmd = offset_HMD * R.from_quat([ row[1], row[2], row[3], row[4] ])
    rr.log("world/hmd", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_hmd.as_quat())))