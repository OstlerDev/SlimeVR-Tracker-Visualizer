import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R

rr.init("magnetometer_recording", spawn=True)

log_data = np.loadtxt("2026-11-11_14-33-50_null_F024F9CC2DF90.log", delimiter=",", dtype=float)
tracker_data = np.loadtxt("2026-11-11_14-33-50_null_F024F9CC2DF90_QUAT.log", delimiter=",", dtype=float)
tracker_data_9d = np.loadtxt("2026-11-11_14-33-50_null_F024F9CC2DF90-quats_TEST.log", delimiter=",", dtype=float)
tracker_data_6d = np.loadtxt("2026-11-11_14-33-50_null_F024F9CC2DF90-quats-6d_TEST.log", delimiter=",", dtype=float)
hmd_data = np.loadtxt("2026-11-11_14-33-50_HEAD_230YC0XXXX00XX_QUAT.log", delimiter=",", dtype=float)

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

time_offset = 0
time_offset_rows = 1000000
time_offset_counter = 0

quat_offset = R.align_vectors([
    log_data[0][0] + hardiron[0], 
    log_data[0][1] + hardiron[1], 
    log_data[0][2] + hardiron[2]
], [[0, -1, 0]])[0] * R.from_euler('y', 180, degrees=True)

latest_time = 0
for row in log_data:
    if time_offset_counter < time_offset_rows:
        time_offset_counter+=1
        time_offset=row[0]
        continue
    if row[1] != 2:
        continue
    if row[0] - latest_time < 25:
        continue
    latest_time = row[0]
    rr.set_time("sensor_time", timestamp=(row[0])/1000)
    rr.log("mag/points", rr.Points3D(positions=np.array([ [ row[2], row[3], row[4] ] ])))

time_offset = 1794436258856

rr.set_time("sensor_time", timestamp=tracker_data[0][0]/1000)
rr.log("world/tracker-real", rr.Asset3D(path="./tracker.gltf", albedo_factor=[255, 0, 0, 255])) # red
rr.log("world/tracker-real", rr.Transform3D(translation=[hardiron[0]-10, hardiron[1], hardiron[2]+10], scale=15.0))
offset_Tracker = R.from_quat([ 1,0,0,0 ])
# offset_Tracker = R.from_quat([ tracker_data[0][1], tracker_data[0][2], tracker_data[0][3], tracker_data[0][4] ]).inv()
quat_tracker = offset_Tracker * R.from_quat([ tracker_data[0][1], tracker_data[0][2], tracker_data[0][3], tracker_data[0][4] ])
# rr.log("world/tracker-real", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker.as_quat())))
latest_time = 0
hasLogged = False
for row in tracker_data:
    if row[0] - latest_time < 100:
        continue
    latest_time = row[0]
    if row[0] < time_offset:
        offset_Tracker = R.from_quat([ row[1], row[2], row[3], row[4] ])
        offset_Tracker = quat_offset * offset_Tracker
        offset_Tracker = offset_Tracker.inv()
        continue
    rr.set_time("sensor_time", timestamp=row[0]/1000)
    quat_tracker = R.from_quat([ row[1], row[2], row[3], row[4] ]) * offset_Tracker
    rr.log("world/tracker-real", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker.as_quat())))
    if hasLogged == False:
        print(quat_tracker)
        hasLogged = True


rr.set_time("sensor_time", timestamp=tracker_data_9d[0][0]/1000)
rr.log("world/tracker-9d", rr.Asset3D(path="./tracker.gltf", albedo_factor=[0, 255, 255, 255])) # cyan
rr.log("world/tracker-9d", rr.Transform3D(translation=[hardiron[0], hardiron[1], hardiron[2]], scale=15.0))
offset_Tracker_9d = R.from_quat([ 1,0,0,0 ])
# quat_tracker_9d = offset_Tracker_9d * R.from_quat([ tracker_data_9d[0][1], tracker_data_9d[0][2], tracker_data_9d[0][3], tracker_data_9d[0][4] ])
# rr.log("world/tracker-9d", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker_9d.as_quat())))
latest_time = 0
hasLogged = False
for row in tracker_data_9d:
    if row[0] - latest_time < 100:
        continue
    latest_time = row[0]
    if row[0] < time_offset:
        offset_Tracker_9d = R.from_quat([ row[1], row[2], row[3], row[4] ])
        offset_Tracker_9d = quat_offset * offset_Tracker_9d
        offset_Tracker_9d = offset_Tracker_9d.inv()
        continue
    rr.set_time("sensor_time", timestamp=(row[0])/1000)
    quat_tracker_9d = R.from_quat([ row[1], row[2], row[3], row[4] ]) * offset_Tracker_9d
    # quat_tracker_9d = R.from_quat([ row[1], row[2], row[3], row[4] ])
    rr.log("world/tracker-9d", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker_9d.as_quat())))
    if hasLogged == False:
        print(quat_tracker_9d)
        hasLogged = True
# print(offset_Tracker_9d.as_quat())


rr.set_time("sensor_time", timestamp=tracker_data_6d[0][0]/1000)
rr.log("world/tracker-6d", rr.Asset3D(path="./tracker.gltf", albedo_factor=[255, 0, 255, 255])) # magenta
rr.log("world/tracker-6d", rr.Transform3D(translation=[hardiron[0], hardiron[1], hardiron[2]], scale=15.0))
offset_Tracker_6d = R.from_quat([ 1,0,0,0 ])
# offset_Tracker_6d = R.from_quat([ tracker_data_6d[0][1], tracker_data_6d[0][2], tracker_data_6d[0][3], tracker_data_6d[0][4] ]).inv()
# quat_tracker_6d = offset_Tracker_6d * R.from_quat([ tracker_data_6d[0][1], tracker_data_6d[0][2], tracker_data_6d[0][3], tracker_data_6d[0][4] ])
# rr.log("world/tracker-6d", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker_6d.as_quat())))
latest_time = 0
hasLogged = False
for row in tracker_data_6d:
    if row[0] - latest_time < 100:
        continue
    latest_time = row[0]
    if row[0] < time_offset:
        offset_Tracker_6d = R.from_quat([ row[1], row[2], row[3], row[4] ])
        offset_Tracker_6d = quat_offset * offset_Tracker_6d
        offset_Tracker_6d = offset_Tracker_6d.inv()
        continue
    rr.set_time("sensor_time", timestamp=(row[0])/1000)
    quat_tracker_6d = R.from_quat([ row[1], row[2], row[3], row[4] ]) * offset_Tracker_6d
    # quat_tracker_6d = R.from_quat([ row[1], row[2], row[3], row[4] ])
    rr.log("world/tracker-6d", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_tracker_6d.as_quat())))
    if hasLogged == False:
        print(quat_tracker_6d)
        hasLogged = True

rr.set_time("sensor_time", timestamp=hmd_data[0][0]/1000)
rr.log("world/hmd", rr.Asset3D(path="./tracker.gltf", albedo_factor=[0, 255, 0, 255])) # green
rr.log("world/hmd", rr.Transform3D(translation=[hardiron[0]+10, hardiron[1], hardiron[2]+10], scale=15.0))
offset_HMD = R.from_quat([ 1,0,0,0 ])
# offset_HMD = R.from_quat([ hmd_data[0][1], hmd_data[0][2], hmd_data[0][3], hmd_data[0][4] ]).inv()
# quat_hmd = offset_HMD * R.from_quat([ hmd_data[0][1], hmd_data[0][2], hmd_data[0][3], hmd_data[0][4] ])
# rr.log("world/hmd", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_hmd.as_quat())))
latest_time = 0
hasLogged = False
for row in hmd_data:
    if row[0] - latest_time < 100:
        continue
    latest_time = row[0]
    if row[0] < time_offset:
        offset_HMD = R.from_quat([ row[1], row[2], row[3], row[4] ])
        offset_HMD = quat_offset * offset_HMD
        offset_HMD = offset_HMD.inv()
        continue
    rr.set_time("sensor_time", timestamp=row[0]/1000)
    quat_hmd = offset_HMD * R.from_quat([ row[1], row[2], row[3], row[4] ])
    rr.log("world/hmd", rr.InstancePoses3D(quaternions=rr.Quaternion(xyzw=quat_hmd.as_quat())))
    if hasLogged == False:
        print(quat_hmd)
        hasLogged = True