

def update_config(path, newCameraMatrixL, distL, newCameraMatrixR, distR, resolution="[LEFT_CAM_FHD]"):

    with open(path, "r") as c:
        config = c.readlines()

    new_config = []
    j = 0
    for i in range(len(config)):

        if config[i].strip() == resolution:
            j = i
            found = True

    # newCameraMatrixL = [[688.6862793,0,450.12414623], [0, 624.61743164, 572.00437535], [  0,           0,           1.        ]]
    # distL = [[ 0.14409229, -0.09604006,  0.05570762, -0.03831593,  0.04128132]]
    # newCameraMatrixR = [[713.08691406,   0.,         444.40685773],[  0.,         642.05340576, 585.83490466],[  0.,           0.,           1.        ]]
    # distR = [[ 0.21390787, -0.18874744,  0.05569236, -0.04486877,  0.08374096]]

    if found:
        j += 1
        config[j] = f"fx={newCameraMatrixL[0][0]}\n"
        j += 1
        config[j] = f"fy={newCameraMatrixL[1][1]}\n"
        j += 1
        config[j] = f"cx={newCameraMatrixL[0][2]}\n"
        j += 1
        config[j] = f"cy={newCameraMatrixL[1][2]}\n"
        j += 1

        config[j] = f"k1={distL[0][0]}\n"
        j += 1

        config[j] = f"k2={distL[0][1]}\n"
        j += 1

        config[j] = f"k3={distL[0][4]}\n"
        j += 1

        config[j] = f"p1={distL[0][2]}\n"
        j += 1

        config[j] = f"p2={distL[0][3]}\n"
        j += 1

        config[j] = "\n"
        j += 1

        config[j] = "[RIGHT_CAM_FHD]\n"
        j += 1

        config[j] = f"fx={newCameraMatrixR[0][0]}\n"
        j += 1

        config[j] = f"fy={newCameraMatrixR[1][1]}\n"
        j += 1

        config[j] = f"cx={newCameraMatrixR[0][2]}\n"
        j += 1

        config[j] = f"cy={newCameraMatrixR[1][2]}\n"
        j += 1

        config[j] = f"k1={distR[0][0]}\n"
        j += 1

        config[j] = f"k2={distR[0][1]}\n"
        j += 1

        config[j] = f"k3={distR[0][4]}\n"
        j += 1

        config[j] = f"p1={distR[0][2]}\n"
        j += 1

        config[j] = f"p2={distR[0][3]}\n"
        j += 1

        config[j] = "\n"


    # for line in config:
    #     print(line)

