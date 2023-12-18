
import time
import argparse
import cv2 as cv
from pupil_apriltags import Detector
import numpy as np
import socket
import math

# Parametry komunikacji UDP
udp_host = "localhost"  # Adres docelowy hosta
udp_port = 12345  # Port docelowy

# Inicjalizacja gniazda UDP
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--families", type=str, default='tag36h11')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args

def prostok(a):
    p = 80
    return math.sqrt(a*a+p*p)

def oblicz_odleglosc(punkt1, punkt2):
    x1, y1 = punkt1
    x2, y2 = punkt2
    odleglosc = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return odleglosc

def det_tags(
    image,
    tags,
    dt,
    pre_err,
    end_points
):
    c_u = 311.753418
    c_v = 232.400955
    f_x = 653.682312
    f_y = 651.856018
    a_q = 4/3
    stala_do_dl_x = 160*f_x
    stala_do_dl_y = 160*f_y
    z = np.array([1, 1, 1, 1])
    a = [0, 0, 0, 0]
    poch_err = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    for tag in tags:
        corners = tag.corners
        print(corners)
        print("Tutaj")
        if len(corners) == 4:
            corner_01 = (int(corners[0][0]), int(corners[0][1]))
            corner_02 = (int(corners[1][0]), int(corners[1][1]))
            corner_03 = (int(corners[2][0]), int(corners[2][1]))
            corner_04 = (int(corners[3][0]), int(corners[3][1]))
        else:
            result = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            return result

    a[0] = oblicz_odleglosc(corner_01, corner_02)
    a[1] = oblicz_odleglosc(corner_03, corner_02)
    a[2] = oblicz_odleglosc(corner_03, corner_04)
    a[3] = oblicz_odleglosc(corner_01, corner_04)
    z[0] = (prostok(stala_do_dl_x / a[0]) + prostok(stala_do_dl_y / a[3])) / 2
    z[1] = (prostok(stala_do_dl_x / a[0]) + prostok(stala_do_dl_y / a[1])) / 2
    z[2] = (prostok(stala_do_dl_x / a[2]) + prostok(stala_do_dl_y / a[1])) / 2
    z[3] = (prostok(stala_do_dl_x / a[2]) + prostok(stala_do_dl_y / a[3])) / 2
    print("odl:")
    print(z)
    cur_points = np.array([(corner_01[0]-c_u)/(f_x*a_q), (float(corner_01[1])-c_v)/f_y, (float(corner_02[0])-c_u)/(f_x*a_q), (float(corner_02[1])-c_v)/f_y, (float(corner_03[0])-c_u)/(f_x*a_q), (float(corner_03[1])-c_v)/f_y, (float(corner_04[0])-c_u)/(f_x*a_q), (float(corner_04[1])-c_v)/f_y])

    err = cur_points - end_points
	    #if (abs(err[0])+abs(err[1])+abs(err[2])+abs(err[3])+abs(err[4])+abs(err[5])+abs(err[6])+abs(err[7])) < 1:
	     #   err = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    poch_err = (pre_err - err)/dt
    L = np.array([
		[-1/z[0], 0, corner_01[0]/z[0], corner_01[0]*corner_01[1], -1-(corner_01[0]*corner_01[0]), corner_01[1]],
		[0, -1/z[0], corner_01[1]/z[0], 1+(corner_01[1]*corner_01[1]), -corner_01[0]*corner_01[1], -corner_01[0]],
		[-1/z[1], 0, corner_02[0]/z[1], corner_02[0]*corner_02[1], -1-(corner_02[0]*corner_02[0]), corner_02[1]],
		[0, -1/z[1], corner_02[1]/z[1], 1+(corner_02[1]*corner_02[1]), -corner_02[0]*corner_02[1], -corner_02[0]],
		[-1/z[2], 0, corner_03[0]/z[2], corner_03[0]*corner_03[1], -1-(corner_03[0]*corner_03[0]), corner_03[1]],
		[0, -1/z[2], corner_03[1]/z[2], 1+(corner_03[1]*corner_03[1]), -corner_03[0]*corner_03[1], -corner_03[0]],
		[-1/z[3], 0, corner_04[0]/z[3], corner_04[0]*corner_04[1], -1-(corner_04[0]*corner_04[0]), corner_04[1]],
		[0, -1/z[3], corner_04[1]/z[3], 1+(corner_04[1]*corner_04[1]), -corner_04[0]*corner_04[1], -corner_04[0]]
	    ])
    pi_L = np.linalg.pinv(L)
    result = np.dot(pi_L, poch_err)
    print("sterowanie")
    print(result)
	  #  ster = f"{round(result[0])} {round(result[1])} {round(result[2])} {round(result[3])} {round(result[4])} {round(result[5])}"
	    # Wysyłanie do hosta i portu za pomocą protokołu UDP
	 #   udp_socket.sendto(ster.encode(), (udp_host, udp_port))

    return err


def main():
    args = get_args()

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    cap = cv.VideoCapture("rtsp://192.168.5.31/color")

    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    dt = 1
    f_x = 653.682312
    f_y = 651.856018
    a = 4/3
    c_u = 311.753418
    c_v = 232.400955

    end_points = np.array([(469-c_u)/(f_x*a), (112-c_v)/f_y, (116-c_u)/(f_x*a), (107-c_v)/f_y, (127-c_u)/(f_x*a), (443-c_v)/f_y, (456-c_u)/(f_x*a), (445-c_v)/f_y])
    pre_err = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    while True:
        start_time = time.time()

        ret, image = cap.read()
        if not ret:
            break

        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        pre_err = det_tags(image, tags, dt, pre_err, end_points)
        end_time = time.time()
        dt = end_time - start_time
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
