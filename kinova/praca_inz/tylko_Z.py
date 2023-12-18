import copy
import time
import argparse
import cv2 as cv
from pupil_apriltags import Detector
import math

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

def oblicz_odleglosc(punkt1, punkt2):
    x1, y1 = punkt1
    x2, y2 = punkt2
    odleglosc = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return odleglosc

def main():
    args = get_args()

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    cap = cv.VideoCapture("odl_300.mp4")

    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    while True:
        ret, image = cap.read()
        if not ret:
            break
        debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        tags = at_detector.detect(
            image,
            estimate_tag_pose=False,
            camera_params=None,
            tag_size=None,
        )

        debug_image = draw_tags(debug_image, tags)

        key = cv.waitKey(1)  # Oczekaj na naciśnięcie dowolnego klawisza

        if key == 27:  # ESC
            break

        #cv.imshow('AprilTag Detect Demo', debug_image)
    cap.release()
    cv.destroyAllWindows()

def prostok(a):
    p = 100
    return math.sqrt(a*a+p*p)

def draw_tags(
    image,
    tags,
):
    f_x = 653.682312
    f_y = 651.856018
    a = 4/3
    stala_do_dl_x = 200*f_x
    stala_do_dl_y = 200*f_y
    c_u = 311.753418
    c_v = 232.400955
    z = [0, 0, 0, 0]
    a = [0, 0, 0, 0]
    b = [0, 0, 0, 0]
    pz = [0, 0]
    c = 0
    for tag in tags:
        corners = tag.corners

        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

    a[0] = oblicz_odleglosc(corner_01, corner_02)
    a[1] = oblicz_odleglosc(corner_03, corner_02)
    a[2] = oblicz_odleglosc(corner_03, corner_04)
    a[3] = oblicz_odleglosc(corner_01, corner_04)
    print("roznica w px")
    print(a)
    pz[0] = oblicz_odleglosc(corner_01, corner_03)
    pz[1] = oblicz_odleglosc(corner_02, corner_04)
    print("przekatna w px")
    print(pz)
    b[0] = stala_do_dl_x / oblicz_odleglosc(corner_01, corner_02)
    b[1] = stala_do_dl_x / oblicz_odleglosc(corner_03, corner_02)
    b[2] = stala_do_dl_x / oblicz_odleglosc(corner_03, corner_04)
    b[3] = stala_do_dl_x / oblicz_odleglosc(corner_01, corner_04)
    print("odleglosc w mm")
    print(b)
    c = ((283*f_y)/pz[0] + (283*f_x)/pz[1]) / 2
    print("odleglosc srodka w mm")
    print(c)
    z[0] = (prostok(stala_do_dl_x / a[0]) + prostok(stala_do_dl_y / a[3])) / 2
    z[1] = (prostok(stala_do_dl_x / a[0]) + prostok(stala_do_dl_y / a[1])) / 2
    z[2] = (prostok(stala_do_dl_x / a[2]) + prostok(stala_do_dl_y / a[1])) / 2
    z[3] = (prostok(stala_do_dl_x / a[2]) + prostok(stala_do_dl_y / a[3])) / 2
    print("odleglosc w mm po sredniej")
    print(z)

    return image


if __name__ == '__main__':
    main()
