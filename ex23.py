from arr2_epec_seg_ex import Point_2
import sys
from read_input import *


def generate_path(path, robot, obstacles, destination):
    print("generate_path input path = " + str(path))
    print("robot = " + str(robot))
    print("obstacles = " + str(obstacles))
    print("destination = " + str(destination))
    path.append(Point_2(300, 400))
    path.append(Point_2(300, 1000))
    path.append(Point_2(700, 1000))
    pass


if __name__ == '__main__':
    print("start")
    print("input = "+ str(sys.argv))
    polygon_scene_input_file = sys.argv[1]
    scene = read_polygon_scene(polygon_scene_input_file)
    # print(scene)
    obc = []
    for i in range(2, len(scene)):
        obc.append(scene[i])
    path = []
    dest = (1000, 1000)  # TODO this is wrong
    generate_path(path, scene[1], obc, dest)
    print("path after generate_path = " + str(path))
    print("finn")
