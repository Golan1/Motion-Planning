from arr2_epec_seg_ex import Point_2
from arr2_epec_seg_ex import Polygon_2
from arr2_epec_seg_ex import minkowski_sum_by_full_convolution_2
import sys
from read_input import *


def generate_path(path, robot, obstacles, destination):
    print("generate_path input path = " + str(path))
    print("robot = " + str(robot))
    print("obstacles = " + str(obstacles))
    print("destination = " + str(destination))
    # TODO: Minkowski sum
    minus_rob = Polygon_2([Point_2(-1*x, -1*y) for x, y in robot])
    cgal_obstacles = []
    c_space_obstacles = []
    for curr_obc in obstacles:
        cgal_obstacles.append(Polygon_2([Point_2(x, y) for x, y in curr_obc]))
    for obs in cgal_obstacles:
        # I tried using the minkowski_sum_2 function it didn't work this seems to work
        c_space_obstacles.append(minkowski_sum_by_full_convolution_2(minus_rob, obs))
    for curr_c_obc in c_space_obstacles:
        print(curr_c_obc)
    # TODO: polygon union
    # TODO: vertical decomposition
    # TODO: create a graph
    # TODO: find start and goal vertices (in the graph)
    # TODO: BFS/DFS
    # TODO: convert graph path to movements
    # TODO: convert movements to path

    # path.append(Point_2(300, 400))
    # path.append(Point_2(300, 1000))
    # path.append(Point_2(700, 1000))
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
