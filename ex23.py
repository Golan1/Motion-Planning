from arr2_epec_seg_ex import *
import sys
from read_input import *
import functools

FREESPACE = 'freespace'


def simple_polygon_to_arrangement(poly):
    assert isinstance(poly, Polygon_2)
    arr = Arrangement_2()
    insert(arr, [Curve_2(e) for e in poly.edges()])
    return arr


def polygon_with_holes_to_arrangement(poly):
    assert isinstance(poly, Polygon_with_holes_2)
    arr = Arrangement_2()
    insert(arr, [Curve_2(e) for e in poly.outer_boundary().edges()])

    # set the freespace flag for the only current two faces
    for f in arr.faces():
        assert isinstance(f, Face)
        f.set_data({FREESPACE: f.is_unbounded()})


    # TODO: test this with a complex polygon
    for hole in poly.holes():
        insert(arr, [Curve_2(e) for e in hole.edges()])

    for f in arr.faces():
        assert isinstance(f, Face)
        if f.data is None:
            f.set_data = {FREESPACE: True}
    return arr


def merge_faces_by_free_space_flag(x, y):
    return {FREESPACE: x[FREESPACE] and y[FREESPACE]}


def overlay_multiple_arrangements(arrs, face_merge_func):
    # def empty(x, y):
    #     return None
    #
    # traits = Arr_overlay_traits(empty, empty, empty, empty, empty, empty, empty, empty, empty, face_merge_func)

    final_arr = arrs[0]
    for arr in arrs[1:]:
        temp_res = Arrangement_2()

        overlay(final_arr, arr, temp_res, Arr_face_overlay_traits(face_merge_func))
        # overlay(final_arr, arr, temp_res, traits)
        final_arr = temp_res
    return final_arr


def locate(arr, point):
    assert isinstance(arr, Arrangement_2)
    assert isinstance(point, Point_2)
    return Arr_naive_point_location(arr).locate(point)


def vertical_decompose(obs):
    assert isinstance(obs, Arrangement_2)

    return obs

    d = {}
    decompose(obs, d)
    for pair in d:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex, that is, the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        for obj in pair[1]:
            if obj.is_vertex():
                v1 = Vertex()
                obj.get_vertex(v1)
            elif obj.is_halfedge():
                he = Halfedge()
                obj.get_halfedge(he)
            else:  # obj is a face
                f = Face()
                obj.get_face(f)


def generate_path(path, robot, obstacles, destination):
    print("robot = " + str(robot))
    print("obstacles = " + str(obstacles))
    print("destination = " + str(destination))

    c_destination = Point_2(destination[0] - robot[0][0], destination[1] - robot[0][1])
    minus_robot = Polygon_2([Point_2(-1 * x, -1 * y) for x, y in robot])

    cgal_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
    c_space_obstacles = [minkowski_sum_by_full_convolution_2(minus_robot, obs) for obs in cgal_obstacles]
    c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
    vertical_decomposition_obstacles = [vertical_decompose(obs) for obs in c_space_arrangements]
    single_arrangement = overlay_multiple_arrangements(vertical_decomposition_obstacles, merge_faces_by_free_space_flag)

    target_face = Face()
    locate(single_arrangement, c_destination).get_face(target_face)
    print(target_face.data())

    # TODO: polygon union - optional
    # TODO: Create arrangement
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
    print("input = " + str(sys.argv))
    # polygon_scene_input_file = sys.argv[1]
    polygon_scene_input_file = 'polygon_scene0.txt'
    scene = read_polygon_scene(polygon_scene_input_file)
    # print(scene)
    obc = []
    for i in range(2, len(scene)):
        obc.append(scene[i])
    path = []
    dest = (1000, 1000)  # TODO this is wrong
    generate_path(path, scene[1], obc, dest)
    print("path after generate_path = " + str(path))
