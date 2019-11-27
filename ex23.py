from arr2_epec_seg_ex import *
import sys
from read_input import *

FREESPACE = 'freespace'
VISITED = 'visited'
DESTINATION = 'destination'


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


def merge_faces_by_freespace_flag(x, y):
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


def vertical_decompose(arr):
    assert isinstance(arr, Arrangement_2)

    d = []
    verticals = Arrangement_2()

    decompose(arr, d)
    for pair in d:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex,
        # that is, the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        for obj in pair[1]:
            if obj.is_vertex():
                v1 = Vertex()
                obj.get_vertex(v1)
                insert(verticals, Curve_2(Segment_2(v0.point(), v1.point())))
            elif obj.is_halfedge():
                he = Halfedge()
                obj.get_halfedge(he)
                v1 = Point_2(v0.point().x(), he.curve().line().y_at_x(v0.point().x()))
                insert(verticals, Curve_2(Segment_2(v0.point(), v1)))
            else:  # obj is a face
                # can only happen for the vertices of the bbox, so IGNORE
                pass

    res = Arrangement_2()
    verticals.unbounded_face().set_data({FREESPACE: True})
    overlay(arr, verticals, res, Arr_face_overlay_traits(merge_faces_by_freespace_flag))

    return res


# def extend_to_graph(arr):
#     assert isinstance(arr, Arrangement_2)
#
#     for f in arr.faces():
#         data = f.data()
#         data[EDGES] = set()
#         f.set_data(data)
#
#     for e in arr.edges():
#         assert isinstance(e, Halfedge)
#         if e.face().data()[FREESPACE] and e.face().data()[FREESPACE]:
#             # add a path through one of the endpoints of the edge
#             data = e.face().data()
#             data[EDGES].add((e.twin().face(), e.source()))
#             e.face().set_data(data)
#
#             data = e.twin().face().data()
#             data[EDGES].add((e.face(), e.source()))
#             e.twin().face().set_data(data)

def DFS(current_face):
    assert isinstance(current_face, Face)

    data = current_face.data()
    data[VISITED] = True
    current_face.set_data(data)

    if current_face.data().get(DESTINATION):
        return []

    for e in current_face.outer_ccb():
        assert isinstance(e, Halfedge)
        # f = Face().assign(e.face())
        f = e.twin().face()
        assert isinstance(f, Face)
        if f.data()[FREESPACE] and not f.data().get(VISITED):
            res = DFS(f)
            if res is not None:
                res.append(e.curve().source())
                return res
    return None


def create_bbox(c_space_obstacles, c_destination):
    max_x = max([max(c.outer_boundary().vertices(), key=lambda v: v.x()) for c in c_space_obstacles]).x()
    max_y = max([max(c.outer_boundary().vertices(), key=lambda v: v.y()) for c in c_space_obstacles]).y()
    min_x = min([min(c.outer_boundary().vertices(), key=lambda v: v.x()) for c in c_space_obstacles]).x()
    min_y = min([min(c.outer_boundary().vertices(), key=lambda v: v.y()) for c in c_space_obstacles]).y()

    EXTRA = 10
    max_x = max(max_x, c_destination.x()).to_double() + EXTRA
    max_y = max(max_y, c_destination.y()).to_double() + EXTRA
    min_x = min(min_x, c_destination.x()).to_double() - EXTRA
    min_y = min(min_y, c_destination.y()).to_double() - EXTRA

    bbox = Polygon_2([Point_2(a, b) for (a, b) in [(min_x, max_y), (max_x, max_y), (max_x, min_y), (min_x, min_y)]])

    arr = Arrangement_2()
    insert(arr, [Curve_2(e) for e in bbox.edges()])

    # the freespace is the bounded face
    for f in arr.faces():
        assert isinstance(f, Face)
        f.set_data({FREESPACE: not f.is_unbounded()})

    return arr


def remove_duplication(list):
    res = [list[0]]
    for x in list[1:]:
        if x != res[-1]:
            res.append(x)
    return res


def get_free_face(arrangement, point):
    face = Face()
    # locate can return a vertex or an edge or a face
    located_obj = locate(arrangement, point)
    if located_obj.is_vertex():
        ver = Vertex()
        located_obj.get_vertex(ver)
        he = Halfedge()
        for e in ver.incident_halfedges():
            if e.face().data()[FREESPACE]:
                he = e
                break
            if e.twin().face().data()[FREESPACE]:
                he = e.twin()
                break
        face = he.face()
    if located_obj.is_halfedge():
        he = Halfedge()
        located_obj.get_halfedge(he)
        if he.face().data()[FREESPACE]:
            face = he.face()
        else:
            face = he.twin().face()
    if located_obj.is_face():
        located_obj.get_face(face)
    return face


def generate_path(path, robot, obstacles, destination):
    print("robot = " + str(robot))
    print("obstacles = " + str(obstacles))
    print("destination = " + str(destination))
    ref = robot[0]
    c_destination = Point_2(destination[0] - ref[0], destination[1] - ref[1])
    minus_robot = Polygon_2([Point_2(-1 * x, -1 * y) for x, y in robot])

    cgal_obstacles = [Polygon_2([Point_2(x, y) for x, y in obs]) for obs in obstacles]
    c_space_obstacles = [minkowski_sum_by_full_convolution_2(minus_robot, obs) for obs in cgal_obstacles]
    c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
    fake_arr_as_bbox = create_bbox(c_space_obstacles, c_destination)
    single_arrangement = overlay_multiple_arrangements(c_space_arrangements + [fake_arr_as_bbox],
                                                       merge_faces_by_freespace_flag)

    print(single_arrangement.number_of_edges())
    print(single_arrangement.number_of_vertices())
    decomposed_arrangement = vertical_decompose(single_arrangement)
    # extend_to_graph(single_arrangement)

    print(decomposed_arrangement.number_of_edges())
    print(decomposed_arrangement.number_of_vertices())

    source_face = get_free_face(decomposed_arrangement, Point_2(0, 0))
    target_face = get_free_face(decomposed_arrangement, c_destination)

    data = target_face.data()
    data[DESTINATION] = True
    target_face.set_data(data)

    c_path = DFS(source_face)
    if c_path is None:
        print('did not find path')
        return None

    c_path.append(Point_2(0, 0))
    c_path.insert(0, c_destination)
    c_path = remove_duplication(c_path)

    translate = Aff_Transformation_2(Translation(), Vector_2(*ref))
    path.extend(translate.transform(p) for p in c_path[::-1])
    print(path)


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
