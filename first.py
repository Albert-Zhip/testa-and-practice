import open3d as o3d
import numpy as np

# 创建一个随机点云
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.random.rand(5000, 3))

# 构建 Octree
max_depth = 4
octree = o3d.geometry.Octree(max_depth)
octree.convert_from_point_cloud(pcd, size_expand=0.01)

# 插入多个点
points = np.random.rand(100, 3)  # 生成 100 个随机点

def leaf_node_init():
    return o3d.geometry.OctreeColorLeafNode()

def leaf_node_update(node):
    pass

def internal_node_init():
    return o3d.geometry.OctreeInternalNode()

def internal_node_update(node):
    pass

for point in points:
    octree.insert_point(point, leaf_node_init, leaf_node_update, internal_node_init, internal_node_update)

# 查询 Octree 中的点
query_point = np.array([0.5, 0.5, 0.5])
success, node_info = octree.locate_leaf_node(query_point)
print("查询点:", query_point)
print("查询成功:", success)
print("节点信息:", node_info)

# 可视化 Octree 节点
def create_pointcloud_from_octree(octree):
    points = []
    colors = []
    def traverse(node, node_info):
        if isinstance(node, o3d.geometry.OctreeColorLeafNode):
            origin = node_info.origin
            size = node_info.size
            depth = node_info.depth
            # 根据深度设置颜色
            if depth == 0:
                color = [1, 0, 0]  # 红色
            elif depth == 1:
                color = [0, 1, 0]  # 绿色
            elif depth == 2:
                color = [0, 0, 1]  # 蓝色
            elif depth == 3:
                color = [1, 1, 0]  # 黄色
            else:
                color = [0, 1, 1]  # 青色
            points.append(origin + size / 2)
            colors.append(color)
        return False
    octree.traverse(traverse)
    pointcloud = o3d.geometry.PointCloud()
    pointcloud.points = o3d.utility.Vector3dVector(np.array(points))
    pointcloud.colors = o3d.utility.Vector3dVector(np.array(colors))
    return pointcloud

octree_pointcloud = create_pointcloud_from_octree(octree)

# 创建 Octree 的线框表示
def create_lineset_from_octree(octree):
    lines = []
    colors = []
    points = []
    def traverse(node, node_info):
        if isinstance(node, o3d.geometry.OctreeColorLeafNode) or isinstance(node, o3d.geometry.OctreeInternalNode):
            origin = node_info.origin
            size = node_info.size
            # 添加立方体的 12 条边
            cube_lines = [
                [0, 1], [1, 3], [3, 2], [2, 0],  # 底面
                [4, 5], [5, 7], [7, 6], [6, 4],  # 顶面
                [0, 4], [1, 5], [2, 6], [3, 7]   # 侧面
            ]
            cube_points = [
                origin,
                origin + [size, 0, 0],
                origin + [0, size, 0],
                origin + [size, size, 0],
                origin + [0, 0, size],
                origin + [size, 0, size],
                origin + [0, size, size],
                origin + [size, size, size]
            ]
            base_index = len(points)
            points.extend(cube_points)
            lines.extend([[base_index + start, base_index + end] for start, end in cube_lines])
            colors.extend([[0, 0, 0] for _ in range(len(cube_lines))])  # 黑色
        return False
    octree.traverse(traverse)
    lineset = o3d.geometry.LineSet()
    lineset.points = o3d.utility.Vector3dVector(np.array(points))
    lineset.lines = o3d.utility.Vector2iVector(np.array(lines))
    lineset.colors = o3d.utility.Vector3dVector(np.array(colors))
    return lineset

octree_lineset = create_lineset_from_octree(octree)

# 创建找到的立方体
if success:
    origin = node_info.origin
    size = node_info.size
    cube = o3d.geometry.TriangleMesh.create_box(width=size, height=size, depth=size)
    cube.translate(origin)
    cube.paint_uniform_color([1, 0, 0])  # 红色

# 使用默认的绘制函数来显示点云和 Octree
geometries = [octree_pointcloud, octree_lineset]
if success:
    geometries.append(cube)
o3d.visualization.draw_geometries(geometries)



