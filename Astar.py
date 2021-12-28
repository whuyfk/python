import numpy as np


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

# 此函数用于返回搜索的路径


def return_path(current_node, maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # 在这里，创建每个位置都有-1的初始化结果迷宫
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # 返回反向路径，因为需要显示从开始到结束的路径
    path = path[::-1]
    start_value = 0
    # 我们更新A-star search找到的从开始到结束的路径，每增加一步
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1
    return result


def search(maze, cost, start, end):
    """
        返回元组列表，作为给定迷宫中从给定起点到给定终点的路径
        ：参数迷宫：
        ：参数成本
        ：参数开始：
        ：参数结束：
        ：返回：
    """

    # 使用g、h和f的初始化值创建开始和结束节点
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # 初始化尚未访问和已访问列表
    # 在此列表中，将放置所有尚未访问的节点进行探索。
    # 从这里，我们将找到下一步要扩展的最低成本节点
    yet_to_visit_list = []
    # 在这个列表中，我们将把所有已经探测过的节点放在一起，这样我们就不会再探测它了
    visited_list = []

    # 添加开始节点
    yet_to_visit_list.append(start_node)

    # 添加停止条件。这是为了避免任何无限循环并停止
    # 经过合理数量的步骤后执行
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # 搜索什么方块。search的动作是左上右下
    # （4）从每个位置移动

    move = [[-1, 0],  # 上
            [0, -1],  # 左
            [1, 0],  # 下
            [0, 1]]  # 右

    """
        1） 我们首先通过比较所有f成本并选择成本最低的节点进行进一步扩展来获得当前节点
        2） 检查是否达到最大迭代次数。设置消息并停止执行
        3） 从尚未访问列表中删除所选节点，并将此节点添加到已访问列表中
        4） Perofmr目标测试并返回路径，否则执行以下步骤
        5） 对于选定节点，查找所有子节点（使用“移动”查找子节点）
        a） 获取所选节点的当前位置（这将成为子节点的父节点）
        b） 检查是否存在有效位置（边界将使少数节点无效）
        c） 如果任何节点是墙，则忽略该节点
        d） 添加到所选父节点的有效子节点列表
        对于所有子节点
        a） 若子节点在已访问列表中，则忽略它并尝试下一个节点
        b） 计算子节点g、h和f值
        c） 如果孩子在尚未访问列表中，则忽略它
        d） 否则将孩子移动到尚未访问列表
    """
    # “查找迷宫”有多少行和列
    no_rows, no_columns = np.shape(maze)

    # 循环直到找到终点

    while len(yet_to_visit_list) > 0:

        # 每次将任何节点从尚未访问列表引用到访问列表时，限制操作的计数器都会递增
        outer_iterations += 1

        # 获取当前节点
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # 如果我们到达这一点，返回路径，例如它可能没有解决方案或计算成本太高
        if outer_iterations > max_iterations:
            print("giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        # 将当前节点弹出到访问列表，添加到访问列表
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # 测试是否达到目标，如果是，则返回路径
        if current_node == end_node:
            return return_path(current_node, maze)

        # 从所有相邻的正方形生成子对象
        children = []

        for new_position in move:

            # 获取节点位置
            node_position = (
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # 确保在范围内（检查是否在迷宫边界内）
            if (node_position[0] > (no_rows - 1) or
                node_position[0] < 0 or
                node_position[1] > (no_columns - 1) or
                    node_position[1] < 0):
                continue

            # 确保可行走的地形
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # 创建新节点
            new_node = Node(current_node, node_position)

            # 追加
            children.append(new_node)

        # 循环子节点
        for child in children:

            # 子项在访问列表中（搜索整个访问列表）
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # 创建f、g和h值
            child.g = current_node.g + cost
            # 这里计算的启发式成本是使用欧氏距离
            child.h = (((child.position[0] - end_node.position[0]) ** 2) +
                       ((child.position[1] - end_node.position[1]) ** 2))

            child.f = child.g + child.h

            # 子节点已经在“尚未到访”名单中，g成本已经更低
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            # 将子节点添加到尚未访问列表中
            yet_to_visit_list.append(child)


if __name__ == '__main__':

    maze = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
                0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
            0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1,
            0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 1],
            [1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1,
            0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
    start = [29, 1]  # starting position
    end = [7, 20]  # ending position
    cost = 1  # cost per movement

    path = search(maze, cost, start, end)
    print(path)
