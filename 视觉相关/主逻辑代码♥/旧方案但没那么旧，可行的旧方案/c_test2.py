def replan_c_task(position_str):
    """
    将位置编号字符串转换为对应的点位顺序和观察状态

    参数:
    position_str: 位置编号字符串，如 "1,6,10,3,2,5,7,8"

    返回:
    tuple: (点位顺序列表, 观察状态列表)
    观察状态说明:
    - 0: 目标点位为20,21时
    - 1: 目标位置为1-4时，或目标位置为5-8且点位在32-35之间时
    - 2: 目标位置为9-12时，或目标位置为5-8且点位在24-27之间时
    """
    # 解析输入字符串
    positions = [int(x.strip()) for x in position_str.split(',')]

    # 基本映射规则
    basic_mapping = {
        1: 24, 2: 25, 3: 26, 4: 27,
        9: 32, 10: 33, 11: 34, 12: 35
    }

    # 模糊位置的两种选择
    ambiguous_mapping = {
        5: (24, 32),
        6: (25, 33),
        7: (26, 34),
        8: (27, 35)
    }

    result = []
    obs_result = []
    last_target = None

    def get_current_obs(position, point):
        """根据位置和点位计算观察状态"""
        if point in [20, 21]:
            return 0
        elif position in [1, 2, 3, 4]:
            return 1
        elif position in [9, 10, 11, 12]:
            return 2
        elif position in [5, 6, 7, 8]:
            if 24 <= point <= 27:
                return 2
            elif 32 <= point <= 35:
                return 1
        return 0  # 默认值

    # 如果第一个位置在1-4中，先加20
    if positions[0] in [1, 2, 3, 4]:
        result.append(20)
        obs_result.append(0)  # 点位20对应观察状态0

    for i, pos in enumerate(positions):
        current_target = None

        # 处理基本映射
        if pos in basic_mapping:
            current_target = basic_mapping[pos]

        # 处理模糊位置
        elif pos in ambiguous_mapping:
            option1, option2 = ambiguous_mapping[pos]

            if last_target is None:
                # 如果是第一个位置，默认选择第一组
                current_target = option1
            elif 24 <= last_target <= 27:
                # 上一个在24-27组，选择24-27组的点位
                current_target = option1
            elif 32 <= last_target <= 35:
                # 上一个在32-35组，选择32-35组的点位
                current_target = option2
            else:
                # 其他情况默认选择第一组
                current_target = option1

        # 检查是否需要添加中间点位
        if last_target is not None and current_target is not None:
            # 从24-27前往32-35
            if 24 <= last_target <= 27 and 32 <= current_target <= 35:
                result.extend([20, 21])
                obs_result.extend([0, 0])  # 中间点位都是观察状态0
            # 从32-35前往24-27
            elif 32 <= last_target <= 35 and 24 <= current_target <= 27:
                result.extend([21, 20])
                obs_result.extend([0, 0])  # 中间点位都是观察状态0

        # 添加当前目标点位和对应的观察状态
        if current_target is not None:
            result.append(current_target)
            obs_result.append(get_current_obs(pos, current_target))
            last_target = current_target

    return result, obs_result


# 测试函数
if __name__ == "__main__":
    # 测试用例
    test_input = "1,6,10,3,2,5,7,8"
    result, obs_result = replan_c_task(test_input)
    print(f"输入: {test_input}")
    print(f"点位输出: {result}")
    print(f"观察状态: {obs_result}")
