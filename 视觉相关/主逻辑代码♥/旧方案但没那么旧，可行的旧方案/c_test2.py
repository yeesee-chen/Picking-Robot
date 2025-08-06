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
        if point in [37, 21]:
            return 0
        elif position in [1, 2, 3, 4]:
            return 2
        elif position in [9, 10, 11, 12]:
            return 1
        elif position in [5, 6, 7, 8]:
            if 24 <= point <= 27:
                return 1
            elif 32 <= point <= 35:
                return 2
        return 0  # 默认值

    def find_first_decisive_target(positions, start_index=1):
        """
        从指定索引开始，寻找第一个非5-8范围的目标
        返回该目标的位置值，如果都在5-8范围内则返回None
        """
        for i in range(start_index, len(positions)):
            if positions[i] not in [5, 6, 7, 8]:
                return positions[i]
        return None

    # 前置逻辑：根据第一个位置和后续位置确定初始点位
    if len(positions) >= 1:
        first_pos = positions[0]

        # 如果第一个位置在5-8之间，需要根据后续目标确定映射
        if 5 <= first_pos <= 8:
            decisive_target = find_first_decisive_target(positions, 1)

            if decisive_target is not None:
                if 1 <= decisive_target <= 4:
                    # 后续目标在1-4，第一个目标映射到24-27，前置为20,37
                    result.extend([20, 37])
                    obs_result.extend([0, 0])
                elif 9 <= decisive_target <= 12:
                    # 后续目标在9-12，第一个目标映射到32-35，前置为20,21
                    result.extend([20, 21])
                    obs_result.extend([0, 0])
                else:
                    # 其他情况，默认处理
                    result.extend([20, 21])
                    obs_result.extend([0, 0])
            else:
                # 所有目标都在5-8范围内，默认处理
                result.extend([20, 21])
                obs_result.extend([0, 0])

        # 原有的前置逻辑处理其他情况
        elif len(positions) >= 2:
            second_pos = positions[1]

            # 如果第一个和第二个都在1-8之间
            if 1 <= first_pos <= 8 and 1 <= second_pos <= 8:
                result.extend([20, 37])
                obs_result.extend([0, 0])
            # 如果第一个在5-12之间且第二个在9-12之间
            elif 5 <= first_pos <= 12 and 9 <= second_pos <= 12:
                result.extend([20, 21])
                obs_result.extend([0, 0])
            # 其他情况保持原逻辑
            elif first_pos in [1, 2, 3, 4]:
                result.extend([21, 37])
                obs_result.extend([0, 0])
            else:
                result.extend([20, 21])
                obs_result.extend([0, 0])
        else:
            # 如果位置数量少于2个，保持原逻辑
            if first_pos in [1, 2, 3, 4]:
                result.extend([21, 37])
                obs_result.extend([0, 0])
            else:
                result.extend([20, 21])
                obs_result.extend([0, 0])

    # 处理每个位置的点位映射
    for i, pos in enumerate(positions):
        current_target = None

        # 处理基本映射
        if pos in basic_mapping:
            current_target = basic_mapping[pos]

        # 处理模糊位置
        elif pos in ambiguous_mapping:
            option1, option2 = ambiguous_mapping[pos]

            # 如果是第一个位置且在5-8范围内，需要特殊处理
            if i == 0 and 5 <= pos <= 8:
                decisive_target = find_first_decisive_target(positions, 1)

                if decisive_target is not None:
                    if 1 <= decisive_target <= 4:
                        # 映射到24-27组
                        current_target = option1
                    elif 9 <= decisive_target <= 12:
                        # 映射到32-35组
                        current_target = option2
                    else:
                        # 默认选择第一组
                        current_target = option1
                else:
                    # 所有目标都在5-8范围内，默认选择第一组
                    current_target = option1

            else:
                # 其他情况保持原有逻辑
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
                result.extend([37, 21])
                obs_result.extend([0, 0])  # 中间点位都是观察状态0
            # 从32-35前往24-27
            elif 32 <= last_target <= 35 and 24 <= current_target <= 27:
                result.extend([21, 37])
                obs_result.extend([0, 0])  # 中间点位都是观察状态0

        # 添加当前目标点位和对应的观察状态
        if current_target is not None:
            result.append(current_target)
            obs_result.append(get_current_obs(pos, current_target))
            last_target = current_target

    # 结尾处理逻辑
    if result and 24 <= result[-1] <= 27:
        result.append(23)
        obs_result.append(0)
    # 如果最后一个在32-35之间，不添加任何点位

    return result, obs_result


# 测试函数
if __name__ == "__main__":
    # 测试用例
    test_cases = [
        "8,4,5,3,7,10,12,1",  # 测试用例
        "7,10,6,9,3,1,8,11"
    ]

    for test_input in test_cases:
        result, obs_result = replan_c_task(test_input)
        print(f"输入: {test_input}")
        print(f"点位输出: {result}")
        print(f"观察状态: {obs_result}")
        print("-" * 50)