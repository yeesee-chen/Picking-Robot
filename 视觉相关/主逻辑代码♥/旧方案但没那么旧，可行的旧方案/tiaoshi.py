def replan_c_task(test_strings):
    """
    根据获得的c区数组，重新规划顺序

    Args:
        test_strings (str): 逗号分隔的数字字符串，表示任务序列

    Returns:
        list: 重新规划后的任务序列
    """
    if not test_strings:
        return []

    parts = test_strings.split(',')

    # 验证输入格式
    try:
        sequence = [int(x) for x in parts]
    except ValueError:
        print("错误：输入包含非数字字符")
        return []

    # 验证数字范围
    if not all(1 <= x <= 12 for x in sequence):
        print("错误：数字必须在1-12范围内")
        return []

    if len(sequence) != 8:
        print("错误：序列长度必须为8")
        return []

    result = []
    flag = 0  # 用于跟踪某种状态

    # 判断起始位置：1-4为右边，5-12为左边
    def is_right_side(pos):
        return pos in (1, 2, 3, 4)

    def is_left_side(pos):
        return pos in (5, 6, 7, 8, 9, 10, 11, 12)

    # 确定初始方向标志
    flag11 = 1 if is_right_side(sequence[0]) else 0

    for i in range(len(sequence)):
        c_now = sequence[i]
        c_next = sequence[i + 1] if i < len(sequence) - 1 else None

        # 处理位置1-8的情况
        if c_now in (1, 2, 3, 4, 5, 6, 7, 8):
            # 如果当前在右边且之前flag为0，则添加特殊指令
            if flag == 0 and flag11 == 1:
                result.append(20)
                flag = 1

            if c_next and c_next in (1, 2, 3, 4, 5, 6, 7, 8):
                if flag11 == 1:  # 在右边
                    c_now_id = c_now + 23
                    result.append(c_now_id)
                else:  # 在左边
                    if c_now in (1, 2, 3, 4):
                        c_now_id = c_now + 23
                        result.append(c_now_id)
                    elif c_now in (5, 6, 7, 8):
                        c_now_id = c_now + 27
                        result.append(c_now_id)
                        if c_next in (1, 2, 3, 4):
                            result.extend([21, 20])

            elif c_next and c_next in (9, 10, 11, 12):
                c_now_id = c_now + 23
                result.extend([c_now_id, 20, 21])

            else:
                # 处理序列末尾的情况
                if c_next is None:
                    c_now_id = c_now + 23
                    result.append(c_now_id)
                else:
                    print(f"未处理的情况：位置{i}，当前值{c_now}")

        # 处理位置9-12的情况
        elif c_now in (9, 10, 11, 12):
            c_now_id = c_now + 23
            result.append(c_now_id)

            if c_next:
                if c_next in (1, 2, 3, 4):
                    result.extend([21, 20])
                elif c_next in (5, 6, 7, 8):
                    pass  # 不需要额外操作
                # 对于c_next in (9, 10, 11, 12)的情况，不需要额外操作

        else:
            print(f"警告：位置{i}的值{c_now}超出有效范围")
            return result

    return result


def test_replan_function():
    """测试函数"""
    test_cases = [
        "1,5,7,6,3,4,11,10",
        "2,4,7,10,3,8,12,11",
        "7,5,1,4,9,10,6,2"
    ]

    for i, test_string in enumerate(test_cases, 1):
        print(f"\n=== 测试用例 {i}: {test_string} ===")
        result = replan_c_task(test_string)
        print(f"结果: {result}")
        print(f"结果长度: {len(result)}")


def replan_c_task_with_output(test_strings):
    """
    原始版本
    """
    if not test_strings:
        return

    parts = test_strings.split(',')
    flag = 0
    c_next = None

    for i in range(8):
        # flag11为0说明在左边，flag11为1说明在右边
        if int(parts[0]) in (5, 6, 7, 8, 9, 10, 11, 12):
            flag11 = 0
        if int(parts[0]) in (1, 2, 3, 4):
            flag11 = 1

        c_now = int(parts[i])
        if i <= 6:
            c_next = int(parts[i + 1])

        # 处理位置1-8
        if c_now in (1, 2, 3, 4, 5, 6, 7, 8):
            if flag == 0 and flag11 == 1:
                print(20)
                flag = 1

            if c_next and c_next in (1, 2, 3, 4, 5, 6, 7, 8):
                if flag11 == 1:
                    c_now_id = c_now + 23
                    print(c_now_id)
                else:
                    if c_now in (1, 2, 3, 4):
                        c_now_id = c_now + 23
                        print(c_now_id)
                    if c_now in (5, 6, 7, 8) and c_next in (1, 2, 3, 4):
                        c_now_id = c_now + 27
                        print(c_now_id)
                        print(21)
                        print(20)
                    if c_now in (5, 6, 7, 8) and c_next in (5, 6, 7, 8):
                        c_now_id = c_now + 27
                        print(c_now_id)

            elif c_next and c_next in (9, 10, 11, 12):
                c_now_id = c_now + 23
                print(c_now_id)
                print(20)
                print(21)

            else:
                if i < len(parts) - 1:  # 不是最后一个元素
                    print(f"位置{i}，当前值{c_now}未处理")

        # 处理位置9-12
        elif c_now in (9, 10, 11, 12):
            if c_next and c_next in (1, 2, 3, 4):
                c_now_id = c_now + 23
                print(c_now_id)
                print(21)
                print(20)
            elif c_next and c_next in (5, 6, 7, 8):
                c_now_id = c_now + 23
                print(c_now_id)
            elif c_next and c_next in (9, 10, 11, 12):
                c_now_id = c_now + 23
                print(c_now_id)
            else:
                # 处理序列末尾
                c_now_id = c_now + 23
                print(c_now_id)

        else:
            print(f"错误：位置{i}的值{c_now}超出范围")
            return


if __name__ == "__main__":
    # 运行测试
    test_replan_function()

    print("\n" + "=" * 50)
    print("原始输出格式测试:")
    test_strings = "2,4,7,10,3,8,12,11"
    print(f"输入: {test_strings}")
    replan_c_task_with_output(test_strings)