def replan_c_task(test_strings):
    """
    根据获得的c区数组，重新规划顺序

    Args:
        test_strings (str): 逗号分隔的数字字符串，表示任务序列

    Returns:
        list: 重新规划后的任务序列
    """
    global c_now_id
    if not test_strings:
        return []
    parts = test_strings.split(',')
    # 验证输入格式
    try:
        sequence = [int(x) for x in parts]
    except ValueError:
        print("错误：输入包含非数字字符")
        return []
    result = []

    # 初始化last，now，next
    last_id = None
    now_id = None
    next_id = None

    if last_id == None:
        now_id = sequence[0]
        next_id = sequence[1]
        last_id = now_id
    else:
        